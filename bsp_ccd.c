#include "bsp_ccd.h"

// 全局变量定义
uint16_t ADV[128] = {0};
uint8_t CCD_median = 64;     // 初始化为中间位置
uint8_t CCD_threshold = 128; // 初始化为中间值
uint8_t ADC_128X32[128] = {0};
CCD_Process ccd = {.exposure_time = 10, .stable_count = 0};

// 快速聚类分析结构体
typedef struct {
  uint32_t sum;   // 数值和
  uint16_t count; // 点数
  uint16_t mean;  // 均值
} Cluster;

// ADC采样函数
static uint16_t Get_Adc_CCD(uint8_t ch) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ch;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 500);
  return HAL_ADC_GetValue(&hadc3);
}

// 动态延时函数
static void Dly_us(void) {
  if (ccd.exposure_time <= 1)
    return;
  for (int i = 0; i < ccd.exposure_time - 1; i++) {
    for (int j = 0; j < 10; j++)
      ;
  }
}

// 数据平滑处理
static void Smooth_Data(void) {
  static uint16_t smooth_buffer[128];

  // 5点加权移动平均，中心点权重最大
  const uint8_t weights[5] = {1, 2, 3, 2, 1};
  const uint8_t total_weight = 9; // 权重和

  for (int i = 0; i < 128; i++) {
    uint32_t sum = 0;
    uint8_t weight_sum = 0;

    // 对每个点进行加权平均
    for (int j = -2; j <= 2; j++) {
      if (i + j >= 0 && i + j < 128) {
        sum += ccd.raw_data[i + j] * weights[j + 2];
        weight_sum += weights[j + 2];
      }
    }

    // 保存平滑后的数据
    smooth_buffer[i] = sum / weight_sum;
  }

  // 更新原始数据为平滑后的数据
  memcpy(ccd.raw_data, smooth_buffer, sizeof(smooth_buffer));
}

// 采集CCD数据
void RD_TSL(void) {
  uint8_t i = 0, tslp = 0;
  TSL_CLK = 1;
  TSL_SI = 0;
  Dly_us();

  TSL_SI = 1;
  TSL_CLK = 0;
  Dly_us();

  TSL_CLK = 1;
  TSL_SI = 0;
  Dly_us();

  for (i = 0; i < 128; i++) {
    TSL_CLK = 0;
    Dly_us();
    Dly_us();

    uint16_t value = (Get_Adc_CCD(CCD_ADC_CH)) >> 4;
    ADV[tslp] = value;
    ccd.raw_data[tslp] = value;
    ++tslp;
    TSL_CLK = 1;
    Dly_us();
  }

  // 在采集后立即进行平滑处理
  Smooth_Data();
}

// 更新曝光时间
static void Update_Exposure_Time(void) {
  if (ccd.max_value > TARGET_MAX_VALUE) {
    if (ccd.exposure_time > MIN_EXPOSURE_TIME) {
      ccd.exposure_time--;
      ccd.stable_count = 0;
    }
  } else if (ccd.max_value < TARGET_MIN_VALUE) {
    if (ccd.exposure_time < MAX_EXPOSURE_TIME) {
      ccd.exposure_time++;
      ccd.stable_count = 0;
    }
  } else if (ccd.stable_count < 255) {
    ccd.stable_count++;
  }
}

// 查找CCD中线
static void Find_CCD_Median(void) {
  // 1. 快速统计特征
  uint32_t total_sum = 0;
  uint16_t global_mean;
  Cluster low_cluster = {0, 0, 0};
  Cluster high_cluster = {0, 0, 0};

  // 计算总体均值
  for (int i = 0; i < 128; i++) {
    total_sum += ccd.raw_data[i];
  }
  global_mean = total_sum / 128;

  // 2. 初步分类
  for (int i = 0; i < 128; i++) {
    if (ccd.raw_data[i] < global_mean) {
      low_cluster.sum += ccd.raw_data[i];
      low_cluster.count++;
    } else {
      high_cluster.sum += ccd.raw_data[i];
      high_cluster.count++;
    }
  }

  // 计算两个聚类的均值
  low_cluster.mean =
      low_cluster.sum / (low_cluster.count > 0 ? low_cluster.count : 1);
  high_cluster.mean =
      high_cluster.sum / (high_cluster.count > 0 ? high_cluster.count : 1);

  // 3. 改进的动态阈值计算
  uint16_t value_range = high_cluster.mean - low_cluster.mean;
  float threshold_ratio;

  // 根据数值范围动态调整阈值比例
  if (high_cluster.mean > 800) { // 高光照条件
    threshold_ratio = 0.3f;
  } else if (high_cluster.mean < 400) { // 低光照条件
    threshold_ratio = 0.7f;
  } else {
    threshold_ratio = 0.5f;
  }

  // 计算最终阈值
  uint16_t threshold =
      low_cluster.mean + (uint16_t)(value_range * threshold_ratio);
  CCD_threshold = threshold;

  // 4. 边缘检测和黑线定位
  int8_t start_pos = -1;
  uint8_t current_width = 0;
  int8_t best_start = -1;
  uint8_t best_width = 0;
  uint16_t best_quality = 0;

  // 计算一阶导数（边缘检测）
  int16_t derivatives[127];
  for (int i = 0; i < 127; i++) {
    derivatives[i] = ccd.raw_data[i + 1] - ccd.raw_data[i];
  }

  // 单遍扫描查找最佳黑线段
  for (int i = 0; i < 128; i++) {
    if (ccd.raw_data[i] < threshold) {
      if (start_pos == -1) {
        // 检查左边缘的梯度
        if (i > 0 && derivatives[i - 1] < -20) { // 明显的下降边缘
          start_pos = i;
        }
      }
      current_width++;
    } else {
      if (current_width > 0) {
        // 检查右边缘的梯度
        if (i < 127 && derivatives[i] > 20) { // 明显的上升边缘
          // 找到一段黑线，检查是否是最佳的
          if (current_width > 3 && current_width < 40) {
            // 计算这段区域的均值和方差
            uint32_t segment_sum = 0;
            uint32_t segment_sq_sum = 0;
            for (int j = start_pos; j < start_pos + current_width; j++) {
              segment_sum += ccd.raw_data[j];
              segment_sq_sum += ccd.raw_data[j] * ccd.raw_data[j];
            }
            uint16_t segment_mean = segment_sum / current_width;
            uint32_t variance =
                segment_sq_sum / current_width - segment_mean * segment_mean;

            // 计算质量分数：考虑方差、与均值的差异、边缘强度和位置
            uint16_t mean_diff = ABS(segment_mean - low_cluster.mean);
            uint16_t edge_strength =
                ABS(derivatives[start_pos - 1]) +
                ABS(derivatives[start_pos + current_width - 1]);
            uint16_t center_dist = ABS(start_pos + current_width / 2 - 64);
            uint16_t quality = (mean_diff * 2) + (edge_strength / 2) -
                               (variance / 100) - (center_dist * 2);

            if (variance < 1000 && quality > best_quality) {
              best_start = start_pos;
              best_width = current_width;
              best_quality = quality;
            }
          }
        }
        start_pos = -1;
        current_width = 0;
      }
    }
  }

  // 5. 更新结果
  if (best_start != -1) {
    ccd.left_edge = best_start;
    ccd.right_edge = best_start + best_width;
    ccd.line_width = best_width;
    CCD_median = best_start + best_width / 2;

    // 更新调试信息
    ccd.max_value = high_cluster.mean;
    ccd.min_value = low_cluster.mean;
  } else {
    ccd.line_width = 0; // 标记丢线
  }
}

// 处理CCD数据
void Deal_Data_CCD(void) {
  RD_TSL();               // 采集并平滑数据
  Find_CCD_Median();      // 使用平滑后的数据进行中线检测
  Update_Exposure_Time(); // 更新曝光时间
}

// 获取显示数据
uint8_t *CCD_Get_ADC_128X32(void) {
  // 直接将平滑后的数据转换为显示格式
  for (int i = 0; i < 128; i++) {
    uint16_t value = ccd.raw_data[i];
    ADC_128X32[i] = (value > 255) ? 31 : (value >> 3);
  }
  return ADC_128X32;
}

// 显示CCD数据
void Print_CCD_data(void) {
  // 获取最新数据（已经平滑）
  Deal_Data_CCD();

  // 显示基本信息
  printf("\r\nDetailed CCD Analysis\r\n");
  printf("********************************\r\n");
  printf("Basic Info:\r\n");
  printf("Median: %d  Width: %d\r\n", CCD_median, ccd.line_width);
  printf("Threshold: %d  Max: %d  Min: %d\r\n", CCD_threshold, ccd.max_value,
         ccd.min_value);
  printf("Exposure: %d  Stable: %d\r\n", ccd.exposure_time, ccd.stable_count);

  // 显示波形图
  printf("\r\nSignal Waveform:\r\n");
  for (int i = 0; i < 128; i++) {
    char display_char;
    if (i == CCD_median) {
      display_char = 'M';
    } else if (i == ccd.left_edge || i == ccd.right_edge) {
      display_char = '|';
    } else {
      display_char = (ccd.raw_data[i] < CCD_threshold) ? '_' : '*';
    }
    printf("%c", display_char);
    if ((i + 1) % 32 == 0)
      printf("\r\n");
  }

  // 显示原始数据
  printf("\r\nProcessed Data:\r\n");
  for (uint8_t i = 0; i < 128; i++) {
    if (i % 8 == 0)
      printf("\r\n");
    printf("[%d]  ", ccd.raw_data[i]);
  }

  // 更新OLED显示
  OLED_Show_CCD_Image(CCD_Get_ADC_128X32());
}
void OLED_Show_CCD_Image(uint8_t *p_img) {
  OLED_Clear();
  for (int i = 0; i < 128; i++) {
    if (p_img[i] < 32) {
      SSD1306_DrawPixel(i, p_img[i], SSD1306_COLOR_WHITE);
    }
  }
  OLED_Refresh();
}