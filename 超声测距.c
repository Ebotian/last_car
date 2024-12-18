#include "bsp.h"

// 卡尔曼滤波器结构体
typedef struct {
  float Q; // 过程噪声协方差
  float R; // 测量噪声协方差
  float P; // 估计误差协方差
  float K; // 卡尔曼增益
  float X; // 状态估计值
} KalmanFilter;

static KalmanFilter distance_filter;
static char oled_buffer[32];

// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter *filter) {
  filter->Q = 0.1f;
  filter->R = 0.1f;
  filter->P = 1.0f;
  filter->K = 0.0f;
  filter->X = 0.0f;
}

// 卡尔曼滤波计算
float Kalman_Update(KalmanFilter *filter, float measurement) {
  // 预测
  filter->P = filter->P + filter->Q;

  // 更新
  filter->K = filter->P / (filter->P + filter->R);
  filter->X = filter->X + filter->K * (measurement - filter->X);
  filter->P = (1 - filter->K) * filter->P;

  return filter->X;
}

void BSP_Init(void) {
  Delay_Init();
  Bsp_UART1_Init();
  Bsp_TIM7_Init();
  OLED_Init();

  // 初始化卡尔曼滤波器
  Kalman_Init(&distance_filter);

  OLED_Draw_Line("Distance Monitor", 1, true, true);
}

void BSP_Loop(void) {
  // 获取原始距离
  float raw_distance = Get_distance();

  // 应用卡尔曼滤波
  float filtered_distance = Kalman_Update(&distance_filter, raw_distance);

  // 显示到OLED
  snprintf(oled_buffer, sizeof(oled_buffer), "Raw: %.1f cm", raw_distance);
  OLED_Draw_Line(oled_buffer, 2, false, false);

  snprintf(oled_buffer, sizeof(oled_buffer), "Filt: %.1f cm",
           filtered_distance);
  OLED_Draw_Line(oled_buffer, 3, false, true);

  // 打印到串口
  printf("Distance - Raw: %.2f cm, Filtered: %.2f cm\r\n", raw_distance,
         filtered_distance);

  Delay_MS(200);
}