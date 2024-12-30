#include "bsp.h"

static RunMode current_mode = MODE_STOP;

// Hardware Initialization
void BSP_Init(void) {
  USART1_UART_Init();
  OLED_Init();
  OLED_Clear();
  OLED_Draw_Line("System Ready!", 0, true, true);
  OLED_Draw_Line("Key1:Track Key2:Display", 2, true, true);
  Bsp_Tim_Init();
  Track_Init(); // 初始化巡线控制
}

// 更新运行模式
static void Update_Mode(void) {
  if (Key1_State(1)) { // Key1按下切换巡线/停止
    if (current_mode != MODE_TRACKING) {
      current_mode = MODE_TRACKING;
      Track_Reset(); // 重置巡线状态
      OLED_Clear();
      OLED_Draw_Line("Tracking Mode", 0, true, true);
      printf("\r\nEntering Tracking Mode\r\n");
    } else {
      current_mode = MODE_STOP;
      Track_Stop();
      OLED_Clear();
      OLED_Draw_Line("Stop Mode", 0, true, true);
      printf("\r\nStopped\r\n");
    }
  }

  if (Key2_State(1)) { // Key2按下切换显示/停止
    if (current_mode != MODE_DISPLAY) {
      current_mode = MODE_DISPLAY;
      Track_Stop();
      OLED_Clear();
      OLED_Draw_Line("Display Mode", 0, true, true);
      printf("\r\nEntering Display Mode\r\n");
    } else {
      current_mode = MODE_STOP;
      OLED_Clear();
      OLED_Draw_Line("Stop Mode", 0, true, true);
      printf("\r\nStopped\r\n");
    }
  }
}

// Loop Run Function
void BSP_Loop(void) {
  Update_Mode(); // 更新运行模式

  switch (current_mode) {
  case MODE_TRACKING:
    Track_Update();                            // 更新巡线控制
    OLED_Show_CCD_Image(CCD_Get_ADC_128X32()); // 实时更新显示
    break;

  case MODE_DISPLAY:
    Print_CCD_data(); // 显示详细数据并更新OLED
    break;

  case MODE_STOP:
  default:
    Motor_Stop(1);                             // 确保电机停止
    OLED_Show_CCD_Image(CCD_Get_ADC_128X32()); // 保持显示更新
    break;
  }

  HAL_Delay(1); // 最小延时，保证系统稳定
}
