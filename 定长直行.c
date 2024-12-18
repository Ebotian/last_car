#include "bsp.h"
#include <stdlib.h>

// 距离配置（单位：厘米）
#define TARGET_DISTANCE_CM 50.0f
#define ENCODER_PER_CM 50.0f
#define TARGET_ENCODER_COUNT ((int)(TARGET_DISTANCE_CM * ENCODER_PER_CM))
//这里的常数也是要依赖场地的,缺乏终点验证机制,即距离测量方式

// 打印间隔时间（毫秒）
#define PRINT_INTERVAL_MS 100

// 全局变量声明
// M1==左前轮
// M2==左后轮
// M3==右前轮
// M4==右后轮
int16_t speed;
int Encoder_M3 = 0;
int Encoder_M2 = 0;
int Encoder_M1 = 0;
int Encoder_M4 = 0;

// K2按键记录的编码器值
int Encoder_M3_K2 = 0;
int Encoder_M2_K2 = 0;
int Encoder_M1_K2 = 0;
int Encoder_M4_K2 = 0;

// K3按键记录的编码器值
int Encoder_M3_K3 = 0;
int Encoder_M2_K3 = 0;
int Encoder_M1_K3 = 0;
int Encoder_M4_K3 = 0;

// 电机控制函数
void Set_All_Motors_Speed(int16_t speed) {
  Motor_Set_Pwm(MOTOR_ID_M1, speed);
  Motor_Set_Pwm(MOTOR_ID_M2, speed);
  Motor_Set_Pwm(MOTOR_ID_M3, speed);
  Motor_Set_Pwm(MOTOR_ID_M4, speed);
}

// 更新编码器值
void Update_All_Encoders(void) {
  Encoder_Update_Count();
  Encoder_M1 = Encoder_Get_Count_Now(MOTOR_ID_M1);
  Encoder_M2 = Encoder_Get_Count_Now(MOTOR_ID_M2);
  Encoder_M3 = Encoder_Get_Count_Now(MOTOR_ID_M3);
  Encoder_M4 = Encoder_Get_Count_Now(MOTOR_ID_M4);
}

// 打印编码器值
void Print_Encoders(const char *prefix) {
  printf("%s Encoder:%d,%d,%d,%d\r\n", prefix, Encoder_M1, Encoder_M2,
         Encoder_M3, Encoder_M4);
}

// 计算实际移动距离（厘米）
float Calculate_Distance(int encoder_count) {
  return (float)encoder_count / ENCODER_PER_CM;
}

// 计算误差百分比
float Calculate_Error_Percentage(float actual_distance) {
  return ((actual_distance - TARGET_DISTANCE_CM) / TARGET_DISTANCE_CM) * 100.0f;
}

// 打印调试信息
void Print_Debug_Info(int encoder_diff, float actual_distance,
                      float error_percentage) {
  printf("\r\n=== Debug Report ===\r\n");
  printf("Target Distance: %.1f cm\r\n", TARGET_DISTANCE_CM);
  printf("Actual Distance: %.1f cm\r\n", actual_distance);
  printf("Target Encoder: %d\r\n", TARGET_ENCODER_COUNT);
  printf("Actual Encoder: %d\r\n", encoder_diff);
  printf("Error: %.1f%%\r\n", error_percentage);
  printf("==================\r\n\r\n");
}

// 检查编码器差值
int Check_Encoder_Difference(int current, int recorded, int16_t motor_speed) {
  int target_diff =
      (motor_speed > 0) ? TARGET_ENCODER_COUNT : -TARGET_ENCODER_COUNT;
  int actual_diff = current - recorded;
  return ((motor_speed > 0 && actual_diff >= target_diff) ||
          (motor_speed < 0 && actual_diff <= target_diff))
             ? 1
             : 0;
}

// 检查所有电机是否达到目标
int Check_All_Motors_Target(int16_t motor_speed) {
  if (motor_speed > 0) {
    return Check_Encoder_Difference(Encoder_M1, Encoder_M1_K2, motor_speed) &&
           Check_Encoder_Difference(Encoder_M2, Encoder_M2_K2, motor_speed) &&
           Check_Encoder_Difference(Encoder_M3, Encoder_M3_K2, motor_speed) &&
           Check_Encoder_Difference(Encoder_M4, Encoder_M4_K2, motor_speed);
  } else if (motor_speed < 0) {
    return Check_Encoder_Difference(Encoder_M1, Encoder_M1_K3, motor_speed) &&
           Check_Encoder_Difference(Encoder_M2, Encoder_M2_K3, motor_speed) &&
           Check_Encoder_Difference(Encoder_M3, Encoder_M3_K3, motor_speed) &&
           Check_Encoder_Difference(Encoder_M4, Encoder_M4_K3, motor_speed);
  }
  return 0;
}

void BSP_Init(void) {
  Bsp_UART1_Init();
  Bsp_Tim_Init();
  printf("\r\n=== System Configuration ===\r\n");
  printf("Target Distance: %.1f cm\r\n", TARGET_DISTANCE_CM);
  printf("Target Encoder Count: %d\r\n", TARGET_ENCODER_COUNT);
  printf("Encoder per CM: %.1f\r\n", ENCODER_PER_CM);
  printf("========================\r\n\r\n");
}

void BSP_Loop(void) {
  static int Flag_K2 = 0;
  static int Flag_K3 = 0;
  static int Last_K2_State = 0;
  static int Last_K3_State = 0;
  static uint32_t last_print_time = 0;
  static int movement_complete = 0;

  Update_All_Encoders();

  int Current_K2_State = Key2_State(0);
  int Current_K3_State = Key3_State(0);

  // Key1处理
  if (Key1_State(0)) {
    Flag_K2 = 0;
    Flag_K3 = 0;
    movement_complete = 0;
    speed = 0;
    Set_All_Motors_Speed(speed);
    Print_Encoders("Manual mode active.");
  }

  // Key2按下瞬间检测
  if (Current_K2_State && !Last_K2_State) {
    Flag_K2 = 1;
    Flag_K3 = 0;
    movement_complete = 0;

    Encoder_M1_K2 = Encoder_M1;
    Encoder_M2_K2 = Encoder_M2;
    Encoder_M3_K2 = Encoder_M3;
    Encoder_M4_K2 = Encoder_M4;

    speed = 1000;
    Set_All_Motors_Speed(speed);
    printf("\r\nStarting forward movement...\r\n");
    printf("Target: %.1f cm (%d encoder counts)\r\n", TARGET_DISTANCE_CM,
           TARGET_ENCODER_COUNT);
  }

  // Key3按下瞬间检测
  if (Current_K3_State && !Last_K3_State) {
    Flag_K3 = 1;
    Flag_K2 = 0;
    movement_complete = 0;

    Encoder_M1_K3 = Encoder_M1;
    Encoder_M2_K3 = Encoder_M2;
    Encoder_M3_K3 = Encoder_M3;
    Encoder_M4_K3 = Encoder_M4;

    speed = -1000;
    Set_All_Motors_Speed(speed);
    printf("\r\nStarting backward movement...\r\n");
    printf("Target: %.1f cm (%d encoder counts)\r\n", TARGET_DISTANCE_CM,
           TARGET_ENCODER_COUNT);
  }

  Last_K2_State = Current_K2_State;
  Last_K3_State = Current_K3_State;

  // 检查是否需要停止电机并输出调试信息
  if ((Flag_K2 || Flag_K3) && speed != 0) {
    if (Check_All_Motors_Target(speed) && !movement_complete) {
      movement_complete = 1;
      speed = 0;
      Set_All_Motors_Speed(speed);

      // 计算实际移动的距离和误差
      int encoder_diff;
      if (Flag_K2) {
        encoder_diff = Encoder_M1 - Encoder_M1_K2;
      } else {
        encoder_diff = Encoder_M1 - Encoder_M1_K3;
      }
      encoder_diff = abs(encoder_diff);

      float actual_distance = Calculate_Distance(encoder_diff);
      float error_percentage = Calculate_Error_Percentage(actual_distance);

      Print_Debug_Info(encoder_diff, actual_distance, error_percentage);
    }
  }

  // 限制打印频率的实时状态更新
  uint32_t current_time = HAL_GetTick();
  if (current_time - last_print_time >= PRINT_INTERVAL_MS) {
    if ((Flag_K2 || Flag_K3) && !movement_complete) {
      int current_diff;
      if (Flag_K2) {
        current_diff = Encoder_M1 - Encoder_M1_K2;
        printf("Current progress: %d/%d counts (%.1f/%.1f cm)\r\n",
               abs(current_diff), TARGET_ENCODER_COUNT,
               Calculate_Distance(abs(current_diff)), TARGET_DISTANCE_CM);
      }
      if (Flag_K3) {
        current_diff = Encoder_M1 - Encoder_M1_K3;
        printf("Current progress: %d/%d counts (%.1f/%.1f cm)\r\n",
               abs(current_diff), TARGET_ENCODER_COUNT,
               Calculate_Distance(abs(current_diff)), TARGET_DISTANCE_CM);
      }
    }
    last_print_time = current_time;
  }
}
