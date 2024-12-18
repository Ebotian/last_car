#include "bsp.h"
#include <stdlib.h>
// 配置参数
#define TURN_ANGLE_DEG 90.0f
#define ENCODER_PER_DEG 25.2f // 原来的4.2 * 6
#define TARGET_ENCODER_COUNT ((int)(TURN_ANGLE_DEG * ENCODER_PER_DEG))
//  这里的常数严重依赖摩擦力,而且各个电机并不全等,
//  必须有终点验证机制比如陀螺仪才可以

// 电机速度配置
#define TURN_SPEED 800
#define PRINT_INTERVAL_MS 200 // 增加打印间隔，避免打印太频繁

// 全局变量
int16_t speed_left = 0;
int16_t speed_right = 0;
int Encoder_M1 = 0; // 左前轮
int Encoder_M2 = 0; // 左后轮
int Encoder_M3 = 0; // 右前轮
int Encoder_M4 = 0; // 右后轮

// 记录转弯开始时的编码器值
int Encoder_M1_Start = 0;
int Encoder_M2_Start = 0;
int Encoder_M3_Start = 0;
int Encoder_M4_Start = 0;

void Update_All_Encoders(void) {
  Encoder_Update_Count();
  Encoder_M1 = Encoder_Get_Count_Now(MOTOR_ID_M1);
  Encoder_M2 = Encoder_Get_Count_Now(MOTOR_ID_M2);
  Encoder_M3 = Encoder_Get_Count_Now(MOTOR_ID_M3);
  Encoder_M4 = Encoder_Get_Count_Now(MOTOR_ID_M4);
}

void Set_Motors_Speed(int16_t left_speed, int16_t right_speed) {
  Motor_Set_Pwm(MOTOR_ID_M1, left_speed);  // 左前
  Motor_Set_Pwm(MOTOR_ID_M2, left_speed);  // 左后
  Motor_Set_Pwm(MOTOR_ID_M3, right_speed); // 右前
  Motor_Set_Pwm(MOTOR_ID_M4, right_speed); // 右后
}

int Check_Turn_Complete(void) {
  int left_diff = abs(Encoder_M1 - Encoder_M1_Start);
  int right_diff = abs(Encoder_M3 - Encoder_M3_Start);
  int avg_diff = (left_diff + right_diff) / 2;

  return (avg_diff >= TARGET_ENCODER_COUNT);
}

void BSP_Init(void) {
  Bsp_UART1_Init();
  Bsp_Tim_Init();
  printf("\r\nTurn Control System Ready.\r\n");
  printf("Target Angle: %.1f degrees\r\n", TURN_ANGLE_DEG);
  printf("Target Encoder Count: %d\r\n", TARGET_ENCODER_COUNT);
}

void BSP_Loop(void) {
  static int turning_state = 0;
  static uint32_t last_print_time = 0;

  Update_All_Encoders();

  // 按键2：向右转90度
  if (Key2_State(0) && !turning_state) {
    turning_state = 1;
    Encoder_M1_Start = Encoder_M1;
    Encoder_M2_Start = Encoder_M2;
    Encoder_M3_Start = Encoder_M3;
    Encoder_M4_Start = Encoder_M4;

    speed_left = TURN_SPEED;
    speed_right = -TURN_SPEED;
    Set_Motors_Speed(speed_left, speed_right);
    printf("\r\nStarting right turn (Target: %d counts)...\r\n",
           TARGET_ENCODER_COUNT);
  }

  // 按键3：向左转90度
  if (Key3_State(0) && !turning_state) {
    turning_state = 1;
    Encoder_M1_Start = Encoder_M1;
    Encoder_M2_Start = Encoder_M2;
    Encoder_M3_Start = Encoder_M3;
    Encoder_M4_Start = Encoder_M4;

    speed_left = -TURN_SPEED;
    speed_right = TURN_SPEED;
    Set_Motors_Speed(speed_left, speed_right);
    printf("\r\nStarting left turn (Target: %d counts)...\r\n",
           TARGET_ENCODER_COUNT);
  }

  // 检查是否完成转弯
  if (turning_state && Check_Turn_Complete()) {
    turning_state = 0;
    speed_left = 0;
    speed_right = 0;
    Set_Motors_Speed(speed_left, speed_right);

    // 计算最终的编码器变化值
    int left_diff = abs(Encoder_M1 - Encoder_M1_Start);
    int right_diff = abs(Encoder_M3 - Encoder_M3_Start);

    printf("\r\n=== Turn Complete ===\r\n");
    printf("Left encoder diff: %d\r\n", left_diff);
    printf("Right encoder diff: %d\r\n", right_diff);
    printf("Target was: %d\r\n", TARGET_ENCODER_COUNT);
    printf("==================\r\n");
  }

  // 按键1：紧急停止
  if (Key1_State(0)) {
    turning_state = 0;
    speed_left = 0;
    speed_right = 0;
    Set_Motors_Speed(speed_left, speed_right);
    printf("Emergency stop!\r\n");
  }

  // 定期打印状态（增加了完成百分比显示）
  uint32_t current_time = HAL_GetTick();
  if (turning_state && current_time - last_print_time >= PRINT_INTERVAL_MS) {
    int left_diff = abs(Encoder_M1 - Encoder_M1_Start);
    int right_diff = abs(Encoder_M3 - Encoder_M3_Start);
    float progress =
        ((float)(left_diff + right_diff) / 2 / TARGET_ENCODER_COUNT) * 100;
    printf("Progress: %.1f%% (L:%d R:%d Target:%d)\r\n", progress, left_diff,
           right_diff, TARGET_ENCODER_COUNT);
    last_print_time = current_time;
  }
}