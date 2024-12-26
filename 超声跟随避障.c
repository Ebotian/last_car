#include "bsp.h"
#include <math.h>
#include <stdlib.h>

// 配置参数
#define FOLLOW_STOP_DISTANCE 2.0f // 停止距离（厘米）
#define FOLLOW_MAX_DISTANCE 10.0f // 最大跟随距离（厘米）
#define FOLLOW_BASE_DISTANCE 5.5f // 基准距离（5-6cm中间值）
#define AVOID_DISTANCE 5.0f       // 避障距离（厘米）

// 速度参数
#define BASE_SPEED 800 // 基础速度
#define MAX_SPEED 2000 // 最大速度
#define SPEED_STEP 100 // 每厘米速度变化步长
#define TURN_SPEED 800 // 转弯速度
#define MIN_SPEED 300  // 最小速度，防止卡死

// IR传感器阈值
#define IR_TURN_THRESHOLD 200 // IR传感器转向阈值
#define IR_BACK_THRESHOLD 100 // IR传感器后退阈值
#define IR_MAX_VALUE 4095     // 12位ADC最大值

// 其他参数
#define PRINT_INTERVAL_MS 100 // 打印间隔

// 运行模式定义
#define MODE_STOP 0
#define MODE_FOLLOW 1
#define MODE_AVOID 2

// 卡尔曼滤波器结构体
typedef struct {
  float Q; // 过程噪声协方差
  float R; // 测量噪声协方差
  float P; // 估计误差协方差
  float K; // 卡尔曼增益
  float X; // 状态估计值
} KalmanFilter;

// 模糊控制结构体
typedef struct {
  float distance_error; // 距离误差
  float ir_left_value;  // 左IR传感器值
  float ir_right_value; // 右IR传感器值
  int16_t left_speed;   // 左侧电机输出
  int16_t right_speed;  // 右侧电机输出
} FuzzyControl;

// 全局变量
static KalmanFilter distance_filter;
static FuzzyControl fuzzy_control;
static char oled_buffer[32];
static int16_t speed_left = 0;
static int16_t speed_right = 0;
static int working_mode = MODE_STOP;

// 卡尔曼滤波器初始化
void Kalman_Init(KalmanFilter *filter) {
  filter->Q = 0.1f;
  filter->R = 0.1f;
  filter->P = 1.0f;
  filter->K = 0.0f;
  filter->X = 0.0f;
}

// 卡尔曼滤波器更新
float Kalman_Update(KalmanFilter *filter, float measurement) {
  filter->P = filter->P + filter->Q;
  filter->K = filter->P / (filter->P + filter->R);
  filter->X = filter->X + filter->K * (measurement - filter->X);
  filter->P = (1 - filter->K) * filter->P;
  return filter->X;
}

// 计算跟随速度的辅助函数（仅处理2-10cm范围）
int16_t Calculate_Follow_Speed(float current_distance) {
  int16_t target_speed;

  if (current_distance > 6.0f) {
    // 6-10cm 加速追赶
    float over_distance = current_distance - 6.0f;
    int16_t extra_speed = (int16_t)(over_distance * SPEED_STEP);
    target_speed = BASE_SPEED + extra_speed;
    // 限制最大速度
    return (target_speed > MAX_SPEED) ? MAX_SPEED : target_speed;
  } else if (current_distance >= 5.0f) {
    // 5-6cm 使用基础速度
    return BASE_SPEED;
  } else {
    // 2-5cm 递减速度
    float slow_distance = 5.0f - current_distance;
    target_speed = BASE_SPEED - (int16_t)(slow_distance * SPEED_STEP);
    // 限制最小速度
    return (target_speed < MIN_SPEED) ? MIN_SPEED : target_speed;
  }
}

void Fuzzy_Control_Update(FuzzyControl *ctrl, float current_distance,
                          bool is_follow_mode) {
  // 获取IR传感器数据
  uint16_t left_ir, right_ir;
  Get_Iravoid_Data(&left_ir, &right_ir);
  ctrl->ir_left_value = left_ir;
  ctrl->ir_right_value = right_ir;

  if (is_follow_mode) {
    // 跟随模式
    if (current_distance <= FOLLOW_STOP_DISTANCE) {
      // 0-2cm，停止
      ctrl->left_speed = 0;
      ctrl->right_speed = 0;
      return;
    }

    if (current_distance > FOLLOW_MAX_DISTANCE) {
      // 距离超过10cm，寻找目标
      bool left_detected = (left_ir < IR_TURN_THRESHOLD);
      bool right_detected = (right_ir < IR_TURN_THRESHOLD);

      if (left_detected && !right_detected) {
        // 仅左侧检测到目标，原地左转
        ctrl->left_speed = -TURN_SPEED;
        ctrl->right_speed = TURN_SPEED;
      } else if (!left_detected && right_detected) {
        // 仅右侧检测到目标，原地右转
        ctrl->left_speed = TURN_SPEED;
        ctrl->right_speed = -TURN_SPEED;
      } else {
        // 两侧都检测到或都未检测到，停止
        ctrl->left_speed = 0;
        ctrl->right_speed = 0;
      }
      return;
    }

    // 2-10cm范围内，计算跟随速度
    int16_t follow_speed = Calculate_Follow_Speed(current_distance);
    ctrl->left_speed = follow_speed;
    ctrl->right_speed = follow_speed;
    return;
  } else {
    // 避障模式
    if (current_distance < AVOID_DISTANCE) {
      // 距离小于5cm，后退
      ctrl->left_speed = -BASE_SPEED;
      ctrl->right_speed = -BASE_SPEED;
      return;
    }

    // IR避障控制
    if (left_ir < IR_BACK_THRESHOLD || right_ir < IR_BACK_THRESHOLD) {
      // 非常接近障碍物，后退
      ctrl->left_speed = -BASE_SPEED;
      ctrl->right_speed = -BASE_SPEED;
    } else if (left_ir < IR_TURN_THRESHOLD && right_ir < IR_TURN_THRESHOLD) {
      // 两侧都较近，后退
      ctrl->left_speed = -BASE_SPEED;
      ctrl->right_speed = -BASE_SPEED;
    } else if (left_ir < IR_TURN_THRESHOLD) {
      // 左侧障碍物，向右转
      ctrl->left_speed = BASE_SPEED;
      ctrl->right_speed = -BASE_SPEED;
    } else if (right_ir < IR_TURN_THRESHOLD) {
      // 右侧障碍物，向左转
      ctrl->left_speed = -BASE_SPEED;
      ctrl->right_speed = BASE_SPEED;
    } else {
      // 无障碍，前进
      ctrl->left_speed = BASE_SPEED;
      ctrl->right_speed = BASE_SPEED;
    }
  }
}

// 设置电机速度
void Set_Motors_Speed(int16_t left_speed, int16_t right_speed) {
  Motor_Set_Pwm(MOTOR_ID_M1, left_speed);  // 左前
  Motor_Set_Pwm(MOTOR_ID_M2, left_speed);  // 左后
  Motor_Set_Pwm(MOTOR_ID_M3, right_speed); // 右前
  Motor_Set_Pwm(MOTOR_ID_M4, right_speed); // 右后
}

// 更新OLED显示
void Update_Display(float raw_distance, float filtered_distance,
                    uint16_t left_ir, uint16_t right_ir) {
  const char *mode_str = "Stop";
  if (working_mode == MODE_FOLLOW)
    mode_str = "Follow";
  if (working_mode == MODE_AVOID)
    mode_str = "Avoid";

  OLED_Draw_Line("Distance Monitor", 1, true, false);
  snprintf(oled_buffer, sizeof(oled_buffer), "Mode:%s D:%.1f", mode_str,
           filtered_distance);
  OLED_Draw_Line(oled_buffer, 2, false, false);
  snprintf(oled_buffer, sizeof(oled_buffer), "IR L:%d R:%d", left_ir, right_ir);
  OLED_Draw_Line(oled_buffer, 3, false, true);
}

// 系统初始化
void BSP_Init(void) {
  Delay_Init();
  Bsp_Tim_Init();
  Bsp_UART1_Init();
  Bsp_TIM7_Init();
  OLED_Init();

  Kalman_Init(&distance_filter);

  printf("\r\nEnhanced Control System Ready\r\n");
  printf("Key2: Follow Mode (%.1f-%.1f cm)\r\n", FOLLOW_STOP_DISTANCE,
         FOLLOW_MAX_DISTANCE);
  printf("Key3: Avoid Mode (%.1f cm)\r\n", AVOID_DISTANCE);
  printf("Key1: Stop\r\n");

  OLED_Draw_Line("System Ready", 1, true, true);
}

// 主循环
void BSP_Loop(void) {
  static uint32_t last_print_time = 0;

  // 获取距离并应用卡尔曼滤波
  float raw_distance = Get_distance();
  float filtered_distance = Kalman_Update(&distance_filter, raw_distance);

  // 按键处理
  if (Key1_State(0)) {
    working_mode = MODE_STOP;
    speed_left = speed_right = 0;
  }
  if (Key2_State(0))
    working_mode = MODE_FOLLOW;
  if (Key3_State(0))
    working_mode = MODE_AVOID;

  // 根据模式执行控制
  switch (working_mode) {
  case MODE_FOLLOW:
    Fuzzy_Control_Update(&fuzzy_control, filtered_distance, true);
    break;

  case MODE_AVOID:
    Fuzzy_Control_Update(&fuzzy_control, filtered_distance, false);
    break;

  default: // MODE_STOP
    fuzzy_control.left_speed = 0;
    fuzzy_control.right_speed = 0;
    break;
  }

  // 更新电机速度
  speed_left = fuzzy_control.left_speed;
  speed_right = fuzzy_control.right_speed;
  Set_Motors_Speed(speed_left, speed_right);

  // 更新显示
  Update_Display(raw_distance, filtered_distance, fuzzy_control.ir_left_value,
                 fuzzy_control.ir_right_value);

  // 定期打印状态
  uint32_t current_time = HAL_GetTick();
  if (current_time - last_print_time >= PRINT_INTERVAL_MS) {
    printf("Mode:%d Dist:%.1f IR(L/R):%.0f/%.0f Speed(L/R):%d/%d\r\n",
           working_mode, filtered_distance, fuzzy_control.ir_left_value,
           fuzzy_control.ir_right_value, speed_left, speed_right);
    last_print_time = current_time;
  }
}