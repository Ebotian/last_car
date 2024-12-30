#include "line_tracking.h"

// 运行状态结构体
typedef struct {
  int16_t base_speed;
  int16_t current_speed;
  uint8_t lost_line_count;
  int8_t last_direction;
  bool is_running;
} Track_State;

static Track_State track = {0};

// 限制数值范围
static int16_t limit_value(int16_t value, int16_t min, int16_t max) {
  if (value > max)
    return max;
  if (value < min)
    return min;
  return value;
}

// 初始化巡线控制
void Track_Init(void) {
  track.base_speed = BASE_SPEED;
  track.current_speed = 0;
  track.lost_line_count = 0;
  track.last_direction = 0;
  track.is_running = true;
}

// 动态速度调整
static int16_t Adjust_Speed(uint8_t line_width, float error) {
  float speed_ratio = 1.0f;

  // 根据偏差大小调整速度
  float error_ratio = ABS(error) / 64.0f; // 归一化偏差

  // 转弯时大幅降低速度
  if (error_ratio > 0.1f) {                    // 偏差超过10%就开始降速
    speed_ratio = 1.0f - (error_ratio * 0.8f); // 最多降至20%速度
    speed_ratio = limit_value(speed_ratio, 0.2f, 1.0f);
  }

  // 计算调整后的速度
  int16_t adjusted_speed = (int16_t)(BASE_SPEED * speed_ratio);
  return limit_value(adjusted_speed, MIN_SPEED, BASE_SPEED);
}

// 更新巡线控制
void Track_Update(void) {
  if (!track.is_running)
    return;

  // 获取最新CCD数据
  Deal_Data_CCD();

  // 丢线检测
  if (ccd.line_width == 0) {
    track.lost_line_count++;
    if (track.lost_line_count > LINE_LOST_THRESHOLD) {
      // 丢线处理：使用最小速度进行寻线
      int16_t search_speed = track.last_direction * MIN_SPEED;

      // 设置电机速度 (M1M2为左轮)
      Motor_Set_Pwm(MOTOR_ID_M1, search_speed);  // 左前轮
      Motor_Set_Pwm(MOTOR_ID_M2, search_speed);  // 左后轮
      Motor_Set_Pwm(MOTOR_ID_M3, -search_speed); // 右前轮
      Motor_Set_Pwm(MOTOR_ID_M4, -search_speed); // 右后轮
      return;
    }
  } else {
    track.lost_line_count = 0;
  }

  // 计算偏差 (线在左边时CCD_median > 64，需要向左转)
  float error = CCD_median - 64;

  // 根据偏差调整基础速度
  int16_t adjusted_speed = Adjust_Speed(ccd.line_width, error);

  // 计算转向量（简单的比例控制）
  float turn_ratio = error / 64.0f; // 归一化转向比例
  int16_t turn = (int16_t)(adjusted_speed * turn_ratio);

  // 限制转向输出
  turn = limit_value(turn, -MAX_SPEED_DIFF, MAX_SPEED_DIFF);

  // 计算左右电机速度
  int16_t left_speed = adjusted_speed - turn;
  int16_t right_speed = adjusted_speed + turn;

  // 确保最小速度
  left_speed = limit_value(left_speed, MIN_SPEED, MAX_SPEED);
  right_speed = limit_value(right_speed, MIN_SPEED, MAX_SPEED);

  // 记录转向方向
  track.last_direction = (turn > 0) ? 1 : -1;

  // 设置电机速度
  Motor_Set_Pwm(MOTOR_ID_M1, left_speed);
  Motor_Set_Pwm(MOTOR_ID_M2, left_speed);
  Motor_Set_Pwm(MOTOR_ID_M3, right_speed);
  Motor_Set_Pwm(MOTOR_ID_M4, right_speed);
}

// 停止巡线
void Track_Stop(void) {
  track.is_running = false;
  Motor_Stop(1); // 1表示刹车
}

// 重置巡线状态
void Track_Reset(void) { Track_Init(); }