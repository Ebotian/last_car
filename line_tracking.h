#ifndef __LINE_TRACKING_H
#define __LINE_TRACKING_H

#include "bsp_ccd.h"
#include "bsp_motor.h"

// 基础速度参数
#define BASE_SPEED 800
#define MAX_SPEED 1000
#define MIN_SPEED 300
#define MAX_SPEED_DIFF 200
#define LINE_LOST_THRESHOLD 50 // 丢线计数阈值

// PID参数
#define PID_KP 12.0f
#define PID_KI 0.15f
#define PID_KD 3.0f

// 动态速度控制参数
#define NORMAL_LINE_WIDTH 12
#define SPEED_REDUCE_RATIO 0.7f

// 丢线检测参数
#define LINE_LOST_THRESHOLD 20
#define SEARCH_SPEED 800

// 函数声明
void Track_Init(void);
void Track_Update(void);
void Track_Stop(void);
void Track_Reset(void);

#endif
