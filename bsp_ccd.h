#ifndef __BSP_CCD_H_
#define __BSP_CCD_H_

#include "bsp.h"

// IO口操作宏定义
#define BITBAND(addr, bitnum)                                                  \
  ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOF_ODR_Addr (GPIOF_BASE + 12)
#define GPIOF_IDR_Addr (GPIOF_BASE + 8)

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)

#define TSL_SI PFout(4)
#define TSL_CLK PFout(5)
#define CCD_ADC_CH ADC_CHANNEL_4

// 参数定义
#define MIN_LINE_WIDTH 4
#define MAX_LINE_WIDTH 30
#define NOISE_THRESHOLD 5
#define SIGNAL_SMOOTH_SIZE 3
#define MAX_EDGE_PAIRS 5

// 曝光控制参数
#define MIN_EXPOSURE_TIME 1
#define MAX_EXPOSURE_TIME 50
#define TARGET_MAX_VALUE 140
#define TARGET_MIN_VALUE 40

// 数据处理结构体
typedef struct {
  uint16_t max_value;
  uint16_t min_value;
  uint16_t last_threshold;
  uint8_t left_edge;
  uint8_t right_edge;
  uint8_t line_width;
  uint8_t exposure_time;
  uint8_t stable_count;
  uint16_t raw_data[128];
  uint16_t smooth_data[128];
  bool is_black[128];
  struct {
    uint8_t left;
    uint8_t right;
    uint8_t width;
    int16_t quality;
  } edges[MAX_EDGE_PAIRS];
  uint8_t edge_count;
} CCD_Process;

// 全局变量声明
extern uint16_t ADV[128];
extern uint8_t CCD_median;
extern uint8_t CCD_threshold;
extern uint8_t ADC_128X32[128];
extern CCD_Process ccd;

// 函数声明
void Deal_Data_CCD(void);
void Find_CCD_Median(void);
void Print_CCD_data(void);
uint8_t *CCD_Get_ADC_128X32(void);
void OLED_Show_CCD_Image(uint8_t *p_img);

#endif