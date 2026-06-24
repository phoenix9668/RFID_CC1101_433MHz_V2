/********************************************************************************
 Author : CAC (China Applications Support Team)

 Date :   May, 2014

 File name :  ADXL362.h

 Description :	 ADXL362 Registers

 Hardware plateform : 	EVAL-ADuCM360MKZ and EVAL-ADXL362Z
********************************************************************************/

#include "stm32l0xx.h"
#include <string.h>
#include "math.h"

#ifndef _ADXL362DRIVER_H_
#define _ADXL362DRIVER_H_

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define _STEP_LOOPNUM 12 // 20min per step,have 12 steps,equal 4 hours
#define _FIFO_LEN 1024
#define _FIFO_SAMPLES_LEN 900
#define _AXIS_LEN 170
#define _FILTER_CNT 3
#define _DIFF_CNT 2
#define _MEM_ROWS 18
#define _MEM_COLS 4

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} axis_info_int16_t;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} axis_info_int32_t;

typedef struct
{
    uint16_t low;
    uint16_t normal;
    uint16_t abovenormal;
    uint16_t high;
} threshold_judge_t;

typedef struct
{
    uint16_t rest;
    uint16_t ingestion;
    uint16_t movement;
    uint16_t climb;
    uint16_t ruminate;
    uint16_t other;
} action_classify_t;

extern action_classify_t action_classify;

/* 正弦波检测相关宏定义 */
#define SINE_WAVE_MIN_FREQ 1.0f    // 最小频率 (Hz)
#define SINE_WAVE_MAX_FREQ 1.7f    // 最大频率 (Hz)
#define SINE_WAVE_MIN_AMPLITUDE 30 // 最小幅度
#define PLATEAU_THRESHOLD 5        // 平台期阈值
#define MIN_PLATEAU_COUNT 3        // 最小平台期计数
#define MAX_PLATEAU_WINDOW 3       // 平台期检测窗口大小
#define MAX_PEAK_COUNT 20          // 最大峰值数量
#define MAX_VALLEY_COUNT 20        // 最大谷值数量
#define MAX_PERIOD_COUNT 11        // 最大周期数量
#define STD_DEV_THRESHOLD 0.3f     // 标准差阈值（幅度的百分比）
#define SMOOTH_WINDOW_SIZE 5       // 平滑窗口大小
#define EXTREMA_WINDOW_SIZE 3      // 极值检测窗口大小（单侧）

/* 正弦波检测相关结构体 */
typedef struct
{
    int16_t value;  // 峰值/谷值
    uint16_t index; // 峰值/谷值索引
} extreme_point_t;

typedef struct
{
    extreme_point_t peaks[MAX_PEAK_COUNT];     // 峰值数组
    extreme_point_t valleys[MAX_VALLEY_COUNT]; // 谷值数组
    uint16_t peak_count;                       // 峰值计数
    uint16_t valley_count;                     // 谷值计数
    uint16_t periods[MAX_PERIOD_COUNT];        // 周期数组
    uint8_t period_count;                      // 周期计数
    uint8_t is_rising;                         // 是否处于上升阶段
    uint8_t is_sine_wave;                      // 是否为正弦波
    float frequency;                           // 频率
    int16_t amplitude;                         // 幅度
} sine_wave_detection_t;

/**
 * \brief           Buffer for FIFO
 * \note            SPI
 */
extern uint8_t fifo[_FIFO_LEN];
extern axis_info_int16_t three_axis_info[_AXIS_LEN];
extern axis_info_int32_t three_axis_average_info;
extern int32_t memory_array[_MEM_ROWS][_MEM_COLS];
extern uint8_t action_classify_array[6];

typedef struct
{
    __IO uint8_t stepStage;
    __IO FlagStatus fifoOverrun;
    __IO uint16_t restArray[_STEP_LOOPNUM];
    __IO uint16_t ingestionArray[_STEP_LOOPNUM];
    __IO uint16_t movementArray[_STEP_LOOPNUM];
    __IO uint16_t climbArray[_STEP_LOOPNUM];
    __IO uint16_t ruminateArray[_STEP_LOOPNUM];
    __IO uint16_t otherArray[_STEP_LOOPNUM];
} step_t;

extern step_t step;

/* ------- Register names ------- */

#define XL362_DEVID_AD 0x00
#define XL362_DEVID_MST 0x01
#define XL362_PARTID 0x02
#define XL362_REVID 0x03
#define XL362_XDATA 0x08
#define XL362_YDATA 0x09
#define XL362_ZDATA 0x0A
#define XL362_STATUS 0x0B
#define XL362_FIFO_ENTRIES_L 0x0C
#define XL362_FIFO_ENTRIES_H 0x0D
#define XL362_XDATA_L 0x0E
#define XL362_XDATA_H 0x0F
#define XL362_YDATA_L 0x10
#define XL362_YDATA_H 0x11
#define XL362_ZDATA_L 0x12
#define XL362_ZDATA_H 0x13
#define XL362_TEMP_L 0x14
#define XL362_TEMP_H 0x15
#define XL362_SOFT_RESET 0x1F
#define XL362_THRESH_ACT_L 0x20
#define XL362_THRESH_ACT_H 0x21
#define XL362_TIME_ACT 0x22
#define XL362_THRESH_INACT_L 0x23
#define XL362_THRESH_INACT_H 0x24
#define XL362_TIME_INACT_L 0x25
#define XL362_TIME_INACT_H 0x26
#define XL362_ACT_INACT_CTL 0x27
#define XL362_FIFO_CONTROL 0x28
#define XL362_FIFO_SAMPLES 0x29
#define XL362_INTMAP1 0x2A
#define XL362_INTMAP2 0x2B
#define XL362_FILTER_CTL 0x2C
#define XL362_POWER_CTL 0x2D
#define XL362_SELF_TEST 0x2E

unsigned char ADXL362RegisterRead(unsigned char Address);
void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue);
void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData);
void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData);
void ADXL362FifoRead(unsigned int NumberofRegisters, unsigned char *RegisterData);
uint16_t ADXL362FifoEntries(void);
void ADXL362FifoProcess(void);
void ADXL362_Init(void);

#endif
