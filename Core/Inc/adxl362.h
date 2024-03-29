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

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

#define _STEP_LOOPNUM         12  // 20min per step,have 12 steps,equal 4 hours
#define _FIFO_LEN             1024
#define _FIFO_SAMPLES_LEN     900
#define _AXIS_LEN             170
#define _FILTER_CNT			      3
#define _DIFF_CNT			        2
#define _MEM_ROWS			        18
#define _MEM_COLS			        4

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

#define XL362_DEVID_AD			  0x00
#define XL362_DEVID_MST			  0x01
#define XL362_PARTID			    0x02
#define XL362_REVID			  	  0x03
#define XL362_XDATA				    0x08
#define XL362_YDATA				    0x09
#define XL362_ZDATA				    0x0A
#define XL362_STATUS			    0x0B
#define XL362_FIFO_ENTRIES_L	0x0C
#define XL362_FIFO_ENTRIES_H	0x0D
#define XL362_XDATA_L			    0x0E
#define XL362_XDATA_H			    0x0F
#define XL362_YDATA_L			    0x10
#define XL362_YDATA_H			    0x11
#define XL362_ZDATA_L			    0x12
#define XL362_ZDATA_H			    0x13
#define XL362_TEMP_L			    0x14
#define XL362_TEMP_H			    0x15
#define XL362_SOFT_RESET		  0x1F
#define XL362_THRESH_ACT_L		0x20
#define XL362_THRESH_ACT_H		0x21
#define XL362_TIME_ACT			  0x22
#define XL362_THRESH_INACT_L	0x23
#define XL362_THRESH_INACT_H	0x24
#define XL362_TIME_INACT_L		0x25
#define XL362_TIME_INACT_H		0x26
#define XL362_ACT_INACT_CTL		0x27
#define XL362_FIFO_CONTROL		0x28
#define XL362_FIFO_SAMPLES		0x29
#define XL362_INTMAP1			    0x2A
#define XL362_INTMAP2			    0x2B
#define XL362_FILTER_CTL		  0x2C
#define XL362_POWER_CTL			  0x2D
#define XL362_SELF_TEST			  0x2E

unsigned char ADXL362RegisterRead(unsigned char Address);
void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue);
void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData);
void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData);
void ADXL362FifoRead(unsigned int NumberofRegisters, unsigned char *RegisterData);
uint16_t ADXL362FifoEntries(void);
void ADXL362FifoProcess(void);
void ADXL362_Init(void);

#endif
