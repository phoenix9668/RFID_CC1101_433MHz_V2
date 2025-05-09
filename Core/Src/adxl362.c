/********************************************************************************
 Author : CAC (China Applications Support Team)

 Date :   May, 2014

 File name :  ADXL362.c

 Description :	 ADXL362 SPI communication driver

 Hardware plateform : 	EVAL-ADuCM360MKZ and EVAL-ADXL362Z

 Connection:
                 EVAL-ADuCM360MKZ       EVAL-ADXL362Z

                 P1.4:MISO,             MISO
                 P1.5:SCLK,             SCLK
                 P1.6:MOSI,             MOSI
                 P1.7:GPIO as CS,       CS
********************************************************************************/

#include "math.h"
#include "adxl362.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "cc1101.h"

axis_info_int16_t three_axis_info[_AXIS_LEN];
axis_info_int16_t diff_three_axis_info[_DIFF_CNT];
axis_info_int32_t three_axis_average_info;
axis_info_int32_t sum_info;
threshold_judge_t threshold_judge;
action_classify_t action_classify;
uint8_t action_classify_array[6];
uint8_t action;

int32_t memory_array[_MEM_ROWS][_MEM_COLS];
uint8_t memory_index = 0;
uint8_t memory_index_o;
uint8_t mid_index = 0;

uint8_t movement_cnt = 0;
uint8_t climb_cnt = 0;
uint8_t rest_cnt1 = 0;
uint8_t rest_cnt2 = 0;
uint8_t ingestion_cnt = 0;
uint8_t rest_cnt = 0;
uint8_t deta_a_cnt = 0;
uint8_t jicha_cnt = 0;
int16_t eighteen_average = 0;
uint16_t sum_eighteen_average = 0;

int16_t x_max_val = -2000;
int16_t x_min_val = 2000;
int16_t y_max_val = -2000;
int16_t y_min_val = 2000;
int16_t z_max_val = -2000;
int16_t z_min_val = 2000;

uint8_t fifo[_FIFO_LEN];
step_t step;

// 初始化正弦波检测结构体
sine_wave_detection_t sine_detection;
// 创建临时缓冲区用于平滑滤波
int16_t smoothed_x[_AXIS_LEN];

// 平台期检测参数
uint8_t plateau_count = 0;
int16_t plateau_value = 0;

extern uint8_t ErrorIndex;

/*******************************************************************
  @brief unsigned char ADXL362RegisterRead(unsigned char Address)
         Read a register value from ADXL362
  @param
         unsigned char Address:       Register address
  @return
         unsigned int  ReceiveValue:  read register value from ADXL362
*******************************************************************/
unsigned char ADXL362RegisterRead(unsigned char Address)
{
    unsigned char SendTemp[2];
    unsigned char ReceiveTemp[2];
    unsigned char ReceiveValue;

    ADXL362_CSN_LOW();     // CS down
    SendTemp[0] = 0x0B;    // 0x0B: read register command
    SendTemp[1] = Address; // address byte
    SpiFunction(SPI2, SendTemp, ReceiveTemp, 2, 1);
    ReceiveValue = ReceiveTemp[0];
    ADXL362_CSN_HIGH();
    return (ReceiveValue); // CS up
}

/*******************************************************************
  @brief void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue)
         send SPI command to ADXL362
  @param
         unsigned char Address:       Register address
         unsigned char SendValue:     Value written to ADXL362 register
  @return
         none
*******************************************************************/
void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue)
{
    unsigned char SendTemp[3];
    unsigned char ReceiveTemp[3];

    ADXL362_CSN_LOW();     // CS down
    SendTemp[0] = 0x0A;    // 0x0A: write register
    SendTemp[1] = Address; // address byte
    SendTemp[2] = SendValue;

    SpiFunction(SPI2, SendTemp, ReceiveTemp, 3, 0);
    ADXL362_CSN_HIGH(); // CS up
}

/*******************************************************************
  @brief void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte read from ADXL362
  @param
         unsigned char Address:           Register address
         unsigned char NumberofRegisters: Register numbers to be read
         unsigned char *RegisterData:     Buffer save the read value
  @return
         none
*******************************************************************/
void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
    unsigned char SendTemp[2];

    ADXL362_CSN_LOW();     // CS down
    SendTemp[0] = 0x0B;    // 0x0B: read register
    SendTemp[1] = Address; // address byte
    SpiFunction(SPI2, SendTemp, RegisterData, 2, NumberofRegisters);
    ADXL362_CSN_HIGH(); // CS up
}

/*******************************************************************
  @brief void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte write to ADXL362
  @param
         unsigned char Address:           Register address
         unsigned char NumberofRegisters: Register numbers to be written
         unsigned char *RegisterData:     Buffer save the written value
  @return
         none
*******************************************************************/
void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
    unsigned char SendTemp[256];
    unsigned char ReceiveTemp[2];
    unsigned char RegisterIndex;

    ADXL362_CSN_LOW();     // CS down
    SendTemp[0] = 0x0A;    // 0x0A: write register
    SendTemp[1] = Address; // address byte

    for (RegisterIndex = 0; RegisterIndex < NumberofRegisters; RegisterIndex++)
    {
        SendTemp[2 + RegisterIndex] = *(RegisterData + RegisterIndex);
    }

    SpiFunction(SPI2, SendTemp, ReceiveTemp, (2 + NumberofRegisters), 0);
    ADXL362_CSN_HIGH(); // CS up
}

/*******************************************************************
  @brief void ADXL362FifoRead(unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte read from ADXL362 FIFO
  @param
         unsigned char NumberofRegisters: Register numbers to be read
         unsigned char *RegisterData:     Buffer save the read value
  @return
         none
*******************************************************************/
void ADXL362FifoRead(unsigned int NumberofRegisters, unsigned char *RegisterData)
{
    unsigned char SendTemp[1];
    ADXL362_CSN_LOW();  // CS down
    SendTemp[0] = 0x0D; // 0x0D: read register
    SpiFunction(SPI2, SendTemp, RegisterData, 1, NumberofRegisters);
    ADXL362_CSN_HIGH(); // CS up
}

/*******************************************************************
  @brief void ADXL362FifoEntries(void)
         read ADXL362 FIFO ENTRIES REGISTERS
  @param
         none
  @return
         none
*******************************************************************/
uint16_t ADXL362FifoEntries(void)
{
    uint16_t ReadValueTemp;
    ReadValueTemp = (0x00ff & ADXL362RegisterRead(XL362_FIFO_ENTRIES_L)) + (0x0300 & ADXL362RegisterRead(XL362_FIFO_ENTRIES_H));
    return (ReadValueTemp);
}

/*******************************************************************
  @brief void ADXL362_Init(void)
         initial and configure ADXL362
  @param
                 none
  @return
         none
*******************************************************************/
void ADXL362_Init(void)
{
    unsigned int ReadValueTemp;
    memset(&step, 0, sizeof(step));
    memset(&three_axis_info, 0, sizeof(three_axis_info));
    memset(memory_array, 0, sizeof(memory_array));

    ADXL362RegisterWrite(XL362_SOFT_RESET, 0x52); // software reset
    HAL_Delay(1000);

#if (_Original_Data_Algorithm == 0)

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, 0x64); // set active threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_L);
    rfid_printf("|*-set THRESH_ACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_ACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_H);
    rfid_printf("|*-set THRESH_ACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06); // set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_L, 0x64); // set inactive threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_L);
    rfid_printf("|*-set THRESH_INACT_L register = 0x%02x-*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_H);
    rfid_printf("|*-set THRESH_INACT_H register = 0x%02x-*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x06); // set inactive time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_H);
    rfid_printf("|*-set TIME_INACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x3F); // configure loop mode,enable active and inactive
    ReadValueTemp = ADXL362RegisterRead(XL362_ACT_INACT_CTL);
    rfid_printf("|*-set ACT_INACT_CTL register = 0x%02x--*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x3F)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP1, 0x10); // configure act map INT1
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP1);
    rfid_printf("|*-set INTMAP1 register = 0x%02x--------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP2, 0x04); // configure fifo_watermark map INT2
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP2);
    rfid_printf("|*-set INTMAP2 register = 0x%02x--------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x04)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x0A); // Above Half, select fifo Steam Mode, not store Temperature Data to FIFO
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_CONTROL);
    rfid_printf("|*-set FIFO_CONTROL register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x0A)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0xC2); // select fifo sample number//0x1C2 = 450
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_SAMPLES);
    rfid_printf("|*-set FIFO_SAMPLES register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0xC2)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x51); // select 4g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x51)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    // any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02); // select measurement mode
    ReadValueTemp = ADXL362RegisterRead(XL362_POWER_CTL);
    rfid_printf("|*-set POWER_CTL register = 0x%02x------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x02)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

#else

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, 0x64); // set active threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_L);
    rfid_printf("|*-set THRESH_ACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_ACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_H);
    rfid_printf("|*-set THRESH_ACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06); // set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_L, 0x64); // set inactive threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_L);
    rfid_printf("|*-set THRESH_INACT_L register = 0x%02x-*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_H);
    rfid_printf("|*-set THRESH_INACT_H register = 0x%02x-*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x06); // set inactive time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_H);
    rfid_printf("|*-set TIME_INACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x3F); // configure loop mode,enable active and inactive
    ReadValueTemp = ADXL362RegisterRead(XL362_ACT_INACT_CTL);
    rfid_printf("|*-set ACT_INACT_CTL register = 0x%02x--*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x3F)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP1, 0x10); // configure act map INT1
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP1);
    rfid_printf("|*-set INTMAP1 register = 0x%02x--------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP2, 0x04); // configure fifo_watermark map INT2
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP2);
    rfid_printf("|*-set INTMAP2 register = 0x%02x--------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x04)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x0A); // Above Half, select fifo Steam Mode, not store Temperature Data to FIFO
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_CONTROL);
    rfid_printf("|*-set FIFO_CONTROL register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x0A)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0xC2); // select fifo sample number//0x1C2 = 450
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_SAMPLES);
    rfid_printf("|*-set FIFO_SAMPLES register = 0x%02x---*|\n", ReadValueTemp);

    if (ReadValueTemp != 0xC2)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x51); // select 4g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x51)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    // any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02); // select measurement mode
    ReadValueTemp = ADXL362RegisterRead(XL362_POWER_CTL);
    rfid_printf("|*-set POWER_CTL register = 0x%02x------*|\n", ReadValueTemp);

    if (ReadValueTemp != 0x02)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

#endif

    rfid_printf("|**************************************|\n");

    HAL_Delay(200);
}

/*******************************************************************
  @brief void ADXL362FifoProcess(void)
         Multibyte read from ADXL362 FIFO
  @param
                 none
  @return
                 none
*******************************************************************/
void ADXL362FifoProcess(void)
{
#if (_Original_Data_Algorithm == 0)
    // 1.init acceler array
    memset(three_axis_info, 0, sizeof(three_axis_info));

    // 2.If the fifo data is not aligned. Organize the data
    if ((fifo[1] >> 6 & 0x03) == 0x1)
    {
        for (uint16_t i = 0; i < sizeof(fifo) - 4; i++)
        {
            fifo[i] = fifo[i + 4];
        }
    }
    else if ((fifo[1] >> 6 & 0x03) == 0x2)
    {
        for (uint16_t i = 0; i < sizeof(fifo) - 2; i++)
        {
            fifo[i] = fifo[i + 2];
        }
    }

    // 3.To 16-bit complement
    for (uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) * 3; i++)
    {
        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x0)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("X[%d] = %hd, %hx ", i/3, xAxis[i/3], xAxis[i/3]);
            //            rfid_printf("samples[%d] :%hd,", i / 3, three_axis_info[i / 3].x);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x1)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("Y[%d] = %hd, %hx ", i/3, yAxis[i/3], yAxis[i/3]);
            //            rfid_printf("%hd,", three_axis_info[i / 3].y);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x2)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("Z[%d] = %hd, %hx\n", i/3, zAxis[i/3], zAxis[i/3]);
            //            rfid_printf("%hd\n", three_axis_info[i / 3].z);
        }
    }

    // 4.Average Filter
    // 5.Dynamic Threshold

    CC1101Send3AxisHandler();

#else
    // 1.init acceler array

    // 2.To 16-bit complement
    for (uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) * 3; i++)
    {
        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x0)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("X[%d] = %hd, %hx ", i/3, xAxis[i/3], xAxis[i/3]);
            //            rfid_printf("samples[%d] :%hd,", i / 3, three_axis_info[i / 3].x);
        }

        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x1)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //            rfid_printf("samples[%d] :%hd\n", i / 3, three_axis_info[i / 3].y);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x2)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("Z[%d] = %hd, %hx\n", i/3, zAxis[i/3], zAxis[i/3]);
            //            rfid_printf("%hd\n", three_axis_info[i / 3].z);
        }
    }

    // 3.Average Calc
    // 4.Calculate whether to breathe
    // 初始化正弦波检测结构体
    memset(&sine_detection, 0, sizeof(sine_detection));
    // 初始化平滑滤波缓冲区
    memset(smoothed_x, 0, sizeof(smoothed_x));
    x_max_val = -2000;
    x_min_val = 2000;
    y_max_val = -2000;
    y_min_val = 2000;
    z_max_val = -2000;
    z_min_val = 2000;
    // 4.1 计算比较XYZ轴幅值
    for (uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
    {
        // 确保读取到最新的值
        int16_t current_x = three_axis_info[i].x;
        int16_t current_y = three_axis_info[i].y;
        if (current_x > x_max_val)
            x_max_val = current_x;
        if (current_x < x_min_val)
            x_min_val = current_x;

        if (current_y > y_max_val)
            y_max_val = current_y;
        if (current_y < y_min_val)
            y_min_val = current_y;
    }
    // 4.2 平滑滤波 - 使用移动平均滤波
    rfid_printf("x_max_val = %d\n", x_max_val);
    rfid_printf("x_min_val = %d\n", x_min_val);
    rfid_printf("y_max_val = %d\n", y_max_val);
    rfid_printf("y_min_val = %d\n", y_min_val);
    if (abs(x_max_val - x_min_val) > 2 * abs(y_max_val - y_min_val))
    {
        for (uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
        {
            // 对每个点应用移动平均滤波
            int32_t sum = 0;
            int16_t count = 0;

            // 计算窗口内的平均值
            for (int16_t j = -SMOOTH_WINDOW_SIZE / 2; j <= SMOOTH_WINDOW_SIZE / 2; j++)
            {
                int16_t idx = i + j;
                if (idx >= 0 && idx < (_FIFO_SAMPLES_LEN / 6))
                {
                    sum += three_axis_info[idx].x;
                    count++;
                }
            }

            // 计算平均值并存储到平滑后的数组
            smoothed_x[i] = (int16_t)(sum / count);
        }

        // 4.3 分析X轴数据是否为正弦波
        for (uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
        {
            // 第一个点初始化
            if (i == 0)
            {
                continue;
            }

            // 改进的平台期检测（扩大检测范围）
            if (i >= MAX_PLATEAU_WINDOW)
            {
                // 检查是否存在平台期（连续多个相似值）
                uint8_t is_plateau = 1;
                int16_t plateau_value_sum = 0;
                uint8_t plateau_count = 0;

                // 检查窗口内的值是否都在阈值范围内
                for (int16_t j = -MAX_PLATEAU_WINDOW; j < 0; j++)
                {
                    if (abs(smoothed_x[i + j] - smoothed_x[i + j + 1]) > PLATEAU_THRESHOLD)
                    {
                        is_plateau = 0;
                        break;
                    }
                    plateau_value_sum += smoothed_x[i + j];
                    plateau_count++;
                }

                // 如果是平台期，计算平台期的平均值
                if (is_plateau && plateau_count >= MIN_PLATEAU_COUNT)
                {
                    int16_t plateau_value = plateau_value_sum / plateau_count;

                    // 检查平台期前后的趋势
                    int16_t before_plateau = 0;
                    int16_t after_plateau = 0;

                    // 获取平台期前的值（如果可能）
                    if (i > MAX_PLATEAU_WINDOW + 2)
                    {
                        before_plateau = smoothed_x[i - MAX_PLATEAU_WINDOW - 2];
                    }

                    // 获取平台期后的值（当前值）
                    after_plateau = smoothed_x[i];

                    // 判断平台期是峰值还是谷值
                    if (before_plateau < plateau_value && after_plateau < plateau_value)
                    {
                        // 平台期是峰值
                        if (sine_detection.is_rising && sine_detection.peak_count < MAX_PEAK_COUNT)
                        {
                            sine_detection.peaks[sine_detection.peak_count].value = plateau_value;
                            sine_detection.peaks[sine_detection.peak_count].index = i - MAX_PLATEAU_WINDOW / 2;

                            // 计算周期（从峰值到峰值）
                            if (sine_detection.peak_count >= 1 && sine_detection.period_count < MAX_PERIOD_COUNT)
                            {
                                sine_detection.periods[sine_detection.period_count] =
                                    sine_detection.peaks[sine_detection.peak_count].index -
                                    sine_detection.peaks[sine_detection.peak_count - 1].index;
                                sine_detection.period_count++;
                            }
                            sine_detection.peak_count++;
                            sine_detection.is_rising = 0;
                        }
                    }
                    else if (before_plateau > plateau_value && after_plateau > plateau_value)
                    {
                        // 平台期是谷值
                        if (!sine_detection.is_rising && sine_detection.valley_count < MAX_VALLEY_COUNT)
                        {
                            sine_detection.valleys[sine_detection.valley_count].value = plateau_value;
                            sine_detection.valleys[sine_detection.valley_count].index = i - MAX_PLATEAU_WINDOW / 2;
                            sine_detection.valley_count++;
                            sine_detection.is_rising = 1;
                        }
                    }
                }
            }

            // 改进的极值检测（扩大比较范围）
            if (i >= EXTREMA_WINDOW_SIZE && i < (_FIFO_SAMPLES_LEN / 6) - EXTREMA_WINDOW_SIZE)
            {
                // 检测是否为局部峰值
                uint8_t is_peak = 1;
                for (int16_t j = -EXTREMA_WINDOW_SIZE; j <= EXTREMA_WINDOW_SIZE; j++)
                {
                    if (j != 0 && smoothed_x[i + j] > smoothed_x[i])
                    {
                        is_peak = 0;
                        break;
                    }
                }

                if (is_peak && sine_detection.is_rising && sine_detection.peak_count < MAX_PEAK_COUNT)
                {
                    // 检查是否与已有峰值过近
                    uint8_t too_close = 0;
                    for (uint16_t j = 0; j < sine_detection.peak_count; j++)
                    {
                        if (abs((int)i - (int)sine_detection.peaks[j].index) < EXTREMA_WINDOW_SIZE)
                        {
                            too_close = 1;
                            // 如果新峰值更高，替换旧峰值
                            if (smoothed_x[i] > sine_detection.peaks[j].value)
                            {
                                sine_detection.peaks[j].value = smoothed_x[i];
                                sine_detection.peaks[j].index = i;
                            }
                            break;
                        }
                    }

                    if (!too_close)
                    {
                        sine_detection.peaks[sine_detection.peak_count].value = smoothed_x[i];
                        sine_detection.peaks[sine_detection.peak_count].index = i;

                        // 计算周期（从峰值到峰值）
                        if (sine_detection.peak_count >= 1 && sine_detection.period_count < MAX_PERIOD_COUNT)
                        {
                            sine_detection.periods[sine_detection.period_count] =
                                sine_detection.peaks[sine_detection.peak_count].index -
                                sine_detection.peaks[sine_detection.peak_count - 1].index;
                            sine_detection.period_count++;
                        }
                        sine_detection.peak_count++;
                    }
                    sine_detection.is_rising = 0;
                }

                // 检测是否为局部谷值
                uint8_t is_valley = 1;
                for (int16_t j = -EXTREMA_WINDOW_SIZE; j <= EXTREMA_WINDOW_SIZE; j++)
                {
                    if (j != 0 && smoothed_x[i + j] < smoothed_x[i])
                    {
                        is_valley = 0;
                        break;
                    }
                }

                if (is_valley && !sine_detection.is_rising && sine_detection.valley_count < MAX_VALLEY_COUNT)
                {
                    // 检查是否与已有谷值过近
                    uint8_t too_close = 0;
                    for (uint16_t j = 0; j < sine_detection.valley_count; j++)
                    {
                        if (abs((int)i - (int)sine_detection.valleys[j].index) < EXTREMA_WINDOW_SIZE)
                        {
                            too_close = 1;
                            // 如果新谷值更低，替换旧谷值
                            if (smoothed_x[i] < sine_detection.valleys[j].value)
                            {
                                sine_detection.valleys[j].value = smoothed_x[i];
                                sine_detection.valleys[j].index = i;
                            }
                            break;
                        }
                    }

                    if (!too_close)
                    {
                        sine_detection.valleys[sine_detection.valley_count].value = smoothed_x[i];
                        sine_detection.valleys[sine_detection.valley_count].index = i;
                        sine_detection.valley_count++;
                    }
                    sine_detection.is_rising = 1;
                }
            }
        }

        // 4.4 对峰值和谷值按索引排序，确保周期计算的连续性
        for (uint16_t i = 0; i < sine_detection.peak_count - 1; i++)
        {
            for (uint16_t j = 0; j < sine_detection.peak_count - i - 1; j++)
            {
                if (sine_detection.peaks[j].index > sine_detection.peaks[j + 1].index)
                {
                    extreme_point_t temp = sine_detection.peaks[j];
                    sine_detection.peaks[j] = sine_detection.peaks[j + 1];
                    sine_detection.peaks[j + 1] = temp;
                }
            }
        }

        for (uint16_t i = 0; i < sine_detection.valley_count - 1; i++)
        {
            for (uint16_t j = 0; j < sine_detection.valley_count - i - 1; j++)
            {
                if (sine_detection.valleys[j].index > sine_detection.valleys[j + 1].index)
                {
                    extreme_point_t temp = sine_detection.valleys[j];
                    sine_detection.valleys[j] = sine_detection.valleys[j + 1];
                    sine_detection.valleys[j + 1] = temp;
                }
            }
        }

        // 4.5 重新计算周期（基于排序后的峰值）
        sine_detection.period_count = 0;
        for (uint16_t i = 1; i < sine_detection.peak_count && sine_detection.period_count < MAX_PERIOD_COUNT; i++)
        {
            sine_detection.periods[sine_detection.period_count] =
                sine_detection.peaks[i].index - sine_detection.peaks[i - 1].index;
            sine_detection.period_count++;
        }

        // 4.6 分析是否为正弦波
        if (sine_detection.peak_count >= 3 && sine_detection.valley_count >= 3)
        {
            // 计算平均周期
            float avg_period = 0;
            for (uint8_t j = 0; j < sine_detection.period_count; j++)
            {
                avg_period += sine_detection.periods[j];
            }
            avg_period /= sine_detection.period_count;

            // 计算频率 (采样率为25Hz)
            sine_detection.frequency = 25.0f / avg_period;

            // 计算峰值和谷值的平均幅度
            int32_t peak_sum = 0, valley_sum = 0;
            for (uint8_t j = 0; j < sine_detection.peak_count; j++)
            {
                peak_sum += sine_detection.peaks[j].value;
            }
            for (uint8_t j = 0; j < sine_detection.valley_count; j++)
            {
                valley_sum += sine_detection.valleys[j].value;
            }
            int16_t avg_peak = peak_sum / sine_detection.peak_count;
            int16_t avg_valley = valley_sum / sine_detection.valley_count;
            sine_detection.amplitude = avg_peak - avg_valley;

            // 判断是否为正弦波：
            // 1. 频率在1.3-1.7Hz范围内
            // 2. 幅度足够大
            // 3. 峰值和谷值的标准差较小（波形规则）
            if (sine_detection.frequency >= SINE_WAVE_MIN_FREQ &&
                sine_detection.frequency <= SINE_WAVE_MAX_FREQ &&
                sine_detection.amplitude > SINE_WAVE_MIN_AMPLITUDE)
            {
                // 计算峰值和谷值的标准差
                float peak_std = 0, valley_std = 0;
                for (uint8_t j = 0; j < sine_detection.peak_count; j++)
                {
                    peak_std += (sine_detection.peaks[j].value - avg_peak) * (sine_detection.peaks[j].value - avg_peak);
                }
                for (uint8_t j = 0; j < sine_detection.valley_count; j++)
                {
                    valley_std += (sine_detection.valleys[j].value - avg_valley) * (sine_detection.valleys[j].value - avg_valley);
                }
                peak_std = sqrtf(peak_std / sine_detection.peak_count);
                valley_std = sqrtf(valley_std / sine_detection.valley_count);

                // 标准差小于幅度的阈值百分比认为是规则波形
                if (peak_std < sine_detection.amplitude * STD_DEV_THRESHOLD &&
                    valley_std < sine_detection.amplitude * STD_DEV_THRESHOLD)
                {
                    sine_detection.is_sine_wave = 1;
                }
            }

            // 输出检测结果
            rfid_printf("Sine wave detection result:\n");
            rfid_printf("Peak number: %d, valley number: %d\n", sine_detection.peak_count, sine_detection.valley_count);
            rfid_printf("Average period: %.2f Sampling point\n", avg_period);
            rfid_printf("Frequency: %.2f Hz\n", sine_detection.frequency);
            rfid_printf("Amplitude: %d\n", sine_detection.amplitude);
            rfid_printf("Is it a sine wave(1.0-1.7Hz): %d\n", sine_detection.is_sine_wave);
        }
    }

    x_max_val = -2000;
    x_min_val = 2000;
    y_max_val = -2000;
    y_min_val = 2000;
    z_max_val = -2000;
    z_min_val = 2000;
    // 5.Difference Derivation And Threshold Judge
    for (uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
    {
        diff_three_axis_info[1] = diff_three_axis_info[0];
        diff_three_axis_info[0] = three_axis_info[i];

        if (three_axis_info[i].x > x_max_val)
            x_max_val = three_axis_info[i].x;

        if (three_axis_info[i].x < x_min_val)
            x_min_val = three_axis_info[i].x;

        if (three_axis_info[i].y > y_max_val)
            y_max_val = three_axis_info[i].y;

        if (three_axis_info[i].y < y_min_val)
            y_min_val = three_axis_info[i].y;

        if (three_axis_info[i].z > z_max_val)
            z_max_val = three_axis_info[i].z;

        if (three_axis_info[i].z < z_min_val)
            z_min_val = three_axis_info[i].z;

        three_axis_average_info.x += three_axis_info[i].x;

        three_axis_info[i].x = diff_three_axis_info[0].x - diff_three_axis_info[1].x;
        three_axis_info[i].y = diff_three_axis_info[0].y - diff_three_axis_info[1].y;
        three_axis_info[i].z = diff_three_axis_info[0].z - diff_three_axis_info[1].z;

        sum_info.x += abs(three_axis_info[i].x);
        sum_info.y += abs(three_axis_info[i].y);
        sum_info.z += abs(three_axis_info[i].z);

        rfid_printf("samples[%d] :%hd\n", i, three_axis_info[i].x);

        if (abs(three_axis_info[i].x) <= 10)
            threshold_judge.low++;
        else if (abs(three_axis_info[i].x) <= 100)
            threshold_judge.normal++;
        else if (abs(three_axis_info[i].x) <= 200)
            threshold_judge.abovenormal++;
        else
            threshold_judge.high++;

        if (((i + 1) / 25) != 0 && ((i + 1) % 25) == 0)
        {
            rfid_printf("threshold_judge.low = %d\n", threshold_judge.low);
            rfid_printf("threshold_judge.normal = %d\n", threshold_judge.normal);
            rfid_printf("threshold_judge.abovenormal = %d\n", threshold_judge.abovenormal);
            rfid_printf("threshold_judge.high = %d\n", threshold_judge.high);

            three_axis_average_info.x = three_axis_average_info.x / 25;
            rfid_printf("average_info.y = %d\n", three_axis_average_info.x);
            rfid_printf("sum_info.y = %d\n", sum_info.x);

            if (sine_detection.is_sine_wave == 1)
                action = 7; // breath
            else if (threshold_judge.low >= 24)
                action = 1; // rest
            else if ((threshold_judge.normal + threshold_judge.abovenormal) > 11 && threshold_judge.high == 0 &&
                     three_axis_average_info.x >= 200)
                action = 2; // ingestion
            else if (three_axis_average_info.x > -200 && three_axis_average_info.x < 100 &&
                     (sum_info.x + sum_info.y + sum_info.z) / 3 > 400 &&
                     (((x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val)) / 3) > 150)
                action = 3; // movement
            else if (threshold_judge.high > 0 && three_axis_average_info.x <= -200)
                action = 4; // climb
            else
                action = 6; // other

            rfid_printf("action = %d\n", action);
            rfid_printf("action_classify.rest = %d\n", action_classify.rest);
            rfid_printf("action_classify.ingestion = %d\n", action_classify.ingestion);
            rfid_printf("action_classify.movement = %d\n", action_classify.movement);
            rfid_printf("action_classify.climb = %d\n", action_classify.climb);
            rfid_printf("action_classify.other = %d\n", action_classify.other);

            // circular storage
            memory_array[memory_index][0] = action;
            memory_array[memory_index][1] = three_axis_average_info.x;
            memory_array[memory_index][2] = (sum_info.x + sum_info.y + sum_info.z) / 3;
            memory_array[memory_index][3] = ((x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val)) / 3;

            rfid_printf("memory_array1:\n");

            // print memory_array
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
            {
                for (uint8_t j = 0; j < _MEM_COLS; j++)
                {
                    rfid_printf("%d ", memory_array[i][j]);
                }

                rfid_printf("\n");
            }

            // redundant movement among rest
            movement_cnt = 0;
            rest_cnt1 = 0;
            rest_cnt2 = 0;

            if (memory_index >= 9)
                mid_index = memory_index - 9;
            else
                mid_index = memory_index + 9;

            rfid_printf("memory_index = %d\n", memory_index);
            rfid_printf("mid_index = %d\n", mid_index);

            if (memory_array[mid_index][0] == 3)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    if (memory_array[i][0] == 3)
                        movement_cnt += 1;

                rfid_printf("movement_cnt = %d\n", movement_cnt);

                if (movement_cnt == 1)
                {
                    if (mid_index < memory_index)
                    {
                        for (uint8_t i = mid_index; i < memory_index; i++)
                            if (memory_array[i][0] == 1)
                                rest_cnt1 += 1;

                        for (uint8_t i = 0; i < mid_index; i++)
                            if (memory_array[i][0] == 1)
                                rest_cnt2 += 1;

                        for (uint8_t i = memory_index; i < _MEM_ROWS; i++)
                            if (memory_array[i][0] == 1)
                                rest_cnt2 += 1;
                    }
                    else if (memory_index < mid_index)
                    {
                        for (uint8_t i = memory_index; i < mid_index; i++)
                            if (memory_array[i][0] == 1)
                                rest_cnt2 += 1;

                        for (uint8_t i = 0; i < memory_index; i++)
                            if (memory_array[i][0] == 1)
                                rest_cnt1 += 1;

                        for (uint8_t i = mid_index; i < _MEM_ROWS; i++)
                            if (memory_array[i][0] == 1)
                                rest_cnt1 += 1;
                    }

                    rfid_printf("rest_cnt1 = %d\n", rest_cnt1);
                    rfid_printf("rest_cnt2 = %d\n", rest_cnt2);

                    if (rest_cnt1 >= 4 && rest_cnt2 >= 4)
                        memory_array[mid_index][0] = 1;
                }
            }

            // redundant climb if movement less than 4
            // eliminate redundant climb
            movement_cnt = 0;
            climb_cnt = 0;

            if (memory_array[memory_index][0] == 4)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                {
                    if (memory_array[i][0] == 3)
                        movement_cnt += 1;

                    if (memory_array[i][0] == 4)
                        climb_cnt += 1;
                }

                if (movement_cnt < 4)
                    memory_array[memory_index][0] = 6;
                else if (climb_cnt >= 2)
                    memory_array[memory_index][0] = 3;
            }

            // index update
            if (memory_index >= 17)
                memory_index = 0;
            else
                memory_index += 1;

            // if ingestion_cnt >= 2 ==> 3 to 6
            ingestion_cnt = 0;

            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if (memory_array[i][0] == 2)
                    ingestion_cnt += 1;

            if (ingestion_cnt >= 2)
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    if (memory_array[i][0] == 3)
                        memory_array[i][0] = 6;

            // ruminate logic
            rest_cnt = 0;
            deta_a_cnt = 0;
            jicha_cnt = 0;
            eighteen_average = 0;
            sum_eighteen_average = 0;

            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if (memory_array[i][0] == 1)
                    rest_cnt += 1;

            if (rest_cnt <= 4 && sine_detection.is_sine_wave == 0)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                {
                    if (memory_array[i][2] > 130 && memory_array[i][2] < 700)
                        deta_a_cnt += 1;

                    if (memory_array[i][3] < 120)
                        jicha_cnt += 1;
                }

                if (deta_a_cnt >= 14 && jicha_cnt >= 14)
                {
                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        eighteen_average += memory_array[i][1];

                    eighteen_average = eighteen_average / _MEM_ROWS;

                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        sum_eighteen_average += abs(eighteen_average - memory_array[i][1]);

                    if (sum_eighteen_average <= 400 && eighteen_average < 150)
                        for (uint8_t i = 0; i < _MEM_ROWS; i++)
                            memory_array[i][0] = 5;
                }
            }

            // delay 18s output
            if (memory_index == 17)
                memory_index_o = 0;
            else
                memory_index_o = memory_index + 1;

            action_classify_array[((i + 1) / 25) - 1] = memory_array[memory_index_o][0];

            if (memory_array[memory_index_o][0] == 7) // breath
                action_classify.other++;
            else if (memory_array[memory_index_o][0] == 1) // rest
                action_classify.rest++;
            else if (memory_array[memory_index_o][0] == 2) // ingestion
                action_classify.ingestion++;
            else if (memory_array[memory_index_o][0] == 3) // movement
                action_classify.movement++;
            else if (memory_array[memory_index_o][0] == 4) // climb
                action_classify.climb++;
            else if (memory_array[memory_index_o][0] == 5) // ruminate
                action_classify.ruminate++;
            // else // other
            //     action_classify.other++;

            memset(&threshold_judge, 0, sizeof(threshold_judge));
            memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
            memset(&sum_info, 0, sizeof(sum_info));
            x_max_val = -2000;
            x_min_val = 2000;
            y_max_val = -2000;
            y_min_val = 2000;
            z_max_val = -2000;
            z_min_val = 2000;
        }
    }

    for (uint8_t i = 0; i < sizeof(action_classify_array); i++)
    {
        rfid_printf("action_classify_array[%d] = %d\n", i, action_classify_array[i]);
    }

//    CC1101Send3AxisHandler();

    //    ADXL362RegisterRead(XL362_STATUS);
    //    rfid_printf("XL362_STATUS: %x\n", ADXL362RegisterRead(XL362_STATUS));

#endif
}
