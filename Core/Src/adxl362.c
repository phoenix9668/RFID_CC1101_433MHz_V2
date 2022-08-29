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

typedef struct
{
    axis_info_t info[_FILTER_CNT];
    uint16_t count;
} filter_avg_t;

typedef struct
{
    uint8_t sample_size;
    axis_info_t newmax;
    axis_info_t newmin;
    axis_info_t oldmax;
    axis_info_t oldmin;
} peak_value_t;

typedef struct
{
    axis_info_t new_sample;
    axis_info_t old_sample;
} slid_reg_t;

typedef struct
{
    uint32_t sample_num;
    uint32_t valid_step_num;
} valid_step_filter_t;

axis_info_t axis_info;
axis_info_t three_axis_info[_AXIS_LEN];
filter_avg_t filter_avg;
peak_value_t peak_value;
slid_reg_t slid_reg;
valid_step_filter_t valid_step_filter;

uint8_t fifo[_FIFO_LEN];
y_axis_calc_t y_axis_calc;
step_t step;

extern uint8_t ErrorIndex;

void filter_calculate(filter_avg_t *filter, axis_info_t *sample);
void peak_value_init(peak_value_t *peak);
void peak_update(peak_value_t *peak, axis_info_t *cur_sample);
uint8_t slid_update(slid_reg_t *slid, axis_info_t *cur_sample, valid_step_filter_t *vsf);
uint8_t is_most_active(peak_value_t *peak);
void detect_step(peak_value_t *peak, slid_reg_t *slid, valid_step_filter_t *vsf);

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

    ADXL362_CSN_LOW();              		//CS down
    SendTemp[0] = 0x0B;                 //0x0B: read register command
    SendTemp[1] = Address;              //address byte
    SpiFunction(SPI2, SendTemp, ReceiveTemp, 2, 1);
    ReceiveValue = ReceiveTemp[0];
    ADXL362_CSN_HIGH();
    return(ReceiveValue);               //CS up
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

    ADXL362_CSN_LOW();              			//CS down
    SendTemp[0] = 0x0A;                 //0x0A: write register
    SendTemp[1] = Address;              //address byte
    SendTemp[2] = SendValue;

    SpiFunction(SPI2, SendTemp, ReceiveTemp, 3, 0);
    ADXL362_CSN_HIGH();              		//CS up
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

    ADXL362_CSN_LOW();         			//CS down
    SendTemp[0] = 0x0B;            	//0x0B: read register
    SendTemp[1] = Address;         	//address byte
    SpiFunction(SPI2, SendTemp, RegisterData, 2, NumberofRegisters);
    ADXL362_CSN_HIGH();        			//CS up
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

    ADXL362_CSN_LOW();										//CS down
    SendTemp[0] = 0x0A;                 //0x0A: write register
    SendTemp[1] = Address;              //address byte

    for (RegisterIndex = 0; RegisterIndex < NumberofRegisters; RegisterIndex++)
    {
        SendTemp[2 + RegisterIndex] = *(RegisterData + RegisterIndex);
    }

    SpiFunction(SPI2, SendTemp, ReceiveTemp, (2 + NumberofRegisters), 0);
    ADXL362_CSN_HIGH();              		//CS up
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
    ADXL362_CSN_LOW();         			//CS down
    SendTemp[0] = 0x0D;            	//0x0D: read register
    SpiFunction(SPI2, SendTemp, RegisterData, 1, NumberofRegisters);
    ADXL362_CSN_HIGH();        			//CS up
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
    return(ReadValueTemp);
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
    memset(&peak_value, 0, sizeof(peak_value));
    peak_value_init(&peak_value);
    memset(&slid_reg, 0, sizeof(slid_reg));
    memset(&valid_step_filter, 0, sizeof(valid_step_filter));

    ADXL362RegisterWrite(XL362_SOFT_RESET, 0x52);   						// software reset
    HAL_Delay(1000);

    #if (_Original_Data_Algorithm == 0)

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, 0x64);						//set active threshold equip 100mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_L);
    rfid_printf("|*-set THRESH_ACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_ACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_H);
    rfid_printf("|*-set THRESH_ACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x03);						//set active time equip 3/12.5s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x03)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_L, 0x64);					//set inactive threshold equip 100mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_L);
    rfid_printf("|*-set THRESH_INACT_L register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_H);
    rfid_printf("|*-set THRESH_INACT_H register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x03);						//set inactive time equip 3/12.5s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x03)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_H);
    rfid_printf("|*-set TIME_INACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x3F);						//configure loop mode,enable active and inactive
    ReadValueTemp = ADXL362RegisterRead(XL362_ACT_INACT_CTL);
    rfid_printf("|*-set ACT_INACT_CTL register = 0x%02x--*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x3F)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP1, 0x10);									//configure act map INT1
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP1);
    rfid_printf("|*-set INTMAP1 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP2, 0x08);									//configure fifo_overrun map INT2
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP2);
    rfid_printf("|*-set INTMAP2 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x08)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x03);						//select fifo Triggered Mode,not store Temperature Data to FIFO
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_CONTROL);
    rfid_printf("|*-set FIFO_CONTROL register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x03)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0x18);						//select fifo sample number//0x18 = 24
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_SAMPLES);
    rfid_printf("|*-set FIFO_SAMPLES register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x18)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x10);             	//select 2g range,ODR:12.5Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02);              	//select measurement mode
    ReadValueTemp = ADXL362RegisterRead(XL362_POWER_CTL);
    rfid_printf("|*-set POWER_CTL register = 0x%02x------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x02)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    #elif (_Original_Data_Algorithm == 1)

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, 0x64);						//set active threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_L);
    rfid_printf("|*-set THRESH_ACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_ACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_H);
    rfid_printf("|*-set THRESH_ACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x0C);						//set active time equip 12/50s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x0C)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_L, 0x64);					//set inactive threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_L);
    rfid_printf("|*-set THRESH_INACT_L register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_H);
    rfid_printf("|*-set THRESH_INACT_H register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x0C);						//set inactive time equip 12/50s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x0C)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_H);
    rfid_printf("|*-set TIME_INACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x3F);						//configure loop mode,enable active and inactive
    ReadValueTemp = ADXL362RegisterRead(XL362_ACT_INACT_CTL);
    rfid_printf("|*-set ACT_INACT_CTL register = 0x%02x--*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x3F)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP1, 0x10);									//configure act map INT1
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP1);
    rfid_printf("|*-set INTMAP1 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP2, 0x04);									//configure fifo_watermark map INT2
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP2);
    rfid_printf("|*-set INTMAP2 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x04)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x0A);						//Above Half, select fifo Steam Mode, not store Temperature Data to FIFO
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_CONTROL);
    rfid_printf("|*-set FIFO_CONTROL register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x0A)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0xC2);						//select fifo sample number//0x1C2 = 450
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_SAMPLES);
    rfid_printf("|*-set FIFO_SAMPLES register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0xC2)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x52);             	//select 4g range,ODR:50Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x52)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02);              	//select measurement mode
    ReadValueTemp = ADXL362RegisterRead(XL362_POWER_CTL);
    rfid_printf("|*-set POWER_CTL register = 0x%02x------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x02)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    #else

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
    /* @brief integration of accelerations
     * @param input fifo
     * @param output acceler
     */

    bool thresholdMark = false;
    // 1.init acceler array
    memset(three_axis_info, 0, sizeof(three_axis_info));
    memset(&y_axis_calc, 0, sizeof(y_axis_calc));

    // 2.If the fifo data is not aligned. Organize the data
    if ((fifo[1] >> 6 & 0x03) == 0x1)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 4; i++)
        {
            fifo[i] = fifo[i + 4];
        }
    }
    else if ((fifo[1] >> 6 & 0x03) == 0x2)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 2; i++)
        {
            fifo[i] = fifo[i + 2];
        }
    }

    // 3.To 16-bit complement
    for(uint16_t i = 0; i < (sizeof(fifo) / 6) * 3; i++)
    {
        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x1)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            rfid_printf("samples[%d] :%hd\n", i / 3, three_axis_info[i / 3].y);
        }
    }

    /*
    // 4.calculate threeAxisUnity array
    for(uint8_t i = 0; i < _AXIS_LEN; i++)
    {
        threeAxisUnity[i] = (uint16_t)sqrt(pow(xAxis[i], 2) + pow(yAxis[i], 2) + pow(zAxis[i], 2));
        rfid_printf("threeAxisUnity[%d] : %hd\n", i, threeAxisUnity[i]);
    }
    */

    // 4.Calculate step average values array
    for (uint8_t i = 0; i < ARRAY_LEN(y_axis_calc.yStepAverage); i++)
    {
        y_axis_calc.yStepAverage[i] = (three_axis_info[i].y + three_axis_info[i + 1].y + three_axis_info[i + 2].y) / 3;
//        rfid_printf("yStepAverage[%d] :%hd\n", i, yStepAverage[i]);
    }

    // 5.Calculate step filter values array
    for (uint8_t i = 0; i < ARRAY_LEN(y_axis_calc.yStepFilter); i++)
    {
        y_axis_calc.yStepFilter[i] =  y_axis_calc.yStepAverage[i] - three_axis_info[i + 2].y;
//        rfid_printf("yStepFilter[%d] :%hd,%hx\n", i, yStepFilter[i], yStepFilter[i]);
    }

    // 6.Calculate the stepNum
    for (uint8_t i = 0; i < ARRAY_LEN(y_axis_calc.yStepFilter); i++)
    {
        if(y_axis_calc.yStepFilter[i] >= 500)
        {
            if(thresholdMark == false)
            {
                step.stepNum++;
                thresholdMark = true;
                rfid_printf("stepNum = %d\n", step.stepNum);
            }
        }
        else
        {
            thresholdMark = false;
        }
    }

    // 7.Calculate ingestion average values array
    for (uint8_t i = 2; i < ARRAY_LEN(y_axis_calc.yIngestionAverage); i++)
    {
        for (uint8_t j = 0; j < 25; j++)
        {
            if(((i - 2) * 25 + j) < 170)
            {
                y_axis_calc.yIngestionAverage[i] += three_axis_info[(i - 2) * 25 + j].y;
            }
        }

        if(i == (ARRAY_LEN(y_axis_calc.yIngestionAverage) - 1))
            y_axis_calc.yIngestionAverage[i] = y_axis_calc.yIngestionAverage[i] / 20;
        else
            y_axis_calc.yIngestionAverage[i] = y_axis_calc.yIngestionAverage[i] / 25;
    }

    y_axis_calc.yIngestionAverage[0] = y_axis_calc.yIngestionAverageOld[0];
    y_axis_calc.yIngestionAverage[1] = y_axis_calc.yIngestionAverageOld[1];

    for (uint8_t i = 0; i < ARRAY_LEN(y_axis_calc.yIngestionAverage); i++)
    {
        rfid_printf("yIngestionAverage[%d] :%hd\n", i, y_axis_calc.yIngestionAverage[i]);
    }

    // 8.Calculate the ingestionNum
    thresholdMark = false;

    for (uint8_t i = 0; i < ARRAY_LEN(y_axis_calc.yIngestionAverage) - 2; i++)
    {
        if(abs(y_axis_calc.yIngestionAverage[i + 2] - y_axis_calc.yIngestionAverage[i]) >= 300)
        {
            if(thresholdMark == false)
            {
                step.ingestionNum++;
                thresholdMark = true;
                rfid_printf("ingestionNum = %d\n", step.ingestionNum);
            }
        }
        else
        {
            thresholdMark = false;
        }
    }

    // 9.Store the yIngestionAverage
    y_axis_calc.yIngestionAverageOld[0] = y_axis_calc.yIngestionAverage[(ARRAY_LEN(y_axis_calc.yIngestionAverage) - 2)];
    y_axis_calc.yIngestionAverageOld[1] = y_axis_calc.yIngestionAverage[(ARRAY_LEN(y_axis_calc.yIngestionAverage) - 1)];

    #elif (_Original_Data_Algorithm == 1)
    // 1.init acceler array
    memset(three_axis_info, 0, sizeof(three_axis_info));
    memset(&filter_avg, 0, sizeof(filter_avg));

    // 2.If the fifo data is not aligned. Organize the data
    if ((fifo[1] >> 6 & 0x03) == 0x1)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 4; i++)
        {
            fifo[i] = fifo[i + 4];
        }
    }
    else if ((fifo[1] >> 6 & 0x03) == 0x2)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 2; i++)
        {
            fifo[i] = fifo[i + 2];
        }
    }

    // 3.To 16-bit complement
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) * 3; i++)
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
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
    {
        filter_avg.info[3] =  filter_avg.info[2];
        filter_avg.info[2] =  filter_avg.info[1];
        filter_avg.info[1] =  filter_avg.info[0];
        filter_avg.info[0] =  three_axis_info[i];

        if(i >= (_FILTER_CNT - 1))
        {
            filter_avg.count++;
            filter_calculate(&filter_avg, &axis_info);
            three_axis_info[i - (_FILTER_CNT - 1)] = axis_info;
            rfid_printf("average filter[%d] :%hd,", i - (_FILTER_CNT - 1), three_axis_info[i - (_FILTER_CNT - 1)].x);
            rfid_printf("%hd,", three_axis_info[i - (_FILTER_CNT - 1)].y);
            rfid_printf("%hd\n", three_axis_info[i - (_FILTER_CNT - 1)].z);
        }
    }

    rfid_printf("filter_avg.count = %d\n", filter_avg.count);

    // 5.Dynamic Threshold
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) - _FILTER_CNT + 1; i++)
    {
        peak_update(&peak_value, &three_axis_info[i]);

        if (slid_update(&slid_reg, &three_axis_info[i], &valid_step_filter)  == 1)
        {
            detect_step(&peak_value, &slid_reg, &valid_step_filter);
        }
    }

    #else
    memset(xAxis, 0, sizeof(xAxis));
    memset(yAxis, 0, sizeof(yAxis));
    memset(zAxis, 0, sizeof(zAxis));

    // If the fifo data is not aligned. Organize the data
    if ((fifo[1] >> 6 & 0x03) == 0x1)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 4; i++)
        {
            fifo[i] = fifo[i + 4];
        }
    }
    else if ((fifo[1] >> 6 & 0x03) == 0x2)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 2; i++)
        {
            fifo[i] = fifo[i + 2];
        }
    }

    CC1101Send3AxisHandler();

    /*
    for(uint16_t i = 0; i < sizeof(fifo) / 6; i++)
    {
        rfid_printf("F[%d] = %x ", 6 * i, fifo[6 * i]);
        rfid_printf("F[%d] = %x ", 6 * i + 1, fifo[6 * i + 1]);
        rfid_printf("F[%d] = %x ", 6 * i + 2, fifo[6 * i + 2]);
        rfid_printf("F[%d] = %x ", 6 * i + 3, fifo[6 * i + 3]);
        rfid_printf("F[%d] = %x ", 6 * i + 4, fifo[6 * i + 4]);
        rfid_printf("F[%d] = %x\n", 6 * i + 5, fifo[6 * i + 5]);
    }

    // 1.To 16-bit complement
    for(uint16_t i = 0; i < (sizeof(fifo) / 6) * 3; i++)
    {
        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x0)
        {
            if ((fifo[2 * i + 1] & 0x08))
                xAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                xAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

    //				rfid_printf("X[%d] = %hd, %hx ", i/3, xAxis[i/3], xAxis[i/3]);
            rfid_printf("samples[%d] :%hd,", i / 3, xAxis[i / 3]);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x1)
        {
            if ((fifo[2 * i + 1] & 0x08))
                yAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                yAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

    //				rfid_printf("Y[%d] = %hd, %hx ", i/3, yAxis[i/3], yAxis[i/3]);
            rfid_printf("%hd,", yAxis[i / 3]);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x2)
        {
            if ((fifo[2 * i + 1] & 0x08))
                zAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                zAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

    //				rfid_printf("Z[%d] = %hd, %hx\n", i/3, zAxis[i/3], zAxis[i/3]);
            rfid_printf("%hd\n", zAxis[i / 3]);
        }
    }

    ADXL362RegisterRead(XL362_STATUS);
    rfid_printf("XL362_STATUS: %x\n", ADXL362RegisterRead(XL362_STATUS));
    */
    #endif
}

/*******************************************************************
  @brief void filter_calculate(filter_avg_t *filter, axis_info_t *sample)
         average filter
  @param
				 filter_avg_t *filter
				 axis_info_t *sample
  @return
				 none
*******************************************************************/
void filter_calculate(filter_avg_t *filter, axis_info_t *sample)
{
    short int x_sum = 0, y_sum = 0, z_sum = 0;

    for (uint8_t i = 0; i < _FILTER_CNT; i++)
    {
        x_sum += filter->info[i].x;
        y_sum += filter->info[i].y;
        z_sum += filter->info[i].z;
    }

    sample->x = x_sum / _FILTER_CNT;
    sample->y = y_sum / _FILTER_CNT;
    sample->z = z_sum / _FILTER_CNT;
}

/*******************************************************************
  @brief void peak_update(peak_value_t *peak, axis_info_t *cur_sample)
         peak calc
  @param
				 peak_value_t *peak
				 axis_info_t *cur_sample
  @return
				 none
*******************************************************************/
void peak_value_init(peak_value_t *peak)
{
    peak->newmax.x = INT16_MIN;
    peak->newmax.y = INT16_MIN;
    peak->newmax.z = INT16_MIN;

    peak->newmin.x = INT16_MAX;
    peak->newmin.y = INT16_MAX;
    peak->newmin.z = INT16_MAX;
}

void peak_update(peak_value_t *peak, axis_info_t *cur_sample)
{
    peak->sample_size++;

    if (peak->sample_size >= _SAMPLE_SIZE)
    {
        peak->sample_size = 0;
        peak->oldmax = peak->newmax;
        peak->oldmin = peak->newmin;
        peak_value_init(peak);
        rfid_printf(" peak->oldmax.x = %d,",  peak->oldmax.x);
        rfid_printf(" peak->oldmin.x = %d\n",  peak->oldmin.x);
        rfid_printf(" peak->oldmax.y = %d,",  peak->oldmax.y);
        rfid_printf(" peak->oldmin.y = %d\n",  peak->oldmin.y);
        rfid_printf(" peak->oldmax.z = %d,",  peak->oldmax.z);
        rfid_printf(" peak->oldmin.z = %d\n",  peak->oldmin.z);
    }

    peak->newmax.x = max(peak->newmax.x, cur_sample->x);
    peak->newmax.y = max(peak->newmax.y, cur_sample->y);
    peak->newmax.z = max(peak->newmax.z, cur_sample->z);

    peak->newmin.x = min(peak->newmin.x, cur_sample->x);
    peak->newmin.y = min(peak->newmin.y, cur_sample->y);
    peak->newmin.z = min(peak->newmin.z, cur_sample->z);
}

/*******************************************************************
  @brief uint8_t slid_update(slid_reg_t *slid, axis_info_t *cur_sample)
         slid calc
  @param
				 slid_reg_t *slid
				 axis_info_t *cur_sample
  @return
				 uint8_t
*******************************************************************/
uint8_t slid_update(slid_reg_t *slid, axis_info_t *cur_sample, valid_step_filter_t *vsf)
{
    uint8_t res = 0;
    vsf->sample_num++;

    if (abs((cur_sample->x - slid->new_sample.x)) > _DYNAMIC_PRECISION)
    {
        slid->old_sample.x = slid->new_sample.x;
        slid->new_sample.x = cur_sample->x;
        res = 1;
    }
    else
    {
        slid->old_sample.x = slid->new_sample.x;
    }

    if (abs((cur_sample->y - slid->new_sample.y)) > _DYNAMIC_PRECISION)
    {
        slid->old_sample.y = slid->new_sample.y;
        slid->new_sample.y = cur_sample->y;
        res = 1;
    }
    else
    {
        slid->old_sample.y = slid->new_sample.y;
    }

    if (abs((cur_sample->z - slid->new_sample.z)) > _DYNAMIC_PRECISION)
    {
        slid->old_sample.z = slid->new_sample.z;
        slid->new_sample.z = cur_sample->z;
        res = 1;
    }
    else
    {
        slid->old_sample.z = slid->new_sample.z;
    }

    return res;
}

/*******************************************************************
  @brief uint8_t is_most_active(peak_value_t *peak)
  @param
				 peak_value_t *peak
  @return
				 uint8_t
*******************************************************************/
uint8_t is_most_active(peak_value_t *peak)
{
    uint8_t res = MOST_ACTIVE_NULL;
    int16_t x_change = abs((peak->oldmax.x - peak->oldmin.x));
    int16_t y_change = abs((peak->oldmax.y - peak->oldmin.y));
    int16_t z_change = abs((peak->oldmax.z - peak->oldmin.z));

    if (x_change > y_change && x_change > z_change && x_change >= _ACTIVE_PRECISION)
    {
        res = MOST_ACTIVE_X;
    }
    else if (y_change > x_change && y_change > z_change && y_change >= _ACTIVE_PRECISION)
    {
        res = MOST_ACTIVE_Y;
    }
    else if (z_change > x_change && z_change > y_change && z_change >= _ACTIVE_PRECISION)
    {
        res = MOST_ACTIVE_Z;
    }

    return res;
}

/*******************************************************************
  @brief void detect_step(peak_value_t *peak, slid_reg_t *slid, axis_info_t *cur_sample)
  @param
				 peak_value_t *peak
				 slid_reg_t *slid
				 axis_info_t *cur_sample
  @return
				 void
*******************************************************************/
void detect_step(peak_value_t *peak, slid_reg_t *slid, valid_step_filter_t *vsf)
{
    uint8_t res = is_most_active(peak);
    rfid_printf(" res = %d\n", res);

    switch (res)
    {
        case MOST_ACTIVE_NULL:
        {
            //fix
            break;

        }

        case MOST_ACTIVE_X:
        {
            int16_t threshold_x = (peak->oldmax.x + peak->oldmin.x) / 2;

            if (slid->old_sample.x > threshold_x && slid->new_sample.x < threshold_x)
            {
                rfid_printf("X vsf->sample_num = %d\n", vsf->sample_num);

                if (vsf->sample_num >= 10 && vsf->sample_num <= 100)
                {
                    vsf->valid_step_num++;
                    rfid_printf("X vsf->valid_step_num = %d\n", vsf->valid_step_num);

                    if( vsf->valid_step_num == 4)
                    {
                        step.stepNum = step.stepNum + 4;
                        rfid_printf("X step.stepNum = %d\n", step.stepNum);
                    }
										else if(vsf->valid_step_num > 4)
										{
												step.stepNum++;
                        rfid_printf("X step.stepNum = %d\n", step.stepNum);
										}
                }
                else
                {
                    vsf->valid_step_num = 0;
                }

                vsf->sample_num = 0;
            }

            break;
        }

        case MOST_ACTIVE_Y:
        {
            int16_t threshold_y = (peak->oldmax.y + peak->oldmin.y) / 2;

            if (slid->old_sample.y > threshold_y && slid->new_sample.y < threshold_y)
            {
                rfid_printf("Y vsf->sample_num = %d\n", vsf->sample_num);

                if (vsf->sample_num >= 10 && vsf->sample_num <= 100)
                {
                    vsf->valid_step_num++;
                    rfid_printf("Y vsf->valid_step_num = %d\n", vsf->valid_step_num);

                    if( vsf->valid_step_num == 4)
                    {
                        step.stepNum = step.stepNum + 4;
                        rfid_printf("Y step.stepNum = %d\n", step.stepNum);
                    }
										else if(vsf->valid_step_num > 4)
										{
												step.stepNum++;
                        rfid_printf("Y step.stepNum = %d\n", step.stepNum);
										}
                }
                else
                {
                    vsf->valid_step_num = 0;
                }

                vsf->sample_num = 0;
            }

            break;
        }

        case MOST_ACTIVE_Z:
        {
            int16_t threshold_z = (peak->oldmax.z + peak->oldmin.z) / 2;

            if (slid->old_sample.z > threshold_z && slid->new_sample.z < threshold_z)
            {
                rfid_printf("Z vsf->sample_num = %d\n", vsf->sample_num);

                if (vsf->sample_num >= 10 && vsf->sample_num <= 100)
                {
                    vsf->valid_step_num++;
                    rfid_printf("Z vsf->valid_step_num = %d\n", vsf->valid_step_num);

                    if( vsf->valid_step_num == 4)
                    {
                        step.stepNum = step.stepNum + 4;
                        rfid_printf("Z step.stepNum = %d\n", step.stepNum);
                    }
										else if(vsf->valid_step_num > 4)
										{
												step.stepNum++;
                        rfid_printf("Z step.stepNum = %d\n", step.stepNum);
										}
                }
                else
                {
                    vsf->valid_step_num = 0;
                }

                vsf->sample_num = 0;
            }

            break;
        }

        default:
            break;
    }
}

