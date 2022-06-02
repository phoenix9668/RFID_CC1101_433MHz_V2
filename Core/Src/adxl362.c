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

step_t step;
uint8_t fifo[_FIFO_LEN];
//static short int xAxis[_AXIS_LEN];
short int yAxis[_AXIS_LEN];
short int yStepAverage[_AXIS_LEN - 2];
short int yStepFilter[_AXIS_LEN - 2];
short int yIngestionAverage[_AXIS_LEN / 25 + 1];
//static short int zAxis[_AXIS_LEN];
//static uint16_t threeAxisUnity[_AXIS_LEN];
//uint16_t threeAxisAverage;

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
  @brief void ADXL362FifoProcess(void)
         Multibyte read from ADXL362 FIFO
  @param
				 none
  @return
				 none
*******************************************************************/
void ADXL362FifoProcess(void)
{
    bool thresholdMark = false;
    #if (_Original_Data_Algorithm == 0)
    /* @brief integration of accelerations
     * @param input fifo
     * @param output acceler
     */

    // 1.init acceler array
//    memset(xAxis, 0, sizeof(xAxis));
    memset(yAxis, 0, sizeof(yAxis));
    memset(yStepAverage, 0, sizeof(yStepAverage));
    memset(yStepFilter, 0, sizeof(yStepFilter));
    memset(yIngestionAverage, 0, sizeof(yIngestionAverage));
//    memset(zAxis, 0, sizeof(zAxis));
//    memset(threeAxisUnity, 0, sizeof(threeAxisUnity));

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
                yAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                yAxis[i / 3] = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

//            rfid_printf("samples[%d] :%hd\n", i / 3, yAxis[i / 3]);
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
    for (uint8_t i = 0; i < ARRAY_LEN(yStepAverage); i++)
    {
        yStepAverage[i] = (yAxis[i] + yAxis[i + 1] + yAxis[i + 2]) / 3;
//        rfid_printf("yStepAverage[%d] :%hd\n", i, yStepAverage[i]);
    }

    // 5.Calculate step filter values array
    for (uint8_t i = 0; i < ARRAY_LEN(yStepFilter); i++)
    {
        yStepFilter[i] =  yStepAverage[i] - yAxis[i + 2];
//        rfid_printf("yStepFilter[%d] :%hd,%hx\n", i, yStepFilter[i], yStepFilter[i]);
    }

    // 6.Calculate the stepNum
    for (uint8_t i = 0; i < ARRAY_LEN(yStepFilter); i++)
    {
        if(yStepFilter[i] >= 100 && yStepFilter[i] <= 500)
        {
            if(thresholdMark == false)
            {
                step.stepNum++;
                thresholdMark = true;
                rfid_printf("stepNum = %d\n", step.stepNum);
            }
        }
        else if(yStepFilter[i] < 100)
        {
            thresholdMark = false;
        }
    }

    // 7.Calculate ingestion average values array
    for (uint8_t i = 0; i < ARRAY_LEN(yIngestionAverage); i++)
    {
        for (uint8_t j = 0; j < 25; j++)
        {
            if((i * 25 + j) < 170)
            {
                yIngestionAverage[i] += yAxis[i * 25 + j];
            }
        }

        if(i == (ARRAY_LEN(yIngestionAverage) - 1))
            yIngestionAverage[i] = yIngestionAverage[i] / 20;
        else
            yIngestionAverage[i] = yIngestionAverage[i] / 25;

//        rfid_printf("yIngestionAverage[%d] :%hd\n", i, yIngestionAverage[i]);
    }

    // 8.Calculate the ingestionNum
    thresholdMark = false;

    for (uint8_t i = 0; i < ARRAY_LEN(yIngestionAverage) - 2; i++)
    {
        if(abs(yIngestionAverage[i + 2] - yIngestionAverage[i]) >= 300)
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
    ADXL362RegisterWrite(XL362_SOFT_RESET, 0x52);   						// software reset
    HAL_Delay(1000);

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

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06);						//set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
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

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x06);						//set inactive time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
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

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x11);             	//select 2g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x11)
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

    rfid_printf("|**************************************|\n");

    HAL_Delay(200);
}

/*******************************************************************
  @brief void ADXL362_ReInit(void)
         reinitial and configure ADXL362
  @param
				 none
  @return
         none
*******************************************************************/
void ADXL362_ReInit(uint8_t thresh_act_h, uint8_t thresh_act_l, uint8_t time_act, uint8_t thresh_inact_h, uint8_t thresh_inact_l, uint8_t time_inact_h, uint8_t time_inact_l, uint8_t filter_ctl)
{
    ADXL362RegisterWrite(XL362_SOFT_RESET, 0x52);   						// software reset
    HAL_Delay(1000);
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, thresh_act_l);						//set active threshold equip 350mg
    ADXL362RegisterWrite(XL362_THRESH_ACT_H, thresh_act_h);
    ADXL362RegisterWrite(XL362_TIME_ACT, time_act);						//set active time equip 45/200s
    ADXL362RegisterWrite(XL362_THRESH_INACT_L, thresh_inact_l);					//set inactive threshold equip 150mg
    ADXL362RegisterWrite(XL362_THRESH_INACT_H, thresh_inact_h);
    ADXL362RegisterWrite(XL362_TIME_INACT_L, time_inact_l);						//set inactive time equip 1/12.5s
    ADXL362RegisterWrite(XL362_TIME_INACT_H, time_inact_h);
    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x07);						//configure default mode,enable active and inactive
    ADXL362RegisterWrite(XL362_INTMAP1, 0x10);									//configure awake map INT1
    ADXL362RegisterWrite(XL362_INTMAP2, 0x20);									//configure awake map INT2
    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x0E);						//select fifo stream mode//0x0E
    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0xFF);						//select fifo sample number//0xFF
    ADXL362RegisterWrite(XL362_FILTER_CTL, filter_ctl);             	//select 2g range,ODR:12.5Hz
    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02);              	//select measurement mode
    HAL_Delay(200);
}
