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
uint8_t ingestion_cnt = 0;
uint8_t rest_cnt = 0;
uint8_t deta_a_cnt = 0;
int16_t eighteen_average = 0;
uint16_t sum_eighteen_average = 0;

uint8_t fifo[_FIFO_LEN];
step_t step;

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

    ADXL362RegisterWrite(XL362_SOFT_RESET, 0x52);   						// software reset
    HAL_Delay(1000);

    #if (_Original_Data_Algorithm == 0)

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

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06);						//set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
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

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x51);             	//select 4g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x51)
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

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06);						//set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
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

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x51);             	//select 4g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x51)
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
    // 5.Dynamic Threshold

    CC1101Send3AxisHandler();

    #else
    // 1.init acceler array

    // 2.To 16-bit complement
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) * 3; i++)
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

    // 3.Average Calc

    // 4.Difference Derivation And Threshold Judge
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
    {
        diff_three_axis_info[1] = diff_three_axis_info[0];
        diff_three_axis_info[0] = three_axis_info[i];

        three_axis_average_info.y += three_axis_info[i].y;
        three_axis_info[i].y = diff_three_axis_info[0].y - diff_three_axis_info[1].y;
        sum_info.y += abs(three_axis_info[i].y);
        rfid_printf("samples[%d] :%hd\n", i, three_axis_info[i].y);

        if (abs(three_axis_info[i].y) <= 10)
            threshold_judge.low ++;
        else if (abs(three_axis_info[i].y) <= 100)
            threshold_judge.normal ++;
        else if (abs(three_axis_info[i].y) <= 200)
            threshold_judge.abovenormal ++;
        else
            threshold_judge.high ++;

        if (((i + 1) / 25) != 0 && ((i + 1) % 25) == 0)
        {
            rfid_printf("threshold_judge.low = %d\n", threshold_judge.low);
            rfid_printf("threshold_judge.normal = %d\n", threshold_judge.normal);
            rfid_printf("threshold_judge.abovenormal = %d\n", threshold_judge.abovenormal);
            rfid_printf("threshold_judge.high = %d\n", threshold_judge.high);

            three_axis_average_info.y = three_axis_average_info.y / 25;
            rfid_printf("average_info.y = %d\n", three_axis_average_info.y);
            rfid_printf("sum_info.y = %d\n", sum_info.y);

            if (threshold_judge.low >= 24)
                action = 1;																											// rest
            else if ((threshold_judge.normal + threshold_judge.abovenormal) > 11 && threshold_judge.high == 0 && three_axis_average_info.y >= 200)
                action = 2;																											// ingestion
            else if (three_axis_average_info.y > -200 && three_axis_average_info.y < 100 && sum_info.y > 400)
                action = 3;																											// movement
            else if (threshold_judge.high > 0 && three_axis_average_info.y <= -200)
                action = 4;																											// climb
            else
                action = 6;																											//other

//            action_classify_array[((i + 1) / 25) - 1] = action;

            rfid_printf("action = %d\n", action);
            rfid_printf("action_classify.rest = %d\n", action_classify.rest);
            rfid_printf("action_classify.ingestion = %d\n", action_classify.ingestion);
            rfid_printf("action_classify.movement = %d\n", action_classify.movement);
            rfid_printf("action_classify.climb = %d\n", action_classify.climb);
            rfid_printf("action_classify.other = %d\n", action_classify.other);

            // circular storage
            memory_array[memory_index][0] = action;
            memory_array[memory_index][1] = three_axis_average_info.y;
            memory_array[memory_index][2] = sum_info.y;

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

            if (memory_index >= 17)
                memory_index = 0;
            else
                memory_index += 1;

            // eliminate redundant climb
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if(memory_array[i][0] == 4)
                {
                    for (uint8_t j = (i + 1); j < _MEM_ROWS; j++)
                    {
                        if (memory_array[j][0] == 4)
                            memory_array[j][0] = 3;
                    }

                    rfid_printf("memory_array2:\n");

                    // print memory_array
                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    {
                        for (uint8_t j = 0; j < _MEM_COLS; j++)
                        {
                            rfid_printf("%d ", memory_array[i][j]);
                        }

                        rfid_printf("\n");
                    }
                }

            // if 2 >= 2 ==> 3 to 6
            ingestion_cnt = 0;

            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if(memory_array[i][0] == 2)
                    ingestion_cnt += 1;

            if (ingestion_cnt >= 2)
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    if(memory_array[i][0] == 3)
                        memory_array[i][0] = 6;

            // ruminate logic
            rest_cnt = 0;
            deta_a_cnt = 0;
            eighteen_average = 0;
            sum_eighteen_average = 0;

            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if(memory_array[i][0] == 1)
                    rest_cnt += 1;

            if (rest_cnt <= 4)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    if(memory_array[i][2] > 130 && memory_array[i][2] < 700)
                        deta_a_cnt += 1;

                if (deta_a_cnt >= 14)
                {
                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        eighteen_average += memory_array[i][1];

                    eighteen_average = eighteen_average / _MEM_ROWS;

                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        sum_eighteen_average += abs(eighteen_average - memory_array[i][1]);

                    if (sum_eighteen_average <= 400 && abs(eighteen_average) < 150)
                        for (uint8_t i = 0; i < _MEM_ROWS; i++)
                            memory_array[i][0] = 5;
                }

//                for (uint8_t i = 0; i < _MEM_ROWS; i++)
//                    if(memory_array[i][0] == 5)
//                        action_classify.ruminate ++;
            }

            // delay 18s output
            if (memory_index == 17)
                memory_index_o = 0;
            else
                memory_index_o = memory_index + 1;

            action_classify_array[((i + 1) / 25) - 1] = memory_array[memory_index_o][0];

            if (memory_array[memory_index_o][0] == 1) // rest
                action_classify.rest ++;
            else if (memory_array[memory_index_o][0] == 2) // ingestion
                action_classify.ingestion ++;
            else if (memory_array[memory_index_o][0] == 3) // movement
                action_classify.movement ++;
            else if (memory_array[memory_index_o][0] == 4) // climb
                action_classify.climb ++;
            else if (memory_array[memory_index_o][0] == 5) // ruminate
                action_classify.ruminate ++;
            else //other
                action_classify.other ++;

            memset(&threshold_judge, 0, sizeof(threshold_judge));
            memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
            memset(&sum_info, 0, sizeof(sum_info));
        }
    }

    for(uint8_t i = 0; i < sizeof(action_classify_array); i++)
    {
        rfid_printf("action_classify_array[%d] = %d\n", i, action_classify_array[i]);
    }

//    CC1101Send3AxisHandler();

//    ADXL362RegisterRead(XL362_STATUS);
//    rfid_printf("XL362_STATUS: %x\n", ADXL362RegisterRead(XL362_STATUS));

    #endif
}

