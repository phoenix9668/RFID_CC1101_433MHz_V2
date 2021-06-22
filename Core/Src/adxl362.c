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

#include "adxl362.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"

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
    for (RegisterIndex=0; RegisterIndex<NumberofRegisters; RegisterIndex++)
    {
        SendTemp[2+RegisterIndex] = *(RegisterData + RegisterIndex);
    }
    SpiFunction(SPI2, SendTemp, ReceiveTemp, (2+NumberofRegisters), 0);
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
  @brief void ADXL362_Init(void)
         initial and configure ADXL362
  @param
				 none
  @return
         none
*******************************************************************/
void ADXL362_Init(void)
{
		ADXL362RegisterWrite(XL362_SOFT_RESET,0x52);   						// software reset
		HAL_Delay(1000);
		ADXL362RegisterWrite(XL362_THRESH_ACT_L,0x5E);						//set active threshold equip 350mg
		ADXL362RegisterWrite(XL362_THRESH_ACT_H,0x01);
		ADXL362RegisterWrite(XL362_THRESH_INACT_L,0x96);					//set inactive threshold equip 150mg
		ADXL362RegisterWrite(XL362_THRESH_INACT_H,0x00);
		ADXL362RegisterWrite(XL362_TIME_INACT_L,0x01);						//set inactive time equip 1/12.5s
		ADXL362RegisterWrite(XL362_TIME_INACT_H,0x00);
		ADXL362RegisterWrite(XL362_ACT_INACT_CTL,0x3F);						//configure loop mode,enable active and inactive
		ADXL362RegisterWrite(XL362_INTMAP1,0x40);									//configure awake map INT1
		ADXL362RegisterWrite(XL362_INTMAP2,0x00);									//configure awake map INT2
		ADXL362RegisterWrite(XL362_FIFO_CONTROL,0x00);						//select fifo stream mode//0x0E
		ADXL362RegisterWrite(XL362_FIFO_SAMPLES,0x00);						//select fifo sample number//0xFF
		ADXL362RegisterWrite(XL362_FILTER_CTL,0x10);             	//select 2g range,ODR:12.5Hz
    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL,0x0A);              	//select measurement mode
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
void ADXL362_ReInit(uint8_t thresh_act_h, uint8_t thresh_act_l, uint8_t thresh_inact_h, uint8_t thresh_inact_l, uint8_t time_inact_h, uint8_t time_inact_l, uint8_t filter_ctl)
{
		ADXL362RegisterWrite(XL362_SOFT_RESET,0x52);   						// software reset
		HAL_Delay(1000);
		ADXL362RegisterWrite(XL362_THRESH_ACT_L,thresh_act_l);						//set active threshold equip 350mg
		ADXL362RegisterWrite(XL362_THRESH_ACT_H,thresh_act_h);
		ADXL362RegisterWrite(XL362_THRESH_INACT_L,thresh_inact_l);					//set inactive threshold equip 150mg
		ADXL362RegisterWrite(XL362_THRESH_INACT_H,thresh_inact_h);
		ADXL362RegisterWrite(XL362_TIME_INACT_L,time_inact_l);						//set inactive time equip 1/12.5s
		ADXL362RegisterWrite(XL362_TIME_INACT_H,time_inact_h);
		ADXL362RegisterWrite(XL362_ACT_INACT_CTL,0x3F);						//configure loop mode,enable active and inactive
		ADXL362RegisterWrite(XL362_INTMAP1,0x40);									//configure awake map INT1
		ADXL362RegisterWrite(XL362_INTMAP2,0x00);									//configure awake map INT2
		ADXL362RegisterWrite(XL362_FIFO_CONTROL,0x00);						//select fifo stream mode//0x0E
		ADXL362RegisterWrite(XL362_FIFO_SAMPLES,0x00);						//select fifo sample number//0xFF
		ADXL362RegisterWrite(XL362_FILTER_CTL,filter_ctl);             	//select 2g range,ODR:12.5Hz
    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL,0x0A);              	//select measurement mode
		HAL_Delay(200);
}
