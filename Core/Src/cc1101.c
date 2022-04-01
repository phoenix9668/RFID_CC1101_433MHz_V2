/**
  ******************************************************************************
  * @file    cc1101.h
  * @author  phoenix
  * @version V1.0.0
  * @date    20-October-2017
  * @brief   This module contains the low level operations for CC1101
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 
#include "cc1101.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "crc.h"
#include "adxl362.h"

/* Array that will be filled with random bytes */
extern uint8_t RandomString[32];
//10, 7, 5, 0, -5, -10, -15, -20, dbm output power
uint8_t PaTabel[]={0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
cc1101_t cc1101;
uint16_t resetCnt = 0;
__IO ITStatus txFiFoUnFlow;
__IO ITStatus rxCatch = RESET;

// Sync word qualifier mode = 30/32 sync word bits detected 
// CRC autoflush = false 
// Channel spacing = 199.951172 
// Data format = Normal mode 
// Data rate = 76.767
// RX filter BW = 232.142857
// PA ramping = false 
// Preamble count = 4 
// Whitening = ture 
// Address config = Address check and 0 (0x00) and 255 (0xFF) broadcast
// Carrier frequency = 432.999817
// Device address = 5 
// TX power = 10 
// Manchester enable = ture 
// CRC enable = true 
// Deviation = 31.738281
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Packet length = 255 
// Modulation format = GFSK
// Base frequency = 432.999817
// Modulated = true 
// Channel number = 0 

static const uint8_t CC1101InitData[47][2]= 
{
	{CC1101_IOCFG2,				0x29},
	{CC1101_IOCFG1,				0x2E},
	{CC1101_IOCFG0,				0x46},
	{CC1101_FIFOTHR,			0x4E},
	{CC1101_SYNC1,				0xD3},
	{CC1101_SYNC0,				0x91},
	{CC1101_PKTLEN,				0xFF},
	{CC1101_PKTCTRL1,			0x07},
	{CC1101_PKTCTRL0,			0x45},
	{CC1101_ADDR,					0x00},
	{CC1101_CHANNR,				0x00},
	{CC1101_FSCTRL1,			0x0B},
	{CC1101_FSCTRL0,			0x00},
	{CC1101_FREQ2,				0x10},
	{CC1101_FREQ1,				0xA7},
	{CC1101_FREQ0, 				0x62},
	{CC1101_MDMCFG4,			0x7B},
	{CC1101_MDMCFG3,			0x83},
	{CC1101_MDMCFG2,			0x9B},
	{CC1101_MDMCFG1,			0x22},
	{CC1101_MDMCFG0,			0xF8},
	{CC1101_DEVIATN,			0x42},
	{CC1101_MCSM2,				0x03},
	{CC1101_MCSM1,				0x30},
	{CC1101_MCSM0,				0x18},
	{CC1101_FOCCFG,				0x1D},
	{CC1101_BSCFG,				0x1C},
	{CC1101_AGCCTRL2,			0xC7},
	{CC1101_AGCCTRL1,			0x00},
	{CC1101_AGCCTRL0,			0xB2},
	{CC1101_WOREVT1,			0x8C},
	{CC1101_WOREVT0,			0xA0},
	{CC1101_WORCTRL,			0x78},
	{CC1101_FREND1,				0xB6},
	{CC1101_FREND0,				0x10},
	{CC1101_FSCAL3,				0xEA},
	{CC1101_FSCAL2,				0x2A},
	{CC1101_FSCAL1,				0x00},
	{CC1101_FSCAL0,				0x1F},
	{CC1101_RCCTRL1,			0x41},
	{CC1101_RCCTRL0,			0x00},
	{CC1101_FSTEST,				0x59},
	{CC1101_PTEST,				0x7F},
	{CC1101_AGCTEST,			0x3F},
	{CC1101_TEST2,				0x81},
	{CC1101_TEST1,				0x35},
	{CC1101_TEST0,				0x09},

};

// Sync word qualifier mode = 30/32 sync word bits detected 
// CRC autoflush = false 
// Channel spacing = 51.910400 
// Data format = Normal mode 
// Data rate = 4.80223
// RX filter BW = 105.468750
// PA ramping = false 
// Preamble count = 4 
// Whitening = ture 
// Address config = Address check and 0 (0x00) and 255 (0xFF) broadcast
// Carrier frequency = 868.999878
// Device address = 5
// TX power = 20
// Manchester enable = ture
// CRC enable = true 
// Deviation = 24.719238
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Packet length = 255 
// Modulation format = GFSK
// Base frequency = 868.999878
// Modulated = true 
// Channel number = 0 

//static const uint8_t CC1101InitData[47][2]= 
//{
//	{CC1101_IOCFG2,				0x29},
//	{CC1101_IOCFG1,				0x2E},
//	{CC1101_IOCFG0,				0x06},
//	{CC1101_FIFOTHR,			0x4C},
//	{CC1101_SYNC1,				0xD3},
//	{CC1101_SYNC0,				0x91},
//	{CC1101_PKTLEN,				0xFF},
//	{CC1101_PKTCTRL1,			0x07},
//	{CC1101_PKTCTRL0,			0x45},
//	{CC1101_ADDR,					0x00},
//	{CC1101_CHANNR,				0x00},
//	{CC1101_FSCTRL1,			0x06},
//	{CC1101_FSCTRL0,			0x00},
//  {CC1101_FREQ2,        0x20},
//  {CC1101_FREQ1,        0x2F},
//  {CC1101_FREQ0,        0x68},
//	{CC1101_MDMCFG4,      0xC7},
//  {CC1101_MDMCFG3,      0x75},
//	{CC1101_MDMCFG2,			0x1B},
//	{CC1101_MDMCFG1,			0x20},
//	{CC1101_MDMCFG0,			0xF8},
//	{CC1101_DEVIATN,			0x37},
//	{CC1101_MCSM2,				0x03},
//	{CC1101_MCSM1,				0x30},
//	{CC1101_MCSM0,				0x18},
//	{CC1101_FOCCFG,				0x16},
//	{CC1101_BSCFG,				0x6C},
//	{CC1101_AGCCTRL2,			0x43},
//	{CC1101_AGCCTRL1,			0x40},
//	{CC1101_AGCCTRL0,			0x91},
//	{CC1101_WOREVT1,			0x8C},
//	{CC1101_WOREVT0,			0xA0},
//	{CC1101_WORCTRL,			0x78},
//	{CC1101_FREND1,				0xB6},
//	{CC1101_FREND0,				0x10},
//	{CC1101_FSCAL3,       0xE9},
//  {CC1101_FSCAL2,       0x2A},
//  {CC1101_FSCAL1,       0x00},
//  {CC1101_FSCAL0,       0x1F},
//	{CC1101_RCCTRL1,			0x41},
//	{CC1101_RCCTRL0,			0x00},
//	{CC1101_FSTEST,				0x59},
//	{CC1101_PTEST,				0x7F},
//	{CC1101_AGCTEST,			0x3F},
//	{CC1101_TEST2,				0x81},
//	{CC1101_TEST1,				0x35},
//	{CC1101_TEST0,				0x09},

//};

/*
================================================================================
Function : CC1101WORInit()
    Initialize the WOR function of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101WORInit(void)
{
    CC1101WriteReg(CC1101_MCSM0, 0x18);
    CC1101WriteReg(CC1101_WORCTRL, 0x78); //EVENT1 = 7,RC_CAL = 1,WOR_RES = 0
																					//tEvent1 = 750/fXOSC * 7 = 48*750/(26*10^6) = 1.385ms
//    CC1101WriteReg(CC1101_MCSM2, 0x00);		//RX_TIME = 0,Duty cycle = 12.5%
																					//tRXtimeout = tEvent0 * Duty cycle = 129.75ms
//		CC1101WriteReg(CC1101_MCSM2, 0x01);		//RX_TIME = 1,Duty cycle = 6.25%
																					//tRXtimeout = tEvent0 * Duty cycle = 64.875ms
//		CC1101WriteReg(CC1101_MCSM2, 0x02);		//RX_TIME = 2,Duty cycle = 3.125%
																					//tRXtimeout = tEvent0 * Duty cycle = 32.4375ms
		CC1101WriteReg(CC1101_MCSM2, 0x03);		//RX_TIME = 3,Duty cycle = 1.563%
																					//tRXtimeout = tEvent0 * Duty cycle = 16.21875ms
//		CC1101WriteReg(CC1101_MCSM2, 0x04);		//RX_TIME = 4,Duty cycle = 0.781%
																					//tRXtimeout = tEvent0 * Duty cycle = 8.109375ms
//		CC1101WriteReg(CC1101_MCSM2, 0x05);		//RX_TIME = 5,Duty cycle = 0.391%
																					//tRXtimeout = tEvent0 * Duty cycle = 4.0546875ms
    CC1101WriteReg(CC1101_WOREVT1, 0x8C);
		CC1101WriteReg(CC1101_WOREVT0, 0xA0);	//EVENT0 = 0d36000
																					//tEvent0 = 750/fXOSC * EVENT0 * 2^(5*WOR_RES)
																					//tEvent0 = 750/(26*10^6) * 36000 * 2^0 = 1.038s
}

//������tEvent0 = 60.495s��tRXtimeout = 1.18s�����RX״̬�µ���17.72mA��SLEEP״̬�µ���0.3uA
//void CC1101WORInit(void)
//{
//    CC1101WriteReg(CC1101_MCSM0, 0x18);
//    CC1101WriteReg(CC1101_WORCTRL, 0x79); //EVENT1 = 7,RC_CAL = 1,WOR_RES = 1
//																					//tEvent1 = 750/fXOSC * 7 = 48*750/(26*10^6) = 1.385ms
//    CC1101WriteReg(CC1101_MCSM2, 0x00);		//RX_TIME = 0,Duty cycle = 1.95%
//																					//tRXtimeout = tEvent0 * Duty cycle = 1.18s
//		CC1101WriteReg(CC1101_WOREVT1, 0xFF);
//		CC1101WriteReg(CC1101_WOREVT0, 0xFF);	//EVENT0 = 0d65535
//																					//tEvent0 = 750/fXOSC * EVENT0 * 2^(5*WOR_RES)
//																					//tEvent0 = 750/(26*10^6) * 65535 * 2^5 = 60.495s
//}
/*
================================================================================
Function : CC1101SetWORMode()
    set the WOR function of CC1101,include configure IOCFG0,IOCFG2
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101SetWORMode(void)
{
		CC1101WriteReg(CC1101_IOCFG0, 0x46);
		CC1101WriteReg(CC1101_IOCFG2, 0x00);	//rx fifo threshold
//		CC1101WriteReg(CC1101_IOCFG2, 0x64);	//Event0 monitor
		CC1101WriteCmd(CC1101_SFRX);
		CC1101WriteCmd(CC1101_SWORRST);
		CC1101WriteCmd(CC1101_SWOR);
}
/*
================================================================================
Function : CC1101ReadReg()
    read a byte from the specified register
INPUT    : addr, The address of the register
OUTPUT   : the byte read from the rigister
================================================================================
*/
uint8_t CC1101ReadReg(uint8_t addr)
{
    uint8_t d;
    CC1101_CSN_LOW();
    SPI_ExchangeByte(SPI1, addr | READ_SINGLE);
    d = SPI_ExchangeByte(SPI1, 0xFF);
    CC1101_CSN_HIGH();
    return d;
}
/*
================================================================================
Function : CC1101ReadMultiReg()
    Read some bytes from the rigisters continously
INPUT    : addr, The address of the register
           buff, The buffer stores the data
           size, How many bytes should be read
OUTPUT   : None
================================================================================
*/
void CC1101ReadMultiReg(uint8_t addr, uint8_t *buff, uint8_t size)
{
    CC1101_CSN_LOW();
    SPI_ExchangeByte(SPI1, addr | READ_BURST);
    for(uint8_t i=0; i<size; i++)
    {
        for(uint8_t j=0; j<20; j++);
        *(buff+i)=SPI_ExchangeByte(SPI1, 0xFF);
    }
    CC1101_CSN_HIGH();
}
/*
================================================================================
Function : CC1101ReadStatus()
    Read a status register
INPUT    : addr, The address of the register
OUTPUT   : the value read from the status register
================================================================================
*/
uint8_t CC1101ReadStatus(uint8_t addr)
{
    uint8_t d;
    CC1101_CSN_LOW();
    SPI_ExchangeByte(SPI1, addr | READ_BURST);
    d = SPI_ExchangeByte(SPI1, 0xFF);
    CC1101_CSN_HIGH();
    return d;
}
/*
================================================================================
Function : CC1101SetTRMode()
    Set the device as TX mode or RX mode
INPUT    : mode selection
OUTPUT   : None
================================================================================
*/
void CC1101SetTRMode(TRMODE mode)
{
    if(mode == TX_MODE)
    {
		    RX_EN_LOW();
        TX_EN_HIGH();
        CC1101WriteReg(CC1101_IOCFG0, 0x46);
				CC1101WriteReg(CC1101_IOCFG2, 0x02);	//tx fifo threshold
        CC1101WriteCmd(CC1101_STX);
    }
    else if(mode == RX_MODE)
    {
		    TX_EN_LOW();
			  RX_EN_HIGH();
        CC1101WriteReg(CC1101_IOCFG0, 0x46);
				CC1101WriteReg(CC1101_IOCFG2, 0x40);	//rx fifo threshold
        CC1101WriteCmd(CC1101_SRX);
    }
}
/*
================================================================================
Function : CC1101WriteReg()
    Write a byte to the specified register
INPUT    : addr, The address of the register
           value, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteReg(uint8_t addr, uint8_t value)
{
    CC1101_CSN_LOW();
    SPI_ExchangeByte(SPI1, addr);
    SPI_ExchangeByte(SPI1, value);
    CC1101_CSN_HIGH();
}
/*
================================================================================
Function : CC1101WriteMultiReg()
    Write some bytes to the specified register
INPUT    : addr, The address of the register
           buff, a buffer stores the values
           size, How many byte should be written
OUTPUT   : None
================================================================================
*/
void CC1101WriteMultiReg(uint8_t addr, uint8_t *buff, uint8_t size)
{
    CC1101_CSN_LOW();
    SPI_ExchangeByte(SPI1, addr | WRITE_BURST);
    for(uint8_t i=0; i<size; i++)
    {
        SPI_ExchangeByte(SPI1, *(buff+i));
    }
    CC1101_CSN_HIGH();
}
/*
================================================================================
Function : CC1101WriteCmd()
    Write a command byte to the device
INPUT    : command, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteCmd(uint8_t command)
{
    CC1101_CSN_LOW();
    SPI_ExchangeByte(SPI1, command);
    CC1101_CSN_HIGH();
}
/*
================================================================================
Function : CC1101Reset()
    Reset the CC1101 device
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Reset(void)
{
    CC1101_CSN_HIGH();
    CC1101_CSN_LOW();
		HAL_Delay(1);
    CC1101_CSN_HIGH();
    HAL_Delay(1);//>40us
		CC1101_CSN_LOW();
		HAL_Delay(1);
		while(CC1101_MISO_READ()){}
    CC1101WriteCmd(CC1101_SRES);
    HAL_Delay(1);
}
/*
================================================================================
Function : CC1101SetIdle()
    Set the CC1101 into IDLE mode
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101SetIdle(void)
{
    CC1101WriteCmd(CC1101_SIDLE);
}
/*
================================================================================
Function : CC1101ClrTXBuff()
    Flush the TX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrTXBuff(void)
{
    CC1101SetIdle();//MUST BE IDLE MODE
    CC1101WriteCmd(CC1101_SFTX);
}
/*
================================================================================
Function : CC1101ClrRXBuff()
    Flush the RX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrRXBuff(void)
{
    CC1101SetIdle();//MUST BE IDLE MODE
    CC1101WriteCmd(CC1101_SFRX);
}
/*
================================================================================
Function : CC1101SendPacket()
    Send a packet
INPUT    : txbuffer, The buffer stores data to be sent
           size, How many bytes should be sent
           mode, Broadcast or address check packet
OUTPUT   : None
================================================================================
*/
void CC1101SendPacket(uint8_t *txbuffer, uint8_t size, TX_DATA_MODE mode)
{
    uint8_t address;

		rxCatch = RESET;
		txFiFoUnFlow = RESET;

    if(mode == BROADCAST)             {address=0;}
    else if(mode == ADDRESS_CHECK)    {address=CC1101ReadReg(CC1101_ADDR);}
    
    CC1101ClrTXBuff();
    if((CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03) != 0){
        CC1101WriteReg(CC1101_TXFIFO, size+1);
        CC1101WriteReg(CC1101_TXFIFO, address);
    }
    else{
        CC1101WriteReg(CC1101_TXFIFO, size);
    }

		if(size <= 60){
			CC1101WriteMultiReg(CC1101_TXFIFO, txbuffer, size);
			CC1101SetTRMode(TX_MODE);
			while(rxCatch != SET){}
			rxCatch = RESET;
		}
		else{
			CC1101WriteMultiReg(CC1101_TXFIFO, txbuffer, 60);
			CC1101SetTRMode(TX_MODE);

			for(uint8_t i=0; i<(size/60); i++){
				if((i+1) == (size/60)){
					while(txFiFoUnFlow != SET){}
					CC1101WriteMultiReg(CC1101_TXFIFO, (txbuffer+(i+1)*60), (size-(i+1)*60));
				}else{
					while(txFiFoUnFlow != SET){}
					txFiFoUnFlow = RESET;
					CC1101WriteMultiReg(CC1101_TXFIFO, (txbuffer+(i+1)*60), 60);
				}
			}
		}
    rxCatch = RESET;
		while(rxCatch != SET){}
		rxCatch = RESET;
    //i = CC1101ReadStatus( CC1101_TXBYTES );//for test, TX status

    CC1101ClrTXBuff();
}
/*
================================================================================
Function : CC1101GetRXCnt()
    Get received count of CC1101
INPUT    : None
OUTPUT   : How many bytes hae been received
================================================================================
*/
uint8_t CC1101GetRXCnt(void)
{
    return (CC1101ReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO);
}
/*
================================================================================
Function : CC1101SetAddress()
    Set the address and address mode of the CC1101
INPUT    : address, The address byte
           AddressMode, the address check mode
OUTPUT   : None
================================================================================
*/
void CC1101SetAddress(uint8_t address, ADDR_MODE AddressMode)
{
    uint8_t btmp=CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03;
    CC1101WriteReg(CC1101_ADDR, address);
    if(AddressMode == BROAD_ALL)        {}
    else if(AddressMode == BROAD_NO)    {btmp |= 0x01;}
    else if(AddressMode == BROAD_0)     {btmp |= 0x02;}
    else if(AddressMode == BROAD_0AND255)   {btmp |= 0x03;}  
}
/*
================================================================================
Function : CC1101SetSYNC()
    Set the SYNC bytes of the CC1101
INPUT    : sync, 16bit sync 
OUTPUT   : None
================================================================================
*/
void CC1101SetSYNC(uint16_t sync)
{
    CC1101WriteReg(CC1101_SYNC1, 0xFF & (sync>>8));
    CC1101WriteReg(CC1101_SYNC0, 0xFF & sync);
}
/*
================================================================================
Function : CC1101RecPacket()
    Receive a packet
INPUT    : rxBuffer, A buffer store the received data
OUTPUT   : 1:received count, 0:no data
================================================================================
*/
uint8_t CC1101RecPacket(uint8_t *rxBuffer, uint8_t *addr, uint8_t *rssi)
{
    uint8_t status[2];
    uint8_t pktLen;

		if(CC1101GetRXCnt() != 0)
		{
			pktLen=CC1101ReadReg(CC1101_RXFIFO);                    // Read length byte
			if((CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03) != 0)
			{
				*addr = CC1101ReadReg(CC1101_RXFIFO);
			}
			if(pktLen == 0) {return 0;}
			else    {pktLen--;}
			CC1101ReadMultiReg(CC1101_RXFIFO, rxBuffer, pktLen);    // Pull data
			CC1101ReadMultiReg(CC1101_RXFIFO, status, 2);           // Read status bytes
			*rssi = status[0];

			CC1101ClrRXBuff();

			if(status[1] & CRC_OK)  {return pktLen;}
			else    {return 1;}
		}
		else	{return 0;}
}
/*
================================================================================
Function : CC1101Init()
    Initialize the CC1101, User can modify it
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Init(uint8_t addr, uint16_t sync)
{
		NVIC_DisableIRQ(EXTI2_3_IRQn);
    CC1101Reset();

    for(uint8_t i=0; i<47; i++)
    {
        CC1101WriteReg(CC1101InitData[i][0], CC1101InitData[i][1]);
    }
    CC1101SetAddress(addr, BROAD_0AND255);
    CC1101SetSYNC(sync);

    CC1101WriteMultiReg(CC1101_PATABLE, PaTabel, 8);
		NVIC_EnableIRQ(EXTI2_3_IRQn);

		rfid_printf("CC1101_PKTCTRL1 = %d\n",CC1101ReadStatus(CC1101_PARTNUM));//for test, must be 0x00
		rfid_printf("CC1101_VERSION = %d\n",CC1101ReadStatus(CC1101_VERSION));//for test, refer to the datasheet,must be 0x14
}
/*
================================================================================
Function : RFIDInitial()
    Initialize the CC1101, Configure addr&sync&mode
INPUT    : None
OUTPUT   : None
================================================================================
*/
void RFIDInitial(uint8_t addr, uint16_t sync, TRMODE mode)
{
	CC1101Init(addr, sync);
	if(mode == RX_MODE)
	{
	  TX_EN_LOW();
		RX_EN_HIGH();
	  CC1101SetTRMode(RX_MODE);
	}
	else if(mode == TX_MODE)
	{
	  RX_EN_LOW();
		TX_EN_HIGH();
		CC1101SetTRMode(TX_MODE);
	}
	else if(mode == IDLE_MODE)
	{
	  TX_EN_LOW();
		RX_EN_LOW();
	  CC1101SetIdle();
	}
	else if(mode == WOR_Mode)
	{
		CC1101SetIdle();
		CC1101WORInit();
		CC1101SetWORMode();
	}
	rxCatch = RESET;
}
/*
================================================================================
Function : int16_t CC1101ReadRSSI(void)
    Read the RSSI value of the CC1101
INPUT    : RSSI, 8bit 
OUTPUT   : rssi_dBm
================================================================================
*/
int16_t CC1101ReadRSSI(void)
{
	uint8_t rssi_dec;
	int16_t rssi_dBm;
	uint8_t	rssi_offset = 74;
	
	rssi_dec = CC1101ReadStatus(CC1101_RSSI);
	if(rssi_dec >= 128)
		rssi_dBm = (int16_t)((int16_t)(rssi_dec - 256)/2) - rssi_offset;
	else
		rssi_dBm = (rssi_dec/2) - rssi_offset;
	return rssi_dBm;
}
/*
================================================================================
Function : int16_t CC1101CalcRSSI_dBm(uint8_t)
    Calc the RSSI value to RSSI dBm
INPUT    : RSSI, 8bit 
OUTPUT   : rssi_dBm
================================================================================
*/
int16_t CC1101CalcRSSI_dBm(uint8_t rssi_dec)
{
	int16_t rssi_dBm;
	uint8_t	rssi_offset = 74;
	
	if(rssi_dec >= 128)
		rssi_dBm = (int16_t)((int16_t)(rssi_dec - 256)/2) - rssi_offset;
	else
		rssi_dBm = (rssi_dec/2) - rssi_offset;
	return rssi_dBm;
}
/*
================================================================================
Function : uint8_t CC1101RecvHandler(void)
    Receive RF Single
INPUT    : None
OUTPUT   : uint8_t
================================================================================
*/
uint8_t CC1101RecvHandler(void)
{
	if(rxCatch == SET)
		{
			HAL_Delay(4);
			rfid_printf("interrupt occur\n");
			for (uint8_t i=0; i<sizeof(cc1101.recvBuffer); i++)   { cc1101.recvBuffer[i] = 0; } // clear array
			cc1101.length = CC1101RecPacket(cc1101.recvBuffer, &cc1101.addr, &cc1101.rssi);

			cc1101.rssidBm = CC1101CalcRSSI_dBm(cc1101.rssi);
			rfid_printf("RSSI = %ddBm, length = %d, address = %d\n",cc1101.rssidBm,cc1101.length,cc1101.addr);
			for(uint8_t i=0; i<sizeof(cc1101.recvBuffer); i++)
			{
				rfid_printf("%x ",cc1101.recvBuffer[i]);
			}

			/* Reset transmission flag */
			rxCatch = RESET;

			if(cc1101.length == 0)
				{
					rfid_printf("receive error or Address Filtering fail\n");
					return 0x01;
				}
			else
				{
					if(cc1101.recvBuffer[3] == device.deviceCode1 && cc1101.recvBuffer[4] == device.deviceCode2 && cc1101.recvBuffer[5] == device.deviceCode3 
						&& cc1101.recvBuffer[6] == device.deviceCode4 && cc1101.recvBuffer[7] == device.deviceCode5 && cc1101.recvBuffer[8] == device.deviceCode6)
						{
						if(cc1101.recvBuffer[2] == 0xC0)
							{return cc1101.recvBuffer[2];}
						else
							{
								rfid_printf("receive function order error\r\n");
								return 0x03;}
							}
					else
						{
							rfid_printf("receive RFID code error\r\n");
							return 0x02;}
				}
		}
	else	{return 0x00;}
}
/*
================================================================================
Function : void CC1101SendHandler(void)
    Send RF Single
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101SendHandler(void)
{
	#if (_DEBUG == 1)
		LED_GREEN_ON();
	#endif
	
	cc1101.sendBuffer[0] = device.deviceCode1;
	cc1101.sendBuffer[1] = device.deviceCode2;
	cc1101.sendBuffer[2] = device.deviceCode3;
	cc1101.sendBuffer[3] = device.deviceCode4;
	cc1101.sendBuffer[4] = device.deviceCode5;
	cc1101.sendBuffer[5] = device.deviceCode6;
	
	for(uint8_t i = 0;i < sizeof(RandomString); i++){
		cc1101.sendBuffer[_RFID_SIZE + i] = RandomString[i];
	}

	for(uint8_t i = 0;i < _STEP_LOOPNUM; i++){
		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i*2] = (uint8_t)(0xFF & step.stepArray[i]>>8);
		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i*2 + 1] = (uint8_t)(0xFF & step.stepArray[i]);
	}

	for(uint8_t i = 0;i < _STEP_LOOPNUM; i++){
		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2*_STEP_LOOPNUM + i*2] = (uint8_t)(0xFF & step.ingestionArray[i]>>8);
		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2*_STEP_LOOPNUM + i*2 + 1] = (uint8_t)(0xFF & step.ingestionArray[i]);
	}

//	for(uint8_t i = 0;i<_STEP_LOOPNUM; i++){
//		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i*2] = i*2;
//		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i*2 + 1] = i*2 + 1;
//	}
//	for(uint8_t i = 0;i<_STEP_LOOPNUM; i++){
//		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2*_STEP_LOOPNUM + i*2] = i*2;
//		cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2*_STEP_LOOPNUM + i*2 + 1] = i*2 + 1;
//	}

	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM] = step.stepStage;
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage)] = (uint8_t)(0xFF & adc.avgValue>>8);
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + 1] = (uint8_t)(0xFF & adc.avgValue);
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE] = (uint8_t)(0xFF & resetCnt>>8);
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + 1] = (uint8_t)(0xFF & resetCnt);

  cc1101.crcValue = ~HAL_CRC_Calculate(&hcrc, (uint32_t *)cc1101.sendBuffer, (uint32_t)(_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE));
	rfid_printf("BufferLength = %d\n",_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE);
	rfid_printf("crcValue = %x\n",cc1101.crcValue);

	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE] = (uint8_t)(0xFF & cc1101.crcValue>>24);
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + 1] = (uint8_t)(0xFF & cc1101.crcValue>>16);
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + 2] = (uint8_t)(0xFF & cc1101.crcValue>>8);
	cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + 3] = (uint8_t)(0xFF & cc1101.crcValue);
	
	for(uint16_t i=0; i<sizeof(cc1101.sendBuffer); i++)
	{	rfid_printf("%02x ",cc1101.sendBuffer[i]);}
	rfid_printf("\n");
	
	for(uint8_t i=0; i<3; i++)
	{
		HAL_Delay(_TX_WAIT_TIME);
		RFIDInitial(0x00, 0x1234, IDLE_MODE);
		CC1101SendPacket(cc1101.sendBuffer, _RFID_SIZE + sizeof(RandomString) + 4*_STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + _CRC32_SIZE, ADDRESS_CHECK);
		CC1101SetIdle();
		CC1101WriteCmd(CC1101_SXOFF);
	}
	memset(&cc1101, 0, sizeof(cc1101));
	
	#if (_DEBUG == 1)
		LED_GREEN_OFF();
	#endif
}
/*
================================================================================
Function : void CC1101Send3AxisHandler(void)
    Send RF Single
INPUT    : None
OUTPUT   : None
================================================================================
*/
//void CC1101Send3AxisHandler(void)
//{
//	#if (_DEBUG == 1)
//		LED_GREEN_ON();
//	#endif
//	
//	cc1101.sendBuffer[0] = device.deviceCode1;
//	cc1101.sendBuffer[1] = device.deviceCode2;
//	cc1101.sendBuffer[2] = device.deviceCode3;
//	cc1101.sendBuffer[3] = device.deviceCode4;
//	cc1101.sendBuffer[4] = device.deviceCode5;
//	cc1101.sendBuffer[5] = device.deviceCode6;
//	
//	for(uint8_t i = 0;i < sizeof(fifo); i++){
//		cc1101.sendBuffer[_RFID_SIZE + i] = fifo[i];
//	}

//  cc1101.crcValue = ~HAL_CRC_Calculate(&hcrc, (uint32_t *)cc1101.sendBuffer, (uint32_t)(_RFID_SIZE + sizeof(fifo)));
//	rfid_printf("BufferLength = %d\n",_RFID_SIZE + sizeof(fifo));
//	rfid_printf("crcValue = %x\n",cc1101.crcValue);

//	cc1101.sendBuffer[_RFID_SIZE + sizeof(fifo)] = (uint8_t)(0xFF & cc1101.crcValue>>24);
//	cc1101.sendBuffer[_RFID_SIZE + sizeof(fifo) + 1] = (uint8_t)(0xFF & cc1101.crcValue>>16);
//	cc1101.sendBuffer[_RFID_SIZE + sizeof(fifo) + 2] = (uint8_t)(0xFF & cc1101.crcValue>>8);
//	cc1101.sendBuffer[_RFID_SIZE + sizeof(fifo) + 3] = (uint8_t)(0xFF & cc1101.crcValue);

//	for(uint16_t i=0; i<sizeof(cc1101.sendBuffer); i++)
//	{	rfid_printf("%02x ",cc1101.sendBuffer[i]);}
//	rfid_printf("\n");

//	RFIDInitial(0x00, 0x1234, IDLE_MODE);
//	CC1101SendPacket(cc1101.sendBuffer, _RFID_SIZE + sizeof(fifo) + _CRC32_SIZE, ADDRESS_CHECK);
//	CC1101SetIdle();
//	CC1101WriteCmd(CC1101_SXOFF);
//	memset(&cc1101, 0, sizeof(cc1101));
//	
//	#if (_DEBUG == 1)
//		LED_GREEN_OFF();
//	#endif
//}
/*
================================================================================
------------------------------------THE END-------------------------------------
================================================================================
*/
