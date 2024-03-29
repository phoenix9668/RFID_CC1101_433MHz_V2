/**
  ******************************************************************************
  * @file    cc1101.h
  * @author  phoenix
  * @version V1.0.0
  * @date    20-October-2017
  * @brief   This file provides set of cc1101 to manage RF functions
  *          available on STM32F4-Discovery Kit from STMicroelectronics.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#ifndef _CC1101_H_
#define _CC1101_H_

#include "cc1101_reg.h"
#include "main.h"

/*===========================================================================
----------------------------------macro definitions--------------------------
============================================================================*/
typedef enum { TX_MODE, RX_MODE, IDLE_MODE, WOR_Mode } TRMODE;
typedef enum { BROAD_ALL, BROAD_NO, BROAD_0, BROAD_0AND255 } ADDR_MODE;
typedef enum { BROADCAST, ADDRESS_CHECK} TX_DATA_MODE;

#define _RECV_LENGTH               128
#define _SEND_LENGTH               256
#define	_TX_WAIT_TIME			   			 50 // cc1101 tx wait time
#define	_RFID_SIZE                 6   // RFID size
#define	_BATTERY_SIZE              2   // battery size
#define	_RESETCNT_SIZE             2   // resetCnt size
#define	_CRC32_SIZE                4   // CRC32 size

typedef struct
{
    uint8_t  recvBuffer[_RECV_LENGTH];
    uint8_t  sendBuffer[_SEND_LENGTH];
    uint32_t crcValue;
    uint8_t  length;
    uint8_t  addr;
    uint8_t  rssi;
    int16_t  rssidBm;
} cc1101_t;

extern cc1101_t cc1101;
extern __IO ITStatus txFiFoUnFlow;
extern __IO ITStatus rxCatch;

/*===========================================================================
-------------------------------------exported APIs---------------------------
============================================================================*/

/*read a byte from the specified register*/
uint8_t CC1101ReadReg(uint8_t addr);

/*Read a status register*/
uint8_t CC1101ReadStatus(uint8_t addr);

/*Set the device as TX mode or RX mode*/
void CC1101SetTRMode(TRMODE mode);

/*Write a command byte to the device*/
void CC1101WriteCmd(uint8_t command);

/*Set the CC1101 into IDLE mode*/
void CC1101SetIdle(void);

/*Send a packet*/
void CC1101SendPacket(uint8_t *txbuffer, uint8_t size, TX_DATA_MODE mode);

/*Set the address and address mode of the CC1101*/
void CC1101SetAddress(uint8_t address, ADDR_MODE AddressMode);

/*Set the SYNC bytes of the CC1101*/
void CC1101SetSYNC(uint16_t sync);

/*Receive a packet*/
uint8_t CC1101RecPacket(uint8_t *rxBuffer, uint8_t *addr, uint8_t *rssi);

/*Initialize the WOR function of CC1101*/
void CC1101WORInit(void);

/*Set the device as the WOR mode*/
void CC1101SetWORMode(void);

/*Initialize the CC1101, User can modify it*/
void CC1101Init(uint8_t addr, uint16_t sync);

/*Initialize the CC1101 GDO0/2, User can modify it*/
void CC1101_GDO_Init(void);

/*Disable the CC1101 GDO0/2, User can modify it*/
void CC1101_GDO_DeInit(void);

/*Initialize the CC1101, Configure addr&sync&mode*/
void RFIDInitial(uint8_t addr, uint16_t sync, TRMODE mode);

/*read some bytes from the rigisters continously*/
void CC1101ReadMultiReg(uint8_t addr, uint8_t *buff, uint8_t size);

/*write a byte to the specified register*/
void CC1101WriteReg(uint8_t addr, uint8_t value);

/*flush the TX buffer of CC1101*/
void CC1101ClrTXBuff(void);

/*flush the RX buffer of CC1101*/
void CC1101ClrRXBuff(void);

/*get received count of CC1101*/
uint8_t CC1101GetRXCnt(void);

/*reset the CC1101 device*/
void CC1101Reset(void);

/*write some bytes to the specified register*/
void CC1101WriteMultiReg(uint8_t addr, uint8_t *buff, uint8_t size);

/*Read the RSSI value in rx*/
int16_t CC1101ReadRSSI(void);

/*Calc the RSSI value to RSSI dBm*/
int16_t CC1101CalcRSSI_dBm(uint8_t rssi_dec);

/*Receive RF Single*/
uint8_t CC1101RecvHandler(void);

/*Send RF Single*/
void CC1101SendHandler(void);
void CC1101Send3AxisHandler(void);

#endif // _CC1101_H_

/******************* END OF FILE ******************/
