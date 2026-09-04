/**
 ******************************************************************************
 * @file    cc1101_app.c
 * @brief   RFID application payload handling built on the CC1101 driver.
 ******************************************************************************
 */

#include <string.h>

#include "cc1101.h"
#include "adc.h"
#include "adxl362.h"
#include "crc.h"
#include "gpio.h"
#include "usart.h"

/* Array that will be filled with random bytes */
extern uint8_t RandomString[32];
extern uint16_t resetCnt;

cc1101_t cc1101;

void RFIDInitial(uint8_t addr, uint16_t sync, TRMODE mode)
{
    CC1101Init(addr, sync);

    if (mode == RX_MODE)
    {
        CC1101SetTRMode(RX_MODE);
    }
    else if (mode == TX_MODE)
    {
        CC1101SetTRMode(TX_MODE);
    }
    else if (mode == IDLE_MODE)
    {
        CC1101SetIdle();
    }
    else if (mode == WOR_Mode)
    {
        CC1101SetIdle();
        CC1101WORInit();
        CC1101SetWORMode();
    }

    rxCatch = RESET;
}

uint8_t CC1101RecvHandler(void)
{
    if (rxCatch == SET)
    {
        HAL_Delay(4);
        rfid_printf("interrupt occur\n");

        for (uint8_t i = 0; i < sizeof(cc1101.recvBuffer); i++)
        {
            cc1101.recvBuffer[i] = 0;
        }

        cc1101.length = CC1101RecPacket(cc1101.recvBuffer, &cc1101.addr, &cc1101.rssi);

        cc1101.rssidBm = CC1101CalcRSSI_dBm(cc1101.rssi);
        rfid_printf("RSSI = %ddBm, length = %d, address = %d\n", cc1101.rssidBm, cc1101.length, cc1101.addr);

        for (uint8_t i = 0; i < sizeof(cc1101.recvBuffer); i++)
        {
            rfid_printf("%x ", cc1101.recvBuffer[i]);
        }

        rxCatch = RESET;

        if (cc1101.length == 0)
        {
            rfid_printf("receive error or Address Filtering fail\n");
            return 0x01;
        }
        else
        {
            if (cc1101.recvBuffer[3] == device.deviceCode1 && cc1101.recvBuffer[4] == device.deviceCode2 && cc1101.recvBuffer[5] == device.deviceCode3 && cc1101.recvBuffer[6] == device.deviceCode4 && cc1101.recvBuffer[7] == device.deviceCode5 && cc1101.recvBuffer[8] == device.deviceCode6)
            {
                if (cc1101.recvBuffer[2] == 0xC0)
                {
                    return cc1101.recvBuffer[2];
                }
                else
                {
                    rfid_printf("receive function order error\r\n");
                    return 0x03;
                }
            }
            else
            {
                rfid_printf("receive RFID code error\r\n");
                return 0x02;
            }
        }
    }
    else
    {
        return 0x00;
    }
}

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

    for (uint8_t i = 0; i < sizeof(RandomString); i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + i] = RandomString[i];
    }

    for (uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i * 2] = (uint8_t)(0xFF & step.restArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i * 2 + 1] = (uint8_t)(0xFF & step.restArray[i]);
    }

    for (uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.ingestionArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.ingestionArray[i]);
    }

    for (uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.movementArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.movementArray[i]);
    }

    for (uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 6 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.climbArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 6 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.climbArray[i]);
    }

    for (uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 8 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.ruminateArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 8 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.ruminateArray[i]);
    }

    for (uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 10 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.otherArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 10 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.otherArray[i]);
    }

    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM] = step.stepStage;
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage)] = (uint8_t)(0xFF & adc.avgValue >> 8);
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + 1] = (uint8_t)(0xFF & adc.avgValue);
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE] = (uint8_t)(0xFF & resetCnt >> 8);
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + 1] = (uint8_t)(0xFF & resetCnt);

    cc1101.crcValue = ~HAL_CRC_Calculate(&hcrc, (uint32_t *)cc1101.sendBuffer, (uint32_t)(_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE));
    rfid_printf("BufferLength = %d\n", _RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE);
    rfid_printf("crcValue = %x\n", cc1101.crcValue);

    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE] = (uint8_t)(0xFF & cc1101.crcValue >> 24);
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + 1] = (uint8_t)(0xFF & cc1101.crcValue >> 16);
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + 2] = (uint8_t)(0xFF & cc1101.crcValue >> 8);
    cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + 3] = (uint8_t)(0xFF & cc1101.crcValue);

    for (uint16_t i = 0; i < sizeof(cc1101.sendBuffer); i++)
    {
        rfid_printf("%02x ", cc1101.sendBuffer[i]);
    }

    rfid_printf("\n");

    CC1101_POWER_ON();
    RFIDInitial(0xEF, 0x1234, IDLE_MODE);
    CC1101SendPacket(cc1101.sendBuffer, _RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + _CRC32_SIZE, ADDRESS_CHECK);
    CC1101SetIdle();
    CC1101WriteCmd(CC1101_SPWD);
    CC1101_GDO_DeInit();
    CC1101_POWER_DOWN();

    memset(&cc1101, 0, sizeof(cc1101));

#if (_DEBUG == 1)
    LED_GREEN_OFF();
#endif
}

void CC1101Send3AxisHandler(void)
{
#if (_DEBUG == 1)
    LED_GREEN_ON();
#endif

    for (uint8_t j = 0; j < 6; j++)
    {
        cc1101.sendBuffer[0] = device.deviceCode1;
        cc1101.sendBuffer[1] = device.deviceCode2;
        cc1101.sendBuffer[2] = device.deviceCode3;
        cc1101.sendBuffer[3] = device.deviceCode4;
        cc1101.sendBuffer[4] = device.deviceCode5;
        cc1101.sendBuffer[5] = device.deviceCode6;
        cc1101.sendBuffer[6] = j;

        cc1101.sendBuffer[7] = action_classify_array[j];
        cc1101.sendBuffer[8] = (uint8_t)(0xFF & action_classify.rest >> 8);
        cc1101.sendBuffer[9] = (uint8_t)(0xFF & action_classify.rest);
        cc1101.sendBuffer[10] = (uint8_t)(0xFF & action_classify.ingestion >> 8);
        cc1101.sendBuffer[11] = (uint8_t)(0xFF & action_classify.ingestion);
        cc1101.sendBuffer[12] = (uint8_t)(0xFF & action_classify.movement >> 8);
        cc1101.sendBuffer[13] = (uint8_t)(0xFF & action_classify.movement);
        cc1101.sendBuffer[14] = (uint8_t)(0xFF & action_classify.climb >> 8);
        cc1101.sendBuffer[15] = (uint8_t)(0xFF & action_classify.climb);
        cc1101.sendBuffer[16] = (uint8_t)(0xFF & action_classify.ruminate >> 8);
        cc1101.sendBuffer[17] = (uint8_t)(0xFF & action_classify.ruminate);
        cc1101.sendBuffer[18] = (uint8_t)(0xFF & action_classify.other >> 8);
        cc1101.sendBuffer[19] = (uint8_t)(0xFF & action_classify.other);

        cc1101.sendBuffer[20] = (uint8_t)(0xFF & memory_array[0][0]);
        cc1101.sendBuffer[21] = (uint8_t)(0xFF & memory_array[1][0]);
        cc1101.sendBuffer[22] = (uint8_t)(0xFF & memory_array[2][0]);
        cc1101.sendBuffer[23] = (uint8_t)(0xFF & memory_array[3][0]);
        cc1101.sendBuffer[24] = (uint8_t)(0xFF & memory_array[4][0]);
        cc1101.sendBuffer[25] = (uint8_t)(0xFF & memory_array[5][0]);
        cc1101.sendBuffer[26] = (uint8_t)(0xFF & memory_array[6][0]);
        cc1101.sendBuffer[27] = (uint8_t)(0xFF & memory_array[7][0]);
        cc1101.sendBuffer[28] = (uint8_t)(0xFF & memory_array[8][0]);
        cc1101.sendBuffer[29] = (uint8_t)(0xFF & memory_array[9][0]);
        cc1101.sendBuffer[30] = (uint8_t)(0xFF & memory_array[10][0]);
        cc1101.sendBuffer[31] = (uint8_t)(0xFF & memory_array[11][0]);
        cc1101.sendBuffer[32] = (uint8_t)(0xFF & memory_array[12][0]);
        cc1101.sendBuffer[33] = (uint8_t)(0xFF & memory_array[13][0]);
        cc1101.sendBuffer[34] = (uint8_t)(0xFF & memory_array[14][0]);
        cc1101.sendBuffer[35] = (uint8_t)(0xFF & memory_array[15][0]);
        cc1101.sendBuffer[36] = (uint8_t)(0xFF & memory_array[16][0]);
        cc1101.sendBuffer[37] = (uint8_t)(0xFF & memory_array[17][0]);

        for (uint8_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
        {
            cc1101.sendBuffer[_RFID_SIZE + 14 + 18 + i] = fifo[i + (_FIFO_SAMPLES_LEN / 6) * j];
        }

        cc1101.crcValue = ~HAL_CRC_Calculate(&hcrc, (uint32_t *)cc1101.sendBuffer, (uint32_t)(_RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6));
        rfid_printf("BufferLength = %d\n", (_RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6));
        rfid_printf("crcValue = %x\n", cc1101.crcValue);

        cc1101.sendBuffer[_RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6] = (uint8_t)(0xFF & cc1101.crcValue >> 24);
        cc1101.sendBuffer[_RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6 + 1] = (uint8_t)(0xFF & cc1101.crcValue >> 16);
        cc1101.sendBuffer[_RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6 + 2] = (uint8_t)(0xFF & cc1101.crcValue >> 8);
        cc1101.sendBuffer[_RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6 + 3] = (uint8_t)(0xFF & cc1101.crcValue);

        for (uint16_t i = 0; i < sizeof(cc1101.sendBuffer); i++)
        {
            rfid_printf("%02x ", cc1101.sendBuffer[i]);
        }

        rfid_printf("\n");

        CC1101_POWER_ON();
        RFIDInitial(0x08, 0x0973, IDLE_MODE);
        CC1101SendPacket(cc1101.sendBuffer, _RFID_SIZE + 14 + 18 + _FIFO_SAMPLES_LEN / 6 + _CRC32_SIZE, ADDRESS_CHECK);
        CC1101SetIdle();
        CC1101WriteCmd(CC1101_SPWD);
        CC1101_GDO_DeInit();
        CC1101_POWER_DOWN();
        HAL_Delay(300);
        memset(&cc1101, 0, sizeof(cc1101));
    }

#if (_DEBUG == 1)
    LED_GREEN_OFF();
#endif
}
