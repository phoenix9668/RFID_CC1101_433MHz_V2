/* Hardware-free protocol oracle: original CC1101SendHandler from 34d57c2. */
#include "rfid_types.h"
#include <assert.h>
#include <string.h>
#define _DEBUG 0
#define _RFID_SIZE 6
#define _STEP_LOOPNUM 12
#define _BATTERY_SIZE 2
#define _RESETCNT_SIZE 2
#define _CRC32_SIZE 4
#define rfid_printf(...) ((void)0)
#define CC1101_POWER_ON() ((void)0)
#define CC1101_POWER_DOWN() ((void)0)
#define RFIDInitial(...) ((void)0)
#define CC1101SetIdle() ((void)0)
#define CC1101WriteCmd(...) ((void)0)
#define CC1101_GDO_DeInit() ((void)0)
#define ADDRESS_CHECK 0
static uint8_t RandomString[32], *wire;
static uint16_t resetCnt;
static struct { uint8_t sendBuffer[191]; uint32_t crcValue; } cc1101;
static struct { uint8_t deviceCode1,deviceCode2,deviceCode3,deviceCode4,deviceCode5,deviceCode6; } device;
static struct { uint16_t restArray[12],ingestionArray[12],movementArray[12],climbArray[12],ruminateArray[12],otherArray[12]; uint8_t stepStage; } step;
static struct { uint16_t avgValue; } adc;
static uint32_t hcrc;
/* Model the base CRC peripheral: byte input reversal, polynomial 04C11DB7,
   initial FFFFFFFF and output bit reversal, without final complement. */
static uint32_t HAL_CRC_Calculate(uint32_t *unused,uint32_t *input,uint32_t length)
{
    (void)unused;
    const uint8_t *bytes=(const uint8_t*)input;
    uint32_t crc=0xffffffffU, reflected=0;
    for(uint32_t i=0;i<length;++i) for(unsigned b=0;b<8;++b) {
        unsigned carry=((crc>>31)^(bytes[i]>>b))&1U;
        crc<<=1; if(carry) crc^=0x04c11db7U;
    }
    for(unsigned i=0;i<32;++i) { reflected=(reflected<<1)|(crc&1); crc>>=1; }
    return reflected;
}
static void CC1101SendPacket(const uint8_t *p,size_t n,int mode)
{
    (void)mode; assert(n==191); memcpy(wire,p,n);
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

    for(uint8_t i = 0; i < sizeof(RandomString); i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + i] = RandomString[i];
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i * 2] = (uint8_t)(0xFF & step.restArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i * 2 + 1] = (uint8_t)(0xFF & step.restArray[i]);
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.ingestionArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.ingestionArray[i]);
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.movementArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 4 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.movementArray[i]);
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 6 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.climbArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 6 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.climbArray[i]);
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 8 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.ruminateArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 8 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.ruminateArray[i]);
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 10 * _STEP_LOOPNUM + i * 2] = (uint8_t)(0xFF & step.otherArray[i] >> 8);
        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 10 * _STEP_LOOPNUM + i * 2 + 1] = (uint8_t)(0xFF & step.otherArray[i]);
    }

//    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
//    {
//        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i * 2] = i * 2;
//        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + i * 2 + 1] = i * 2 + 1;
//    }

//    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
//    {
//        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2 * _STEP_LOOPNUM + i * 2] = i * 2;
//        cc1101.sendBuffer[_RFID_SIZE + sizeof(RandomString) + 2 * _STEP_LOOPNUM + i * 2 + 1] = i * 2 + 1;
//    }

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

    for(uint16_t i = 0; i < sizeof(cc1101.sendBuffer); i++)
    {
        rfid_printf("%02x ", cc1101.sendBuffer[i]);
    }

    rfid_printf("\n");

//    for(uint8_t i = 0; i < 3; i++)
//    {
//    HAL_Delay(_TX_WAIT_TIME);
    CC1101_POWER_ON();
    RFIDInitial(0xEF, 0x1234, IDLE_MODE);
    CC1101SendPacket(cc1101.sendBuffer, _RFID_SIZE + sizeof(RandomString) + 12 * _STEP_LOOPNUM + sizeof(step.stepStage) + _BATTERY_SIZE + _RESETCNT_SIZE + _CRC32_SIZE, ADDRESS_CHECK);
    CC1101SetIdle();
    CC1101WriteCmd(CC1101_SPWD);
    CC1101_GDO_DeInit();
    CC1101_POWER_DOWN();
//    }

    memset(&cc1101, 0, sizeof(cc1101));

    #if (_DEBUG == 1)
    LED_GREEN_OFF();
    #endif
}

void legacy_encode(uint8_t out[191],const uint8_t id[6],const uint8_t random[32],const rfid_history_t *s,uint16_t battery)
{
    wire=out; memcpy(&device,id,6); memcpy(RandomString,random,32);
    memcpy(&step,s->history,144); step.stepStage=s->stage; resetCnt=s->reset_count; adc.avgValue=battery;
    CC1101SendHandler();
}
