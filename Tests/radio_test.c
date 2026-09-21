#include "radio.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "legacy/rf_config.h"
static uint8_t regs[64], pa[8], wire[300], rx[300], command, configured[47];
static bool pins[8], header, active_tx, active_rx, so_stuck, spi_fail, tx_stuck, underflow, rx_overflow;
static size_t tx_count, wire_size, expected, rx_read, rx_size, rx_available, configured_count;
static uint32_t now;
static bool reset_stuck;
uint32_t board_millis(void) { return now; }
void board_delay(uint32_t n) { now+=n; if(active_rx && rx_available<rx_size) { rx_available+=8; if(rx_available>rx_size) rx_available=rx_size; } }
void board_spi_enable(unsigned b,bool on) { assert(b==1); (void)on; }
void board_radio_off(void) { pins[PIN_RADIO_POWER]=pins[PIN_TX_EN]=pins[PIN_RX_EN]=false; active_tx=active_rx=false; }
void board_pin_write(board_pin_t p,bool h) { pins[p]=h; if(p==PIN_RADIO_CS && !h) header=true; }
bool board_pin_read(board_pin_t p)
{
    if(p==PIN_RADIO_SO) return so_stuck;
    if(p==PIN_GDO0) return !active_tx;
    return pins[p];
}
rfid_status_t board_spi_exchange(unsigned bus,uint8_t *data,size_t n)
{
    assert(bus==1 && !pins[PIN_RADIO_CS] && !so_stuck);
    if(spi_fail) return RFID_IO;
    if(header) {
        assert(n==1); header=false; command=data[0]; data[0]=0;
        if(command==0x30) { memset(regs,0,sizeof(regs)); configured_count=0; so_stuck=reset_stuck; }
        if(command==0x35) active_tx=true;
        if(command==0x36) active_tx=active_rx=false;
        if(command==0x3b) tx_count=0;
        if(command==0x34) { active_rx=true; rx_available=rx_size<8?rx_size:8; }
        return RFID_OK;
    }
    if(command==0x7f) {
        assert(n<=60 && tx_count+n<=64 && wire_size+n<sizeof(wire));
        memcpy(wire+wire_size,data,n); wire_size+=n; tx_count+=n; expected=(size_t)wire[0]+1;
    } else if(command==0xff) {
        assert(rx_available-rx_read>=n);
        /* Before all bytes arrive, the driver must retain at least one byte. */
        assert(rx_available==rx_size || rx_available-rx_read>n);
        memcpy(data,rx+rx_read,n); rx_read+=n;
    } else if(command==0x7e) { assert(n==8); memcpy(pa,data,n); }
    else if(command&0x80) {
        assert(n==1); uint8_t reg=command&0x3f;
        if(reg==0x30) *data=0;
        else if(reg==0x31) *data=0x14;
        else if(reg==0x3a) *data=underflow?0x80:(uint8_t)tx_count;
        else if(reg==0x3b) *data=rx_overflow?0x80:(uint8_t)(rx_available-rx_read);
        else if(reg==0x35) *data=active_tx?0x13:1;
        else *data=regs[reg];
    } else {
        assert(n==1); regs[command]=*data;
        if(configured_count<47) { assert(command==configured_count); configured[configured_count++]=*data; }
    }
    return RFID_OK;
}
static void reset_mock(void)
{
    radio_abort(); memset(pins,0,sizeof(pins)); pins[PIN_RADIO_CS]=true;
    wire_size=tx_count=expected=rx_read=rx_size=rx_available=0; now=0;
    so_stuck=spi_fail=tx_stuck=underflow=rx_overflow=reset_stuck=false; active_tx=active_rx=false;
}
static rfid_status_t send(const uint8_t *p,size_t n)
{
    assert(radio_begin(p,n)==RFID_OK);
    rfid_status_t s=RFID_BUSY;
    for(unsigned tick=0;tick<2000 && s==RFID_BUSY;++tick) {
        if(tick%4==0) ++now;
        if(active_tx && !tx_stuck && tx_count) {
            --tx_count;
            if(!tx_count) {
                if(wire_size==expected) active_tx=false;
                else underflow=true;
            }
        }
        s=radio_poll();
    }
    assert(!pins[PIN_RADIO_POWER] && !pins[PIN_TX_EN] && !pins[PIN_RX_EN]);
    return s;
}
int main(void)
{
    uint8_t data[255]; for(unsigned i=0;i<sizeof(data);++i) data[i]=(uint8_t)i;
    const unsigned lengths[]={1,59,60,61,120,121,180,191,240,254};
    for(unsigned k=0;k<sizeof(lengths)/sizeof(lengths[0]);++k) {
        reset_mock(); unsigned n=lengths[k]; assert(send(data,n)==RFID_OK);
        assert(wire_size==n+2 && wire[0]==n+1 && wire[1]==0xef && !memcmp(wire+2,data,n));
        assert(!memcmp(gold,configured,47) && pa[0]==0xc0);
        for(unsigned i=0;i<47;++i) assert(radio_config_byte(i)==gold[i]);
        assert(regs[4]==0x12 && regs[5]==0x34 && regs[7]==7);
    }
    reset_mock(); so_stuck=true; assert(send(data,191)==RFID_TIMEOUT);
    reset_mock(); reset_stuck=true; assert(send(data,191)==RFID_TIMEOUT);
    reset_mock(); spi_fail=true; assert(send(data,191)==RFID_IO);
    reset_mock(); tx_stuck=true; assert(send(data,191)==RFID_TIMEOUT);
    reset_mock(); underflow=true; assert(send(data,191)==RFID_OVERFLOW);
    reset_mock(); assert(radio_begin(data,255)==RFID_INVALID);
    for(unsigned failure=0;failure<4;++failure) {
        reset_mock(); rx[0]=192; rx[1]=failure==2?0x12:0xef;
        memcpy(rx+2,data,191); rx[193]=0x80; rx[194]=failure==1?0:0x80; rx_size=195;
        rx_overflow=failure==3;
        size_t n=999; uint8_t result[191];
        rfid_status_t status=radio_receive(result,sizeof(result),&n,250);
        const rfid_status_t want[]={RFID_OK,RFID_CRC,RFID_INVALID,RFID_OVERFLOW};
        assert(status==want[failure] && !pins[PIN_RADIO_POWER]);
        if(!failure) assert(n==191 && !memcmp(result,data,n)); else assert(n==0);
    }
    reset_mock(); size_t n; assert(radio_receive(data,sizeof(data),&n,10)==RFID_TIMEOUT);
    puts("PASS: RF configuration, TX boundaries, RX streaming/CRC and bounded failures"); return 0;
}
