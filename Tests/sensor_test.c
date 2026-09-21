#include "sensor.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t regs[256];
static bool cs=true, enabled;
static unsigned chunks[3], calls, fifo_offset, fail_at;
static uint8_t bad_readback;
void board_pin_write(board_pin_t p,bool h) { assert(p==PIN_SENSOR_CS); cs=h; }
void board_spi_enable(unsigned bus,bool on) { assert(bus==2); if(!on) assert(cs); enabled=on; }
void board_delay(uint32_t ms) { (void)ms; }
rfid_status_t board_spi_exchange(unsigned bus,uint8_t *p,size_t n)
{
    assert(bus==2 && !cs && enabled && n<=512);
    if(p[0]==0x0d) {
        assert((n&1)==1 && n>=3); assert(calls<3);
        chunks[calls++]=(unsigned)n-1;
        if(fail_at==calls) return RFID_TIMEOUT;
        for(size_t i=1;i<n;++i) p[i]=(uint8_t)fifo_offset++;
    } else {
        assert(n>=3);
        uint8_t command=p[0], address=p[1];
        if(command==0x0a && address==0x2c && p[2]!=0x51)
            assert(regs[0x2d]==0);
        for(size_t i=2;i<n;++i) {
            if(command==0x0a) regs[address++]=p[i];
            else { assert(command==0x0b); p[i]=regs[address];
                   if(bad_readback && address==bad_readback && regs[address]!=0x51) p[i]^=1;
                   address++; }
        }
    }
    return RFID_OK;
}
int main(void)
{
    regs[0]=0xad; regs[1]=0x1d; regs[2]=0xf2;
    assert(sensor_init()==RFID_OK);
    assert(cs && !enabled);
    assert(regs[0x20]==100 && regs[0x22]==6 && regs[0x23]==100 && regs[0x25]==6);
    assert(regs[0x27]==0x3f && regs[0x28]==0x0a && regs[0x29]==0xc2);
    assert(regs[0x2a]==0x10 && regs[0x2b]==4 && regs[0x2c]==0x51 && regs[0x2d]==2);
    assert(sensor_data_ready_begin()==RFID_OK);
    assert(regs[0x28]==0 && regs[0x2b]==1 && regs[0x2c]==0x51 && regs[0x2d]==2);
    assert(sensor_data_ready_end()==RFID_OK && regs[0x2d]==0);
    regs[0x2c]=0; assert(sensor_data_ready_begin()==RFID_IO);
    assert(sensor_data_ready_begin_rate(200)==RFID_INVALID);
    for(unsigned hz=25;hz<=100;hz*=2) {
        assert(sensor_init()==RFID_OK);
        assert(sensor_data_ready_begin_rate((uint16_t)hz)==RFID_OK);
        assert(regs[0x2c]==(hz==25?0x51:hz==50?0x52:0x53));
        assert(regs[0x28]==0 && regs[0x2b]==1 && regs[0x2d]==2);
        assert(sensor_data_ready_end()==RFID_OK);
    }
    assert(sensor_init()==RFID_OK); bad_readback=0x2c;
    assert(sensor_data_ready_begin_rate(50)==RFID_IO && regs[0x2d]==0);
    bad_readback=0;
    assert(sensor_init()==RFID_OK);
    uint8_t data[1026];
    for(unsigned length=900;length<=1024;length+=124) {
        memset(data,0xa5,sizeof(data)); calls=0; fifo_offset=0;
        assert(sensor_fifo_read(data+1,length)==RFID_OK);
        assert(chunks[0]==510 && chunks[1]==(length==900?390:510));
        assert(calls==(length==900?2:3));
        if(length==1024) assert(chunks[2]==4);
        assert(data[0]==0xa5 && data[length+1]==0xa5);
        for(unsigned i=0;i<length;++i) assert(data[i+1]==(uint8_t)i);
    }
    for(fail_at=1;fail_at<=3;++fail_at) {
        calls=0; memset(data,0xa5,sizeof(data));
        assert(sensor_fifo_read(data,1024)==RFID_TIMEOUT && calls==fail_at);
        for(unsigned i=0;i<1024;++i) assert(data[i]==0);
        assert(cs && !enabled);
    }
    fail_at=0;
    assert(sensor_fifo_read(data,901)==RFID_INVALID);
    uint16_t entries; bool overrun;
    regs[0x0c]=0xc2; regs[0x0d]=1; regs[0x0b]=8;
    assert(sensor_fifo_entries(&entries,&overrun)==RFID_OK && entries==450 && overrun);
    assert(sensor_fifo_restart()==RFID_OK && regs[0x28]==0x0a);
    regs[2]=0; assert(sensor_init()!=RFID_OK);
    regs[2]=0xf2; assert(sensor_init()==RFID_OK);
    puts("PASS: official ADI driver, fixed allocator, chunking and transport errors"); return 0;
}
