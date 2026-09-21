#include "sensor_odr_test.h"
#include "sensor.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
static uint32_t now, due, period, log_cost;
static bool high, sweeping, active;
static unsigned reads, clears, begins, ends, fault, phase_reads;
static uint16_t sequence[18], nominal;
uint32_t board_millis(void) { return now; }
uint32_t board_rtc_millis(void) { return now; }
void board_delay(uint32_t ms) { now += ms; }
void board_watchdog(void) {}
void board_log(const char *s)
{
    (void)s;
    if(sweeping) assert(!active || phase_reads==0);
    now+=log_cost;
}
static void update_ready(void)
{
    if(active && fault!=1 && (int32_t)(now-due)>=0) {
        high=true;
        do { due+=period; } while((int32_t)(now-due)>=0);
    }
}
bool board_pin_read(board_pin_t p)
{
    assert(p==PIN_SENSOR_IRQ);
    update_ready();
    return high;
}
rfid_status_t sensor_init(void) { return fault==4 ? RFID_IO : RFID_OK; }
rfid_status_t sensor_data_ready_begin_rate(uint16_t hz)
{
    assert(hz==25 || hz==50 || hz==100);
    assert(begins<18);
    sequence[begins++]=hz;
    if(fault==6 && begins==5) return RFID_IO;
    nominal=hz;
    if(sweeping) period=(fault==5 ? 1266U : 1000U)/hz;
    due=now+period; high=false; phase_reads=0; active=true;
    return RFID_OK;
}
rfid_status_t sensor_data_ready_end(void) { ends++; active=false; return RFID_OK; }
rfid_status_t sensor_read_register(uint8_t address,uint8_t *value)
{
    *value=0;
    if(address==3) return RFID_OK;
    if(address==0x2c) { *value=nominal==25?0x51:nominal==50?0x52:0x53; return RFID_OK; }
    assert(address==0x0e);
    update_ready();
    if(fault==3) return RFID_IO;
    if(high) { reads++; phase_reads++; }
    else clears++;
    if(fault!=2) high=false;
    return RFID_OK;
}
static void setup(unsigned mode,uint32_t tick,uint32_t interval)
{
    fault=mode; now=tick; period=interval; due=now+period;
    high=sweeping=active=false; log_cost=0;
    reads=clears=begins=ends=phase_reads=0;
}
int main(void)
{
    setup(0,0,40); assert(sensor_odr_test_run(1200)==RFID_OK);
    assert(reads==30 && clears==1 && begins==1 && ends==1);
    setup(0,UINT32_MAX-100,51); assert(sensor_odr_test_run(1200)==RFID_OK);
    assert(reads==23 && ends==1);
    setup(0,0,40); assert(sensor_odr_test_run(240000)==RFID_OK);
    assert(now==240001 && reads==6000 && ends==1);
    setup(1,0,40); assert(sensor_odr_test_run(1200)==RFID_TIMEOUT && now<300 && ends==1);
    setup(2,0,40); assert(sensor_odr_test_run(1200)==RFID_TIMEOUT && reads==1 && now<60);
    setup(3,0,40); assert(sensor_odr_test_run(1200)==RFID_IO && ends==1);
    setup(4,0,40); assert(sensor_odr_test_run(1200)==RFID_IO && begins==0 && ends==1);
    setup(0,0,40); assert(sensor_odr_test_run(240001)==RFID_INVALID && begins==0);
    assert(sensor_odr_test_run(999)==RFID_INVALID && begins==0);
    for(unsigned mode=0;mode<=5;mode+=5) {
        setup(mode,UINT32_MAX-100,40); sweeping=true; log_cost=10;
        uint32_t start=now;
        assert(sensor_odr_sweep_run()==RFID_OK && begins==18 && ends==18);
        assert((uint32_t)(now-start)>216000 && (uint32_t)(now-start)<220000);
        for(unsigned i=0;i<18;i++) assert(sequence[i]==(i%3==0?25:i%3==1?50:100));
        unsigned expected=0;
        for(unsigned hz=25;hz<=100;hz*=2)
            expected+=6*(10010/((mode==5?1266U:1000U)/hz));
        assert(reads==expected);
    }
    setup(6,0,40); sweeping=true;
    assert(sensor_odr_sweep_run()==RFID_IO && begins==5 && ends==5 && !active);
    puts("PASS: bounded DATA_READY bench/sweep, wrap, log isolation and faults");
}
