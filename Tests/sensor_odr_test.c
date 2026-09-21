#include "sensor_odr_test.h"
#include "sensor.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
static uint32_t now, due, period;
static bool high;
static unsigned reads, begins, ends, fault;
uint32_t board_millis(void) { return now; }
uint32_t board_rtc_millis(void) { return now; }
void board_delay(uint32_t ms) { now += ms; }
void board_watchdog(void) {}
void board_log(const char *s) { (void)s; }
bool board_pin_read(board_pin_t p)
{
    assert(p==PIN_SENSOR_IRQ);
    if(fault!=1 && (int32_t)(now-due)>=0) high=true;
    return high;
}
rfid_status_t sensor_init(void) { return fault==4 ? RFID_IO : RFID_OK; }
rfid_status_t sensor_data_ready_begin(void) { begins++; return RFID_OK; }
rfid_status_t sensor_data_ready_end(void) { ends++; return RFID_OK; }
rfid_status_t sensor_read_register(uint8_t address,uint8_t *value)
{
    *value=0;
    if(address==3) return RFID_OK;
    assert(address==0x0e);
    reads++;
    if(fault==3) return RFID_IO;
    if(fault!=2) high=false;
    due+=period;
    return RFID_OK;
}
static void setup(unsigned mode,uint32_t tick,uint32_t interval)
{
    fault=mode; now=tick; period=interval; due=now+period; high=false;
    reads=begins=ends=0;
}
int main(void)
{
    setup(0,0,40); assert(sensor_odr_test_run(1200)==RFID_OK);
    assert(reads==29 && begins==1 && ends==1);
    setup(0,UINT32_MAX-100,51); assert(sensor_odr_test_run(1200)==RFID_OK);
    assert(reads==23 && ends==1);
    setup(0,0,40); assert(sensor_odr_test_run(240000)==RFID_OK);
    assert(now==240000 && reads==5999 && ends==1);
    setup(1,0,40); assert(sensor_odr_test_run(1200)==RFID_TIMEOUT && now<300 && ends==1);
    setup(2,0,40); assert(sensor_odr_test_run(1200)==RFID_TIMEOUT && reads==1 && now<60);
    setup(3,0,40); assert(sensor_odr_test_run(1200)==RFID_IO && ends==1);
    setup(4,0,40); assert(sensor_odr_test_run(1200)==RFID_IO && begins==0 && ends==1);
    setup(0,0,40); assert(sensor_odr_test_run(240001)==RFID_INVALID && begins==0);
    assert(sensor_odr_test_run(999)==RFID_INVALID && begins==0);
    puts("PASS: bounded DATA_READY bench, wrap, stuck pins and SPI/init faults");
}
