#include "sensor_extclock_test.h"
#include "sensor_test_clock.h"
#include "sensor.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now, due, period, clock_started;
static unsigned fault, inits, prepares, starts, measures, ends, reads, active_reads;
static bool clock_on, prepared, measuring, high, external;
static uint8_t power;
static sensor_test_clock_stats_t stats;
uint32_t board_millis(void) { return now; }
uint32_t board_rtc_millis(void) { return now; }
void board_watchdog(void) {}
void board_delay(uint32_t ms) { now+=ms; }
void board_log(const char *s)
{
    assert(!measuring || active_reads==0 || strstr(s,"EXTCLK FAULT"));
    assert(s && strlen(s)<192);
    now+=3;
}
static void update(void)
{
    if(measuring && fault!=5 && (int32_t)(now-due)>=0) {
        high=true;
        do { due+=period; } while((int32_t)(now-due)>=0);
    }
}
bool board_pin_read(board_pin_t pin)
{
    assert(pin==PIN_SENSOR_IRQ);
    update(); return high;
}
rfid_status_t sensor_init(void)
{
    assert(!clock_on); inits++; prepared=false; power=2;
    return fault==1 ? RFID_IO : RFID_OK;
}
rfid_status_t sensor_clock_test_prepare(void)
{
    assert(!clock_on); prepares++; power=0;
    if(fault==2) return RFID_IO;
    prepared=true; return RFID_OK;
}
rfid_status_t sensor_clock_test_measure(bool ext)
{
    assert(prepared && power==0 && clock_on==ext);
    assert(ext==((measures&1)!=0)); measures++;
    if(fault==3) return RFID_IO;
    external=ext; power=ext?0x42:2; period=ext?64:51;
    measuring=true; high=false; due=now+period; active_reads=0;
    return RFID_OK;
}
rfid_status_t sensor_clock_test_end(void)
{
    ends++; measuring=false; power=0;
    return fault==7 ? RFID_IO : RFID_OK;
}
rfid_status_t sensor_read_register(uint8_t address,uint8_t *value)
{
    if(address==0x0b || address==0x2b) { *value=1; return RFID_OK; }
    assert(measuring && clock_on==external);
    *value=0;
    if(address==3) { *value=3; return RFID_OK; }
    if(address==0x2c) { *value=0x51; return RFID_OK; }
    if(address==0x2d) { *value=power; return RFID_OK; }
    assert(address==0x0e);
    if(fault==4) return RFID_IO;
    update(); reads++; active_reads++;
    if(fault==11 && active_reads==1) high=true;
    else if(fault!=6) high=false;
    return RFID_OK;
}
rfid_status_t sensor_test_clock_start(void)
{
    assert(prepared && power==0 && !clock_on);
    starts++;
    if(fault==8) return RFID_IO;
    clock_on=true; clock_started=now; stats=(sensor_test_clock_stats_t){0};
    return RFID_OK;
}
rfid_status_t sensor_test_clock_status(void)
{
    assert(clock_on);
    return fault==9 || (fault==10 && (uint32_t)(now-clock_started)>900) ? RFID_TIMEOUT : RFID_OK;
}
sensor_test_clock_stats_t sensor_test_clock_stats(void) { assert(!clock_on); return stats; }
void sensor_test_clock_stop(void)
{
    if(clock_on) { assert(!measuring); stats.edges=(now-clock_started)*64; }
    clock_on=false;
}
static void setup(unsigned mode)
{
    now=UINT32_MAX-1000; fault=mode;
    inits=prepares=starts=measures=ends=reads=active_reads=0;
    clock_on=prepared=measuring=high=external=false;
}
int main(void)
{
    for(unsigned mode=0;mode<=11;mode+=11) {
        setup(mode); uint32_t start=now;
        assert(sensor_extclock_test_run()==RFID_OK);
        assert(inits==24 && prepares==24 && starts==12 && measures==24 && ends==24);
        assert(!clock_on && !measuring && power==0 && reads>4200 && reads<4400);
        assert((uint32_t)(now-start)>302000 && (uint32_t)(now-start)<304000);
    }
    for(unsigned mode=1;mode<=10;++mode) {
        setup(mode);
        assert(sensor_extclock_test_run()!=RFID_OK);
        assert(!clock_on && !measuring && power==0);
        assert(ends==(mode>=8?2U:1U));
        if(mode<=2) assert(starts==0 && measures==0);
    }
    puts("PASS: external-clock A/B, safe pin sequencing, wrap, failures and cleanup");
}
