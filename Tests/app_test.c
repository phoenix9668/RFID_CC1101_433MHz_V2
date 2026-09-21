#include "app.h"
#include "board.h"
#include "sensor.h"
#include "radio.h"
#include "protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t now, ticks, nv[512], critical, report_count, radio_end, watchdogs;
static unsigned fifo_words, sample_number, fifo_failures, overrun_requests, writes;
static bool radio_active, fail_radio, fail_store, post_during_write;
static bool allow_stop=true;
static uint8_t sent[191];
uint32_t board_millis(void) { return ticks; }
uint32_t board_rtc_millis(void) { return now; }
uint32_t board_critical_enter(void) { uint32_t p=critical; critical=1; return p; }
void board_critical_exit(uint32_t p) { critical=p; }
void board_watchdog(void) { ++watchdogs; }
void board_log(const char *p) { (void)p; }
uint32_t board_eeprom_read(void *c,uint16_t o) { (void)c; return nv[o/4]; }
rfid_status_t board_eeprom_write(void *c,uint16_t o,uint32_t v)
{
    (void)c; assert(o>=0x240 && o<0x800 && !(o&3)); ++writes;
    if(post_during_write) { post_during_write=false; app_signal(APP_EVENT_FIFO); }
    if(fail_store) return RFID_IO;
    nv[o/4]=v; return RFID_OK;
}
rfid_status_t board_battery(uint16_t *v) { *v=0x765; return RFID_OK; }
void board_identity(uint8_t id[6],uint64_t *s,uint64_t *q) { memset(id,0x55,6); *s=123; *q=456; }
rfid_status_t sensor_init(void) { fifo_words=0; return RFID_OK; }
rfid_status_t sensor_fifo_restart(void) { fifo_words=0; return RFID_OK; }
rfid_status_t sensor_fifo_entries(uint16_t *n,bool *o)
{
    *n=(uint16_t)fifo_words; *o=overrun_requests!=0;
    if(overrun_requests) --overrun_requests;
    return RFID_OK;
}
rfid_status_t sensor_fifo_read(uint8_t *p,uint16_t n)
{
    assert(n==fifo_words*2);
    if(fifo_failures) { --fifo_failures; fifo_words=0; return RFID_IO; }
    for(unsigned i=0;i<n/2;++i) {
        const int16_t values[]={30,24,-333};
        uint16_t w=((uint16_t)values[i%3]&0xfff) | (i%3)<<14;
        p[2*i]=(uint8_t)w; p[2*i+1]=(uint8_t)(w>>8);
    }
    fifo_words=0; return RFID_OK;
}
rfid_status_t radio_begin(const uint8_t *p,size_t n)
{
    assert(!radio_active && n==191);
    memcpy(sent,p,n); ++report_count; radio_active=true; radio_end=now+200;
    assert(sent[182]==report_count%12);
    unsigned slot=(report_count-1)%12, sum=0;
    for(unsigned c=0;c<6;++c) sum+=(sent[38+c*24+slot*2]<<8)|sent[39+c*24+slot*2];
    assert(sum==1200);
    assert(rfid_crc32(sent,187)==((uint32_t)sent[187]<<24|(uint32_t)sent[188]<<16|(uint32_t)sent[189]<<8|sent[190]));
    return RFID_OK;
}
rfid_status_t radio_poll(void)
{
    if((int32_t)(now-radio_end)<0) return RFID_BUSY;
    radio_active=false; return fail_radio?RFID_TIMEOUT:RFID_OK;
}
static unsigned current_sum(void)
{
    rfid_history_t s; app_snapshot(&s); unsigned n=0;
    for(unsigned c=0;c<6;++c) n+=s.current[c];
    return n;
}
static void step(void)
{
    now+=40; ticks+=40; sample_number++; fifo_words+=3;
    if(fifo_words>=450) app_signal(APP_EVENT_FIFO);
    if(now%10000==0) app_signal(APP_EVENT_RTC);
    if(app_events_pending() || !allow_stop || now%1000==0) allow_stop=app_poll();
}
int main(void)
{
    memset(nv,0xff,sizeof(nv));
    nv[0]=0x01020304; nv[1]=0x05060000;
    app_init(); assert(!critical);
    /* 4 h 20 min real cadence simulated, including failed sends. */
    fail_radio=true;
    while(now<15600400) step();
    assert(report_count==13 && !radio_active && watchdogs>5000);
    rfid_history_t s; app_snapshot(&s); assert(s.stage==1);
    for(unsigned w=0;w<12;++w) {
        unsigned sum=0; for(unsigned c=0;c<6;++c) sum+=s.history[c][w];
        assert(sum==1200);
    }
    /* Restart at an arbitrary second recovers the last 60-second checkpoint. */
    while(now<15680400) step();
    unsigned before=current_sum(); app_init(); unsigned restored=current_sum();
    assert(restored<=before && before-restored<=60);
    app_snapshot(&s); assert(s.reset_count==2);
    /* Simultaneous events posted while persisting are not erased by the loop. */
    post_during_write=true;
    while(post_during_write) step();
    assert(app_events_pending());
    (void)app_poll();
    assert(!app_events_pending() && !critical);
    unsigned count=current_sum();
    fifo_words=450; fifo_failures=1; app_signal(APP_EVENT_FIFO); (void)app_poll();
    assert(current_sum()==count);
    now+=10000; ticks+=10000; (void)app_poll();
    fifo_words=512; overrun_requests=1; app_signal(APP_EVENT_FIFO); (void)app_poll();
    assert(current_sum()==count && fifo_words==0);
    assert(nv[0]==0x01020304 && nv[1]==0x05060000 && writes>2000);
    /* An unaligned boot must not postpone a checkpoint until the next FIFO IRQ. */
    app_snapshot(&s);
    uint32_t delta = (60 - s.elapsed % 60) * 1000;
    uint32_t origin = now;
    app_init(); app_snapshot(&s);
    delta = (60 - s.elapsed % 60) * 1000;
    assert(app_wakeup_delay_ms()==10000);
    now = origin + delta - 1;
    assert(app_wakeup_delay_ms()==1);
    now++;
    assert(app_wakeup_delay_ms()==0);
    (void)app_poll();
    assert(app_wakeup_delay_ms()==10000);
    puts("PASS: 4h20 scheduler, 13 exact reports, resets, concurrent events and sensor failures");
    return 0;
}
