#include "storage.h"
#include "protocol.h"
#include "history.h"
#include "fifo_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
void legacy_encode(uint8_t out[191],const uint8_t id[6],const uint8_t random[32],const rfid_history_t *s,uint16_t battery);
static uint32_t nv[512], saved[512];
static int budget = -1, torn;
static uint32_t read_word(void *ctx, uint16_t o) { (void)ctx; assert(!(o&3) && o<2048); return nv[o/4]; }
static rfid_status_t write_word(void *ctx, uint16_t o, uint32_t v)
{
    (void)ctx; assert(o>=0x240 && o<0x800 && !(o&3));
    if (budget == 0) {
        if (torn) nv[o/4] = (nv[o/4]&0xffff0000U) | (v&0xffffU);
        return RFID_IO;
    }
    if (budget>0) --budget;
    nv[o/4]=v; return RFID_OK;
}
static storage_t fresh(void) { storage_t s={0}; s.read_word=read_word; s.write_word=write_word; return s; }
static void legacy(void)
{
    memset(nv,0xff,sizeof(nv)); budget=-1;
    nv[0]=0x01020304; nv[1]=0x05060000; nv[2]=3; nv[3]=7; nv[4]=42;
    const unsigned bases[]={0x100,0x130,0x160,0x190,0x1c0,0x200};
    for(unsigned c=0;c<6;++c) for(unsigned w=0;w<12;++w) nv[(bases[c]/4)+w]=c*30+w;
}
static void storage_tests(void)
{
    rfid_history_t a,b;
    legacy(); memcpy(saved,nv,sizeof(nv));
    /* Every word boundary of the first import, including torn writes. */
    for (torn=0;torn<2;++torn) for(int cut=0;cut<=117;++cut) {
        memcpy(nv,saved,sizeof(nv)); budget=cut; storage_t s=fresh();
        (void)storage_restore(&s,&a);
        budget=-1; s=fresh(); assert(storage_restore(&s,&b)==RFID_OK);
        assert(b.stage==3 && b.elapsed==420 && b.reset_count==7);
        for(unsigned c=0;c<6;++c) {
            assert(b.current[c]==c*30+3);
            for(unsigned w=0;w<12;++w) assert(b.history[c][w]==c*30+w);
        }
        assert(!memcmp(nv,saved,0x240));
    }
    torn=0; legacy(); storage_t s=fresh(); assert(storage_restore(&s,&a)==RFID_OK);
    memcpy(saved,nv,sizeof(nv));
    for (torn=0;torn<2;++torn) for(int complete=0;complete<2;++complete) for(int cut=0;cut<=9;++cut) {
        memcpy(nv,saved,sizeof(nv)); budget=-1; s=fresh(); assert(storage_restore(&s,&a)==RFID_OK);
        a.current[0]=999; a.elapsed=complete?1200:480; budget=cut;
        rfid_status_t status=complete?storage_complete(&s,&a):storage_checkpoint(&s,&a);
        budget=-1; s=fresh(); assert(storage_restore(&s,&b)==RFID_OK);
        if(status==RFID_OK) {
            assert(b.stage==(complete?4:3)); assert(b.elapsed==(complete?0:480));
            assert((complete?b.history[0][3]:b.current[0])==999);
        } else { assert(b.stage==3 && b.elapsed==420 && b.current[0]==3); }
        assert(!memcmp(nv,saved,0x240));
    }
    torn=0; legacy(); s=fresh(); assert(storage_restore(&s,&a)==RFID_OK);
    for(unsigned w=0;w<40;++w) {
        unsigned stage=a.stage;
        for(unsigned c=0;c<6;++c) a.current[c]=w*10+c;
        a.elapsed=1200; assert(storage_complete(&s,&a)==RFID_OK);
        history_close(&a);
        for(unsigned t=60;t<1200;t+=60) { a.elapsed=t; assert(storage_checkpoint(&s,&a)==RFID_OK); }
        storage_t reboot=fresh(); assert(storage_restore(&reboot,&b)==RFID_OK);
        assert(b.history[0][stage]==w*10); assert(!memcmp(&a,&b,sizeof(a)));
    }
    /* Sequence rollover, CRC rejection and no mutation of identity. */
    memset(nv+0x240/4,0xff,2048-0x240); s=fresh(); s.sequence=UINT32_MAX-2;
    memset(&a,0,sizeof(a));
    for(unsigned i=0;i<6;++i) { a.elapsed=i*60; assert(storage_checkpoint(&s,&a)==RFID_OK); }
    storage_t reboot=fresh(); assert(storage_restore(&reboot,&b)==RFID_OK && b.elapsed==300);
    nv[(0x240+16*32+5*32+4)/4]^=1;
    reboot=fresh(); assert(storage_restore(&reboot,&b)==RFID_OK && b.elapsed==240);
    /* Unknown old windows are discarded as a whole; identity is never rewritten. */
    for(unsigned fault=0;fault<4;++fault) {
        legacy();
        if(fault==0) nv[4]=252;
        if(fault==1) nv[2]=UINT32_MAX;
        if(fault==2) nv[0x160/4+3]=2832;
        if(fault==3) { nv[0x100/4+3]=1000; nv[0x130/4+3]=1000; }
        memcpy(saved,nv,sizeof(nv)); s=fresh();
        assert(storage_restore(&s,&b)==RFID_OK && b.elapsed==0);
        for(unsigned c=0;c<6;++c) {
            assert(b.current[c]==0);
            if(fault>=2) assert(b.history[c][3]==0);
        }
        assert(!memcmp(nv,saved,0x240));
    }
}
static void protocol_tests(void)
{
    uint8_t id[]={1,2,3,4,5,6}, random[32], out[193];
    rfid_history_t s={0};
    for(unsigned i=0;i<32;++i) random[i]=(uint8_t)(i*7);
    for(unsigned c=0;c<6;++c) for(unsigned w=0;w<12;++w) s.history[c][w]=c*256+w;
    s.stage=11; s.reset_count=0x1234;
    memset(out,0xa5,sizeof(out));
    assert(protocol_encode(out+1,191,id,random,&s,0x4567)==RFID_OK);
    uint8_t expected[191];
    legacy_encode(expected,id,random,&s,0x4567);
    assert(!memcmp(expected,out+1,191));
    assert(out[0]==0xa5 && out[192]==0xa5);
    const uint8_t *p=out+1;
    assert(!memcmp(p,id,6) && !memcmp(p+6,random,32));
    for(unsigned c=0;c<6;++c) for(unsigned w=0;w<12;++w) {
        assert(p[38+c*24+w*2]==c && p[39+c*24+w*2]==w);
    }
    assert(p[182]==11 && p[183]==0x45 && p[184]==0x67 && p[185]==0x12 && p[186]==0x34);
    uint32_t crc=rfid_crc32(p,187);
    for(unsigned i=0;i<4;++i) assert(p[187+i]==(uint8_t)(crc>>(24-i*8)));
    assert(rfid_crc32((const uint8_t*)"123456789",9)==0xcbf43926U);
    assert(protocol_encode(out,190,id,random,&s,0)==RFID_INVALID);
}
static void fifo_tests(void)
{
    fifo_parser_t p; accel_sample_t v;
    fifo_parser_reset(&p);
    assert(!fifo_parser_word(&p,0x4001,&v));
    assert(!fifo_parser_word(&p,0x8002,&v));
    assert(!fifo_parser_word(&p,0x0800,&v));
    assert(!fifo_parser_word(&p,0x47ff,&v));
    assert(fifo_parser_word(&p,0x8fff,&v));
    assert(v.x==-2048 && v.y==2047 && v.z==-1);
    assert(!fifo_parser_word(&p,0x0001,&v));
    assert(!fifo_parser_word(&p,0xc000,&v));
    assert(!fifo_parser_word(&p,0x4002,&v));
    assert(!fifo_parser_word(&p,0x8003,&v));
    assert(p.discarded==5);
}
int main(void) { storage_tests(); protocol_tests(); fifo_tests(); puts("PASS: journal fault injection, protocol, FIFO"); return 0; }
