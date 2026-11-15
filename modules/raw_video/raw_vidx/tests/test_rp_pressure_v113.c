#include "../rp_pressure_v113.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#define MIB (1024u*1024u)
static struct rp_v112_decision p(uint32_t level)
{
    struct rp_v112_decision d;
    memset(&d, 0, sizeof(d));
    d.level=level; d.max_write_us=UINT32_MAX; return d;
}
static struct rp_v113_input fixture(const struct rp_v112_decision *d,
                                     struct rp_v113_history *h)
{
    struct rp_v113_input i;
    memset(&i,0,sizeof(i));
    i.free_bytes=40*MIB; i.free_slots=6; i.ingress_x100=11000;
    i.fps_x1000=24000; i.fallback_rate_x100=7500;
    i.pressure=d; i.history=h; return i;
}
int main(void)
{
    struct rp_v113_history h;
    struct rp_v113_choice o;
    uint32_t slots[16];
    for(unsigned k=0;k<16;k++) slots[k]=4*MIB;
    rp_v113_history_reset(&h);
    assert(h.count==0 && h.cursor==0);
    struct rp_v112_decision d=p(RP_V112_PRESSURE_NORMAL);
    struct rp_v113_input i=fixture(&d,&h);
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==9 && o.budget_us==UINT32_MAX);

    d=p(RP_V112_PRESSURE_WATCH); i.pressure=&d;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==9 && o.budget_us==UINT32_MAX);

    d=p(RP_V112_PRESSURE_HIGH); i.pressure=&d;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames>=1 && o.frames<9 && o.no_release_us!=UINT32_MAX);
    assert(o.bytes==o.frames*4*MIB);
    assert(o.predicted_us<=o.budget_us || (o.frames==1 && o.budget_exceeded));
    printf("HIGH: %u frames, %u us budget, %u us prediction\n",
           o.frames,o.budget_us,o.predicted_us);

    i.draining=1;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==9 && o.budget_us==UINT32_MAX);
    i.draining=0;

    i.free_slots=3;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames <= 2);

    i.free_slots=0; i.free_bytes=0;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==1 && o.budget_exceeded && o.budget_us==0);

    i.free_slots=8; i.free_bytes=40*MIB;
    i.ingress_x100=0; i.fps_x1000=0;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==9 && o.no_release_us==UINT32_MAX);

    i.ingress_x100=11000; i.fps_x1000=24000;
    d.max_write_us=95000; i.pressure=&d;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.budget_us==95000 && o.frames==1);
    d.max_write_us=UINT32_MAX;

    slots[0]=0;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==0);
    slots[0]=4*MIB; slots[1]=0;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==1);
    slots[1]=4*MIB;

    rp_v113_history_observe(&h,16*MIB,300000,0);
    assert(h.count==0);
    rp_v113_history_observe(&h,16*MIB,300000,1);
    assert(h.count==1);
    rp_v113_history_observe(&h,16*MIB,150000,1);
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.conservative_rate_x100>0);
    uint32_t prev_rate=o.conservative_rate_x100;
    for(unsigned j=0;j<RP_V113_IO_HISTORY+3u;j++)
        rp_v113_history_observe(&h,16*MIB,160000,1);
    assert(h.count==RP_V113_IO_HISTORY && h.cursor<RP_V113_IO_HISTORY);
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.conservative_rate_x100>prev_rate);

    i.free_slots=UINT32_MAX; i.free_bytes=UINT32_MAX;
    i.ingress_x100=1; i.fps_x1000=1;
    rp_v113_select_prefix(&i,slots,9,&o);
    assert(o.frames==9);

    i.draining=1;
    slots[0]=UINT32_MAX; slots[1]=4*MIB;
    rp_v113_select_prefix(&i,slots,2,&o);
    assert(o.frames==1 && o.bytes==UINT32_MAX);

    rp_v113_select_prefix(NULL,slots,2,&o);
    assert(o.frames==0);
    rp_v113_select_prefix(&i,NULL,2,&o);
    assert(o.frames==0);
    rp_v113_select_prefix(&i,slots,0,&o);
    assert(o.frames==0);
    rp_v113_select_prefix(&i,slots,2,NULL);
    rp_v113_history_reset(NULL);
    rp_v113_history_observe(NULL,1,1,1);
    puts("PASS: v113 pressure horizon / synchronous FIO / drain / history / edge cases");
    return 0;
}
