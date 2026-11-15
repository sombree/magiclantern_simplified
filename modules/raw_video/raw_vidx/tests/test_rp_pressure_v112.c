#include "rp_pressure_v112.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

static struct rp_v112_decision decide(unsigned bytes, unsigned slots,
                                     unsigned ingress, unsigned drain,
                                     unsigned stop)
{
    struct rp_v112_pressure in = {bytes,slots,ingress,drain,7300u,25000u,stop};
    struct rp_v112_decision d;
    rp_v112_pressure_decide(&in,&d);
    return d;
}
int main(void)
{
    const unsigned MiB=1024u*1024u;
    struct rp_v112_decision d;
    d=decide(160u*MiB,12,0,0,0);
    assert(d.level==RP_V112_PRESSURE_NORMAL && d.overflow_us==UINT32_MAX);
    d=decide(100u*MiB,12,11000,7300,0);
    assert(d.level==RP_V112_PRESSURE_WATCH);
    assert(d.overflow_us > 2500000u && d.overflow_us < 3000000u);
    assert(rp_v112_pressure_wait(&d,500000u)==30000u);
    assert(rp_v112_pressure_limit_frames(&d,5,2,22u*MiB)==5u);
    d=decide(32u*MiB,6,11000,7300,0);
    assert(d.level==RP_V112_PRESSURE_HIGH);
    assert(rp_v112_pressure_wait(&d,500000u)==0u);
    assert(rp_v112_pressure_limit_frames(&d,5,1,22u*MiB)==5u);
    d=decide(0,0,11000,7300,0);
    assert(d.level==RP_V112_PRESSURE_CRITICAL && d.overflow_us==0);
    d=decide(100u*MiB,12,6000,7300,0);
    assert(d.level==RP_V112_PRESSURE_NORMAL && d.overflow_us==UINT32_MAX);
    assert(rp_v112_pressure_limit_frames(&d,5,3,22u*MiB)==3u);
    d=decide(100u*MiB,12,11000,7300,1);
    assert(d.wait_cap_us==0);
    d=decide(UINT32_MAX,1000,UINT32_MAX,1,0);
    assert(d.overflow_us < UINT32_MAX);
    assert(rp_v112_pressure_limit_frames(NULL,5,2,10u*MiB)==2u);
    assert(rp_v112_pressure_limit_frames(&d,0,0,0)==0u);
    puts("PASS: v112 unknown rates, byte horizon, high pressure, full batch, stop, overflow");
    return 0;
}
