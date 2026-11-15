#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "rp_adaptive_writer.h"
#define MiB (1024u * 1024u)

static void success(struct rp_adapt_writer *s, unsigned mib, unsigned write_us, unsigned wait_us)
{
    rp_adapt_observe(s, mib * MiB, write_us, wait_us, 1);
}
int main(void)
{
    struct rp_adapt_writer s;
    rp_adapt_reset(&s);
    assert(s.allowed_index == 6u);
    assert(rp_adapt_target(&s, 32, 0) == 24 * MiB);
    assert(rp_adapt_target(&s, 4, 0) == 31 * MiB);
    assert(rp_adapt_target(&s, 32, 1) == 31 * MiB);
    assert(rp_adapt_wait_budget(48*MiB, 8*MiB, 4*MiB, 12, 24000, 0) == 0);
    assert(rp_adapt_wait_budget(24*MiB, 20*MiB, 4*MiB, 20, 24000, 0) > 0);
    assert(rp_adapt_wait_budget(24*MiB, 20*MiB, 4*MiB, 20, 24000, 1) == 0);
    for (int i=0;i<11;i++) success(&s,24,160000,0);
    assert(s.allowed_index == 6);
    rp_adapt_observe(&s,24*MiB,160000,0,0); /* never unlock on failure */
    assert(s.allowed_index == 6);
    success(&s,24,160000,0);
    assert(s.allowed_index == 7);
    assert(s.last_target_bytes <= 31*MiB);
    for (int i=0;i<2;i++) success(&s,40,300000,0);
    assert(s.allowed_index == 8);
    for (int i=0;i<2;i++) success(&s,48,360000,0);
    assert(s.allowed_index == 9);
    for (int i=0;i<2;i++) success(&s,64,450000,0);
    assert(s.allowed_index == 10);
    for (int i=0;i<2;i++) success(&s,96,760000,0);
    assert(s.allowed_index == 11);
    success(&s,128, 1000000, 10000);
    assert(s.buckets[11].samples == 1);
    assert(s.last_write_rate_x100 > 0);
    assert(s.last_effective_rate_x100 <= s.last_write_rate_x100);
    rp_adapt_reset(&s);
    assert(s.allowed_index == 6);
    assert(s.writes_ok == 0);
    puts("PASS: initial cap, STOP/pressure, wait budget, staged 31->40->48->64->96->128, reset, telemetry");
    return 0;
}
