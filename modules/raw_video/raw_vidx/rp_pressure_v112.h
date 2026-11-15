#ifndef RP_PRESSURE_V112_H
#define RP_PRESSURE_V112_H
#include <stdint.h>
/* Pure decision layer. Rates are ML's bytes*100 / microsecond (MB/s x100).
 * Does not own, allocate, move, or free any Canon memory. */
enum rp_v112_pressure_level {
    RP_V112_PRESSURE_NORMAL = 0,
    RP_V112_PRESSURE_WATCH = 1,
    RP_V112_PRESSURE_HIGH = 2,
    RP_V112_PRESSURE_CRITICAL = 3
};
struct rp_v112_pressure {
    uint32_t free_bytes;     /* currently allocatable FREE regions, not RESERVED */
    uint32_t free_slots;
    uint32_t ingress_x100;
    uint32_t drain_x100;
    uint32_t writer_x100;
    uint32_t fps_x1000;
    uint32_t draining;
};
struct rp_v112_decision {
    uint32_t level;
    uint32_t overflow_us;   /* UINT32_MAX: not predicted with current rates */
    uint32_t deficit_x100;
    uint32_t wait_cap_us;
    uint32_t max_write_us;  /* advisory only; does not change hardware timeout */
};
void rp_v112_pressure_decide(const struct rp_v112_pressure *in,
                             struct rp_v112_decision *out);
uint32_t rp_v112_pressure_wait(const struct rp_v112_decision *d,
                               uint32_t old_wait_us);
uint32_t rp_v112_pressure_limit_frames(const struct rp_v112_decision *d,
                                       uint32_t frames, uint32_t legacy_limit,
                                       uint32_t write_bytes);
#endif
