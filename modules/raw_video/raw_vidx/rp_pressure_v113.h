#ifndef RP_PRESSURE_V113_H
#define RP_PRESSURE_V113_H
#include <stdint.h>
#include "rp_pressure_v112.h"

/* Pure writer decision; no MMIO, allocator, FIO calls or ownership changes.
 * Rates: decimal MB/s * 100 == bytes * 100 / microseconds.
 * The decision cannot guarantee no starve: Canon/FIO scheduling is variable. */
#define RP_V113_IO_HISTORY 8u
struct rp_v113_history {
    uint32_t rate_x100[RP_V113_IO_HISTORY];
    uint32_t count;
    uint32_t cursor;
};
struct rp_v113_input {
    uint32_t free_bytes;
    uint32_t free_slots;
    uint32_t ingress_x100;
    uint32_t fps_x1000;
    uint32_t fallback_rate_x100;
    uint32_t draining;
    const struct rp_v112_decision *pressure;
    const struct rp_v113_history *history;
};
struct rp_v113_choice {
    uint32_t frames;
    uint32_t bytes;
    uint32_t predicted_us;
    uint32_t budget_us;
    uint32_t no_release_us;
    uint32_t conservative_rate_x100;
    uint32_t budget_exceeded; /* Even one VIDF cannot fit budget. */
};
void rp_v113_history_reset(struct rp_v113_history *h);
void rp_v113_history_observe(struct rp_v113_history *h,
                             uint32_t bytes, uint32_t fio_us, int success);
void rp_v113_select_prefix(const struct rp_v113_input *in,
                           const uint32_t *slot_bytes, uint32_t max_frames,
                           struct rp_v113_choice *out);
#endif
