#include "rp_pressure_v112.h"
#include <limits.h>

static uint32_t sat_u32(uint64_t x) { return x > UINT32_MAX ? UINT32_MAX : (uint32_t)x; }

void rp_v112_pressure_decide(const struct rp_v112_pressure *in,
                             struct rp_v112_decision *out)
{
    if (!out) return;
    out->level = RP_V112_PRESSURE_NORMAL;
    out->overflow_us = UINT32_MAX;
    out->deficit_x100 = 0;
    out->wait_cap_us = 200000u;
    out->max_write_us = UINT32_MAX;
    if (!in) return;

    /* If the measured drain exceeds ingress we don't invent a future
     * overflow.  Under-sampled drains are deliberately treated separately. */
    if (in->ingress_x100 && in->drain_x100 &&
        in->ingress_x100 > in->drain_x100)
    {
        uint32_t deficit = in->ingress_x100 - in->drain_x100;
        out->deficit_x100 = deficit;
        /* free_bytes / (deficit_x100 / 100 bytes/us) */
        out->overflow_us = sat_u32(((uint64_t)in->free_bytes * 100u) / deficit);
    }
    /* Before both rates are available we deliberately leave the horizon
     * unknown. free_slots thresholds below still protect the writer, but
     * unlike the old free_slots/fps estimate this does NOT misrepresent a
     * large, initially empty arena as about to overflow. */

    if (in->free_slots <= 2u || in->free_bytes == 0u ||
        out->overflow_us <= 400000u)
        out->level = RP_V112_PRESSURE_CRITICAL;
    else if (in->free_slots <= 5u || out->overflow_us <= 1500000u)
        out->level = RP_V112_PRESSURE_HIGH;
    else if (in->free_slots <= 8u || out->overflow_us <= 3500000u)
        out->level = RP_V112_PRESSURE_WATCH;

    if (out->level == RP_V112_PRESSURE_WATCH)
        out->wait_cap_us = 30000u;
    if (out->level >= RP_V112_PRESSURE_HIGH || in->draining)
        out->wait_cap_us = 0u;

    if (out->overflow_us != UINT32_MAX)
        out->max_write_us = sat_u32(((uint64_t)out->overflow_us * 85u) / 100u);
}

uint32_t rp_v112_pressure_wait(const struct rp_v112_decision *d,
                               uint32_t old_wait_us)
{
    return (d && old_wait_us > d->wait_cap_us) ? d->wait_cap_us : old_wait_us;
}

uint32_t rp_v112_pressure_limit_frames(const struct rp_v112_decision *d,
                                       uint32_t frames, uint32_t legacy_limit,
                                       uint32_t write_bytes)
{
    if (!frames) return 0u;
    if (!legacy_limit || legacy_limit > frames) legacy_limit = frames;
    if (!d) return legacy_limit;
    /* During high pressure, splitting one already-contiguous batch into
     * smaller FIO calls usually increases per-call overhead and cannot free
     * any slots until FIO returns. Preserve the WHOLE ready batch within the
     * separately staged adaptive size cap. Never synthesize a larger batch. */
    if (d->level >= RP_V112_PRESSURE_HIGH && write_bytes > 0u)
        return frames;
    /* When byte-based overflow horizon is distant, avoid shrinking a good
     * contiguous batch solely because free_slots / fps looked pessimistic. */
    if (d->overflow_us > 2500000u && d->deficit_x100 > 0u)
        return frames;
    return legacy_limit;
}
