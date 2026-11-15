#include "rp_pressure_v113.h"
#include <limits.h>

#define RP_V113_FALLBACK_IO_RATE 5000u /* 50.00 decimal MB/s, startup only */
#define RP_V113_IO_OVERHEAD_US 20000u
#define RP_V113_HEADROOM_PERCENT 70u
#define RP_V113_IO_MARGIN_PERCENT 115u

static uint32_t rp_v113_sat_u32(uint64_t value)
{
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}
static uint32_t rp_v113_min(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

void rp_v113_history_reset(struct rp_v113_history *h)
{
    if (h) *h = (struct rp_v113_history){0};
}

void rp_v113_history_observe(struct rp_v113_history *h,
                             uint32_t bytes, uint32_t fio_us, int success)
{
    if (!h || !success || !bytes || !fio_us) return;
    uint32_t rate = rp_v113_sat_u32(((uint64_t)bytes * 100u) / fio_us);
    if (!rate) rate = 1;
    h->rate_x100[h->cursor] = rate;
    h->cursor = (h->cursor + 1u) % RP_V113_IO_HISTORY;
    if (h->count < RP_V113_IO_HISTORY) h->count++;
}

/* Lower quartile of recent FIO throughput. Unlike a lifetime minimum, an
 * early slow write ages out; unlike an EWMA, temporary slow writes matter. */
static uint32_t rp_v113_io_rate(const struct rp_v113_input *in)
{
    uint32_t rate = in->fallback_rate_x100;
    if (!rate) rate = RP_V113_FALLBACK_IO_RATE;
    const struct rp_v113_history *h = in->history;
    if (!h || !h->count) return rate;
    uint32_t sorted[RP_V113_IO_HISTORY];
    uint32_t n = rp_v113_min(h->count, RP_V113_IO_HISTORY);
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t v = h->rate_x100[i];
        if (!v) return rate;
        uint32_t j = i;
        while (j > 0 && sorted[j - 1u] > v)
        {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = v;
    }
    uint32_t conservative = sorted[(n - 1u) / 4u];
    return conservative ? conservative : rate;
}

static uint32_t rp_v113_estimate_write_us(uint32_t bytes, uint32_t rate_x100)
{
    if (!bytes || !rate_x100) return UINT32_MAX;
    uint64_t base = ((uint64_t)bytes * 100u + rate_x100 - 1u) / rate_x100;
    base += RP_V113_IO_OVERHEAD_US;
    base = (base * RP_V113_IO_MARGIN_PERCENT + 99u) / 100u;
    return rp_v113_sat_u32(base);
}

void rp_v113_select_prefix(const struct rp_v113_input *in,
                           const uint32_t *slot_bytes, uint32_t max_frames,
                           struct rp_v113_choice *out)
{
    if (!out) return;
    *out = (struct rp_v113_choice){0};
    out->budget_us = UINT32_MAX;
    out->no_release_us = UINT32_MAX;
    if (!in || !slot_bytes || !max_frames) return;

    out->conservative_rate_x100 = rp_v113_io_rate(in);
    /* During STOP there is no new producer traffic; preserve big contiguous
     * batches for a fast drain. Do not mistake draining=1 for low pressure. */
    if (!in->draining)
    {
        if (in->ingress_x100)
            out->no_release_us = rp_v113_min(out->no_release_us,
                rp_v113_sat_u32(((uint64_t)in->free_bytes * 100u) / in->ingress_x100));
        if (in->fps_x1000)
            out->no_release_us = rp_v113_min(out->no_release_us,
                rp_v113_sat_u32(((uint64_t)in->free_slots * 1000000000ull) / in->fps_x1000));
        /* Enforce the blocking-write deadline only at HIGH/CRITICAL pressure.
         * WATCH already suppresses long coalescing waits in v112; splitting
         * WRITE batches too early would pay unnecessary per-FIO overhead. */
        if (in->pressure && in->pressure->level >= RP_V112_PRESSURE_HIGH)
        {
            if (out->no_release_us != UINT32_MAX)
                out->budget_us = rp_v113_sat_u32(
                    (uint64_t)out->no_release_us * RP_V113_HEADROOM_PERCENT / 100u);
            if (in->pressure->max_write_us != UINT32_MAX)
                out->budget_us = rp_v113_min(out->budget_us,
                                               in->pressure->max_write_us);
        }
    }

    uint32_t cumulative = 0u;
    for (uint32_t i = 0; i < max_frames; i++)
    {
        if (!slot_bytes[i] || slot_bytes[i] > UINT32_MAX - cumulative)
            break;
        uint32_t next_bytes = cumulative + slot_bytes[i];
        uint32_t predicted = rp_v113_estimate_write_us(
            next_bytes, out->conservative_rate_x100);
        if (out->frames && predicted > out->budget_us)
            break;
        cumulative = next_bytes;
        out->frames = i + 1u;
        out->bytes = cumulative;
        out->predicted_us = predicted;
        if (predicted > out->budget_us)
        {
            out->budget_exceeded = 1u;
            break; /* One complete VIDF is indivisible. */
        }
    }
}
