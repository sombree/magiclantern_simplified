#include "rp_adaptive_writer.h"
#include <string.h>

#define MIB (1024u * 1024u)

const uint32_t rp_adapt_sizes_bytes[RP_ADAPT_BUCKET_COUNT] = {
     8u * MIB, 12u * MIB, 16u * MIB, 20u * MIB, 24u * MIB,
    28u * MIB, 31u * MIB, 40u * MIB, 48u * MIB, 64u * MIB,
    96u * MIB, 128u * MIB
};

void rp_adapt_reset(struct rp_adapt_writer *st)
{
    if (!st) return;
    memset(st, 0, sizeof(*st));
    st->allowed_index = RP_ADAPT_SAFE_LAST_BUCKET;
    st->preferred_index = 4u; /* initial 24 MiB, not an unverified large FIO */
    st->last_target_bytes = rp_adapt_sizes_bytes[4];
}

static uint32_t rp_adapt_nearest_bucket(uint32_t bytes)
{
    uint32_t closest = 0;
    uint32_t min_distance = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < RP_ADAPT_BUCKET_COUNT; ++i)
    {
        uint32_t size = rp_adapt_sizes_bytes[i];
        uint32_t distance = (size > bytes) ? size - bytes : bytes - size;
        if (distance < min_distance)
        {
            min_distance = distance;
            closest = i;
        }
    }
    return closest;
}

static void rp_adapt_reselect(struct rp_adapt_writer *st)
{
    /* Compare actual end-to-end effective drain, including coalescing wait.
     * Require 3 samples; choose smaller bucket when within 5% of best to
     * avoid spending more time with memory pinned for minimal improvement. */
    uint32_t best_rate = 0;
    uint32_t best = st->preferred_index;
    int found = 0;

    for (uint32_t i = 0; i <= st->allowed_index; ++i)
    {
        if (st->buckets[i].samples < RP_ADAPT_MIN_SAMPLES)
            continue;
        uint32_t rate = st->buckets[i].effective_rate_x100;
        if (!found || rate > best_rate)
        {
            best_rate = rate;
            best = i;
            found = 1;
        }
    }
    if (found)
    {
        for (uint32_t i = 0; i < best; ++i)
        {
            if (st->buckets[i].samples >= RP_ADAPT_MIN_SAMPLES &&
                (uint64_t)st->buckets[i].effective_rate_x100 * 100u >=
                (uint64_t)best_rate * 95u)
            {
                best = i;
                break;
            }
        }
        st->preferred_index = best;
    }
}

uint32_t rp_adapt_target(struct rp_adapt_writer *st, uint32_t free_slots,
                         int draining)
{
    if (!st) return 16u * MIB;
    uint32_t i = st->preferred_index;

    if (i > st->allowed_index) i = st->allowed_index;

    /* Each 8th successful batch probes an adjacent higher/lower bucket.
     * The newest unlocked large size is tried in those probe opportunities.
     * This does not force a full-size allocation or add any memory copies. */
    if (!draining && free_slots >= 12u && st->writes_ok > 0u &&
        (st->writes_ok % 8u) == 0u)
    {
        uint32_t stage = st->allowed_index;
        if (stage > RP_ADAPT_SAFE_LAST_BUCKET &&
            st->buckets[stage].samples < RP_ADAPT_MIN_SAMPLES)
            i = stage;
        else if ((st->probe_attempts & 1u) == 0u &&
                 i + 1u <= st->allowed_index)
            i++;
        else if (i > 0u)
            i--;
        st->probe_attempts++;
    }

    /* Under pressure or STOP, WRITE ready contiguous data immediately.
     * Do not shrink the target to 8 MiB: that would multiply FIO calls and
     * slow the very drain needed to release slots.  Backpressure still caps
     * write duration in the writer; coalescing wait is separately disabled. */
    if (draining || free_slots <= 5u)
        i = st->allowed_index;

    st->last_target_bytes = rp_adapt_sizes_bytes[i];
    return st->last_target_bytes;
}

uint32_t rp_adapt_wait_budget(uint32_t target_bytes, uint32_t have_bytes,
                              uint32_t avg_frame_bytes, uint32_t free_slots,
                              uint32_t fps_x1000, int draining)
{
    if (draining || free_slots < 8u || fps_x1000 == 0u ||
        avg_frame_bytes == 0u || have_bytes >= target_bytes)
        return 0u;

    uint32_t missing = target_bytes - have_bytes;
    uint32_t needed_frames = missing / avg_frame_bytes +
                             ((missing % avg_frame_bytes) != 0u);
    /* Do not pin already-ready frames while waiting for more than our
     * remaining FREE capacity can reasonably sustain. */
    if (needed_frames + 5u >= free_slots)
        return 0u;

    uint64_t interval_us = 1000000000ull / fps_x1000;
    uint64_t predicted_us = (uint64_t)needed_frames * interval_us;
    /* Bounded waiting.  Avoid spending 600 ms chasing a goal predicted
     * multiple seconds away.  Permit near-future frame arrival with jitter. */
    if (predicted_us > (uint64_t)RP_ADAPT_MAX_WAIT_US * 2ull)
        return 0u;
    uint64_t wait_us = predicted_us + interval_us / 4u;
    if (wait_us > RP_ADAPT_MAX_WAIT_US)
        wait_us = RP_ADAPT_MAX_WAIT_US;
    return (uint32_t)wait_us;
}

void rp_adapt_observe(struct rp_adapt_writer *st, uint32_t bytes,
                      uint32_t fio_us, uint32_t coalesce_us, int success)
{
    if (!st || !success || !bytes || !fio_us)
        return;

    uint64_t full_us = (uint64_t)fio_us + coalesce_us;
    uint32_t rate = (uint32_t)(((uint64_t)bytes * 100ull) / fio_us);
    uint32_t effective = (uint32_t)(((uint64_t)bytes * 100ull) / full_us);
    st->last_write_rate_x100 = rate;
    st->last_effective_rate_x100 = effective;
    st->writes_ok++;
    if (bytes > st->peak_success_bytes) st->peak_success_bytes = bytes;

    /* Batches smaller than 6 MiB are not representative of our lowest
     * size bucket; never misclassify a one-frame 2 MiB write as 8 MiB. */
    if (bytes >= 6u * MIB)
    {
        uint32_t bin = rp_adapt_nearest_bucket(bytes);
        struct rp_adapt_bucket *b = &st->buckets[bin];
        if (!b->samples)
        {
            b->write_rate_x100 = rate;
            b->effective_rate_x100 = effective;
        }
        else
        {
            b->write_rate_x100 =
                (uint32_t)(((uint64_t)b->write_rate_x100 * 7u + rate) / 8u);
            b->effective_rate_x100 =
                (uint32_t)(((uint64_t)b->effective_rate_x100 * 7u + effective) / 8u);
        }
        b->samples++;
    }

    /* Enable >31 MiB only after a stable run of successful writes.
     * Escalate 40 -> 48 -> 64 -> 96 -> 128.  If frame size granularity
     * prevents approaching a ceiling (e.g. 20+20 cannot fit a 31 MiB cap),
     * advance after 12 further successful writes. These staged trials are
     * still experimental: success at one size does not prove the next. */
    if (st->allowed_index == RP_ADAPT_SAFE_LAST_BUCKET)
    {
        if (st->writes_ok >= 12u && st->peak_success_bytes >= 8u * MIB)
        {
            st->allowed_index++;
            st->high_successes = 0;
            st->stage_successes = 0;
        }
    }
    else if (st->allowed_index + 1u < RP_ADAPT_BUCKET_COUNT)
    {
        uint32_t floor_bytes =
            (rp_adapt_sizes_bytes[st->allowed_index] / 4u) * 3u;
        st->stage_successes++;
        if (bytes >= floor_bytes)
            st->high_successes++;
        if (st->high_successes >= 2u || st->stage_successes >= 12u)
        {
            st->allowed_index++;
            st->high_successes = 0;
            st->stage_successes = 0;
        }
    }

    if (st->writes_ok % 6u == 0u)
        rp_adapt_reselect(st);
}
