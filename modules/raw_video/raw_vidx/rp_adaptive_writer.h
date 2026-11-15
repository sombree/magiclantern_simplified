#ifndef RP_ADAPTIVE_WRITER_H
#define RP_ADAPTIVE_WRITER_H

#include <stdint.h>

/* On-camera experimental FIO request cap, NOT a confirmed DIGIC 8 DMA cap.
 * Writes > legacy ~32 MiB are explicitly experimental and are enabled through
 * a staged, successful-write-only ramp.  Never retry a failed/short FIO write:
 * file position may already have advanced. */
#define RP_ADAPT_MAX_BATCH_BYTES (128u * 1024u * 1024u)
#define RP_ADAPT_BUCKET_COUNT 12u
#define RP_ADAPT_SAFE_LAST_BUCKET 6u /* 31 MiB */
#define RP_ADAPT_MIN_SAMPLES 3u
#define RP_ADAPT_MAX_WAIT_US 600000u

struct rp_adapt_bucket
{
    uint32_t samples;
    uint32_t write_rate_x100;     /* EWMA bytes * 100 / FIO microseconds */
    uint32_t effective_rate_x100; /* EWMA bytes * 100 / (FIO + coalesce wait) */
};

struct rp_adapt_writer
{
    struct rp_adapt_bucket buckets[RP_ADAPT_BUCKET_COUNT];
    uint32_t writes_ok;
    uint32_t peak_success_bytes;
    uint32_t allowed_index;       /* staged upper limit for 48/64/96/128 MiB */
    uint32_t preferred_index;
    uint32_t high_successes;      /* successes near current allowed ceiling */
    uint32_t stage_successes;     /* fallback progression for coarse frame sizes */
    uint32_t probe_attempts;
    uint32_t last_target_bytes;
    uint32_t last_effective_rate_x100;
    uint32_t last_write_rate_x100;
};

extern const uint32_t rp_adapt_sizes_bytes[RP_ADAPT_BUCKET_COUNT];
void rp_adapt_reset(struct rp_adapt_writer *st);
uint32_t rp_adapt_target(struct rp_adapt_writer *st, uint32_t free_slots,
                         int draining);
uint32_t rp_adapt_wait_budget(uint32_t target_bytes, uint32_t have_bytes,
                              uint32_t avg_frame_bytes, uint32_t free_slots,
                              uint32_t fps_x1000, int draining);
void rp_adapt_observe(struct rp_adapt_writer *st, uint32_t bytes,
                      uint32_t fio_us, uint32_t coalesce_us, int success);

#endif
