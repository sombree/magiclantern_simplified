#ifndef RP_W3_TRANSPORT_H
#define RP_W3_TRANSPORT_H

#include <stdint.h>

/* Geometry is intentionally separate from sensor sample precision.
 * Model only; creating/reading this plan NEVER writes W3 MMIO, CPU registers,
 * Canon Mem1 config, or raw_info. In particular 16-bit transport is NOT
 * safe to enable until the destination is exclusively ML-owned. */
struct rp_w3_transport_plan {
    uint32_t width, height;
    uint32_t sample_bpp;         /* Canon sensor/PU input depth: 10, 12, 14 */
    uint32_t wire_bpp;           /* W3 serialized container: sample_bpp or 16 */
    uint32_t nominal_pitch, nominal_bytes; /* packed payload currently emitted */
    uint32_t target_pitch, target_bytes;   /* full physical payload if enabled */
    uint32_t observed_pitch;     /* currently programmed W3 descriptor pitch */
    uint32_t source_capacity;    /* usable bytes per redirect ring slot, 0 unknown */
    uint32_t le16;               /* b0=1: LE16 serialization model */
    uint32_t hw_full_width;      /* observational only: descriptor matches target */
    uint32_t buffer_full_width;  /* model allocation capacity matches target */
};

int rp_w3_transport_make_plan(struct rp_w3_transport_plan *p,
                              uint32_t width, uint32_t height,
                              uint32_t sample_bpp, int le16,
                              uint32_t observed_pitch, uint32_t capacity);

#endif
