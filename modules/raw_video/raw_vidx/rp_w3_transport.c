#include "rp_w3_transport.h"
#include <string.h>

int rp_w3_transport_make_plan(struct rp_w3_transport_plan *p,
                              uint32_t width, uint32_t height,
                              uint32_t sample_bpp, int le16,
                              uint32_t observed_pitch, uint32_t capacity)
{
    if (!p) return -1;
    memset(p, 0, sizeof(*p));
    if (!width || !height ||
        (sample_bpp != 10u && sample_bpp != 12u && sample_bpp != 14u))
        return -2;
    if (le16 != 0 && le16 != 1) return -3;
    const uint32_t wire_bpp = le16 ? 16u : sample_bpp;
    const uint64_t nominal_bits = (uint64_t)width * sample_bpp;
    const uint64_t target_bits = (uint64_t)width * wire_bpp;
    if ((nominal_bits & 7u) || (target_bits & 7u)) return -4;
    const uint64_t nominal_pitch = nominal_bits / 8u;
    const uint64_t target_pitch = target_bits / 8u;
    const uint64_t nominal_bytes = nominal_pitch * height;
    const uint64_t target_bytes = target_pitch * height;
    /* Low 16 bits in W3+0x50 represent the row stride. Reject overflow.
     * Using 32-bit arithmetic here could silently under-allocate buffers. */
    if (nominal_pitch > 0xffffu || target_pitch > 0xffffu ||
        nominal_bytes > 0x7fffffffu || target_bytes > 0x7fffffffu)
        return -5;
    p->width = width;
    p->height = height;
    p->sample_bpp = sample_bpp;
    p->wire_bpp = wire_bpp;
    p->nominal_pitch = (uint32_t)nominal_pitch;
    p->nominal_bytes = (uint32_t)nominal_bytes;
    p->target_pitch = (uint32_t)target_pitch;
    p->target_bytes = (uint32_t)target_bytes;
    p->observed_pitch = observed_pitch;
    p->source_capacity = capacity;
    p->le16 = (uint32_t)le16;
    p->hw_full_width = observed_pitch == p->target_pitch;
    p->buffer_full_width = capacity >= p->target_bytes;
    return 0;
}
