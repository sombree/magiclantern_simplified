#ifndef RAW_VIDX_MLV_FPS_RATIONAL_H
#define RAW_VIDX_MLV_FPS_RATIONAL_H

#include <stdint.h>

/* The recorder supplies fps * 1000 (e.g. 23976 for 24000/1001).
 * Restore exact NTSC-style fractions only when the supplied integer is
 * precisely the truncated value of N*1000/1001.  Preserve all other rates,
 * including deliberately nonstandard FPS override values. */
static inline void mlv_fps_to_rational(int fps_x1000,
                                       uint32_t *nom, uint32_t *denom)
{
    if (fps_x1000 <= 0)
    {
        *nom = 1;
        *denom = 1000;
        return;
    }

    uint32_t fps = (uint32_t)fps_x1000;
    uint32_t whole = (uint32_t)(((uint64_t)fps * 1001u + 500000u) / 1000000u);
    if (fps % 1000u && whole > 0u &&
        (uint32_t)(((uint64_t)whole * 1000000u) / 1001u) == fps)
    {
        *nom = whole * 1000u;
        *denom = 1001u;
    }
    else
    {
        *nom = fps;
        *denom = 1000u;
    }
}

#endif /* RAW_VIDX_MLV_FPS_RATIONAL_H */
