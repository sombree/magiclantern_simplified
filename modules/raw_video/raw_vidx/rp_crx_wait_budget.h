#ifndef RAW_VIDX_RP_CRX_WAIT_BUDGET_H
#define RAW_VIDX_RP_CRX_WAIT_BUDGET_H

#include <stdint.h>

/* Remaining-time-bounded task wait. 0 means timeout already elapsed.
 * Rounds up to milliseconds, but never waits longer than the chosen slice;
 * the caller must recheck the completed flag after every wake/timeout. */
static inline uint32_t rp_crx_wait_budget_ms(uint32_t elapsed_us,
                                              uint32_t deadline_us,
                                              uint32_t slice_ms)
{
    if (elapsed_us >= deadline_us || slice_ms == 0u)
        return 0u;
    uint32_t remaining_us = deadline_us - elapsed_us;
    uint32_t wait_ms = remaining_us / 1000u;
    if (remaining_us % 1000u)
        ++wait_ms;
    return wait_ms > slice_ms ? slice_ms : wait_ms;
}

#endif /* RAW_VIDX_RP_CRX_WAIT_BUDGET_H */
