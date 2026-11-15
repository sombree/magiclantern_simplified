/*
 * EOS RP 1.6.0 raw_vidx hot path.
 *
 * IMPORTANT ABI discipline (same model as hardware-validated rpsdtrc):
 * - this translation unit alone is Thumb/ARMv7-A;
 * - no ML/DryOS/FIO/menu/task/semaphore imports;
 * - no direct Thumb -> ARM call into another module object;
 * - no executable Canon ROM function hooks or trampoline entries;
 * - Canon's existing ch3 Complete callback-table slot invokes this function;
 * - Complete IRQ callback performs RAM state transitions only.
 */

#include <stdint.h>
#include "rp_hot_shared.h"

struct rp_capture_buffer rp_bufs[RP_CAPTURE_BUFS];
volatile int rp_buf_count = 0;

volatile int rp_current_slot = -1;

volatile int rp_capture_active = 0;
volatile int rp_stop_requested = 0;
volatile int rp_handback_done = 0;
volatile int rp_final_complete_seen = 0;
volatile int rp_ownership_uncertain = 0;

volatile uint32_t rp_owner_seq = 0;
volatile uint32_t rp_complete_seq = 0;
volatile uint32_t rp_ready_count = 0;
volatile uint32_t rp_drop_no_free = 0;
volatile uint32_t rp_bad_complete = 0;
volatile uint32_t rp_fatal = 0;
volatile uint32_t rp_overlap_count = 0;
volatile uint32_t rp_complete_frame_size = 0;
volatile uint32_t rp_complete_last_cur = 0;
volatile uint32_t rp_complete_matched = 0;
volatile uint32_t rp_complete_unmatched = 0;
static inline __attribute__((always_inline)) void rp_hot_dmb(void)
{
    __asm__ __volatile__("dmb ish" ::: "memory");
}

/* Existing Canon ch3 Complete IRQ tail-calls this pointer. */
void __attribute__((noinline,used)) raw_vid_rp_complete_tap(void *canon_arg)
{
    (void)canon_arg;

    rp_complete_seq++;

    /* v183.4 proved that continuous 4K may expose W3+A0 from one transfer
     * while W3+EC still belongs to another.  EC itself is nevertheless an
     * exact physical end pointer.  Match only dma + frozen physical_size. */
    uint32_t cur = *(volatile uint32_t *)(uintptr_t)0xD04202ECu;
    rp_complete_last_cur = cur;

    uint32_t frame_size = rp_complete_frame_size;
    int matched = -1;

    if (frame_size != 0u)
    {
        int count = rp_buf_count;
        if (count > RP_CAPTURE_BUFS) count = RP_CAPTURE_BUFS;

        for (int i = 0; i < count; i++)
        {
            if (rp_bufs[i].state == RPBUF_IN_FLIGHT &&
                cur == rp_bufs[i].dma + frame_size)
            {
                matched = i;
                break;
            }
        }
    }

    if (matched < 0)
    {
        /* Canon-owned or stale pipeline Complete.  This is normal during
         * arm/handback and must never be treated as an ownership error. */
        rp_complete_unmatched++;
        rp_hot_dmb();
        return;
    }

    int slot = matched;
    rp_complete_matched++;
    rp_bufs[slot].complete_seq = rp_complete_seq;

    if (rp_stop_requested)
    {
        /* Preserve historical stop semantics: frames completing after STOP
         * are drained but not posted behind COMMAND_STOP. */
        rp_bufs[slot].state = RPBUF_FREE;
    }
    else
    {
        rp_bufs[slot].state = RPBUF_READY;
        rp_ready_count++;
    }

    if (rp_handback_done && !rp_ownership_uncertain)
    {
        int any_in_flight = 0;
        int count = rp_buf_count;
        if (count > RP_CAPTURE_BUFS) count = RP_CAPTURE_BUFS;
        for (int i = 0; i < count; i++)
            if (rp_bufs[i].state == RPBUF_ARMED ||
                rp_bufs[i].state == RPBUF_IN_FLIGHT)
                any_in_flight = 1;
        if (!any_in_flight)
            rp_final_complete_seen = 1;
    }

    rp_hot_dmb();
}
