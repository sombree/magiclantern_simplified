#ifndef _RAW_VIDX_RP_HOT_SHARED_H_
#define _RAW_VIDX_RP_HOT_SHARED_H_

#include <stdint.h>

/* Shared RAM contract between the ordinary ARM control object and the tiny
 * Thumb/ARMv7 Canon/IRQ hot-path object.  The hot object must not call any
 * ML/DryOS function; communication is data-only through these variables. */

#define RP_CAPTURE_BUFS 3

enum rp_capture_state
{
    RPBUF_FREE = 0,
    /* Destination has been published in MNAV but has not yet latched through
     * provider -> Evf -> Mem1 -> W3. Complete must ignore this state. */
    RPBUF_ARMED,
    /* Full four-owner latch observed; exact W3+EC may now complete it. */
    RPBUF_IN_FLIGHT,
    RPBUF_READY,
    RPBUF_QUEUED,
};

struct rp_capture_buffer
{
    void *alloc;
    uint32_t dma;
    volatile uint32_t state;
    uint32_t owner_seq;
    uint32_t complete_seq;
};

extern struct rp_capture_buffer rp_bufs[RP_CAPTURE_BUFS];
extern volatile int rp_buf_count;

extern volatile int rp_current_slot;

extern volatile int rp_capture_active;
extern volatile int rp_stop_requested;
extern volatile int rp_handback_done;
extern volatile int rp_final_complete_seen;
extern volatile int rp_ownership_uncertain;

extern volatile uint32_t rp_owner_seq;
extern volatile uint32_t rp_complete_seq;
extern volatile uint32_t rp_ready_count;
extern volatile uint32_t rp_drop_no_free;
extern volatile uint32_t rp_bad_complete;
extern volatile uint32_t rp_fatal;
extern volatile uint32_t rp_overlap_count;

/* v23 RAM-only MNAV backend: true ch3 Complete is identified by W3+EC,
 * not by the currently programmed W3 base address.  4K may expose A0/EC from
 * adjacent pipeline frames, so only an exact end pointer is authoritative. */
extern volatile uint32_t rp_complete_frame_size;
extern volatile uint32_t rp_complete_last_cur;
extern volatile uint32_t rp_complete_matched;
extern volatile uint32_t rp_complete_unmatched;

/* Canon's pre-existing ch3 Complete callback table is writable RAM.
 * The ARM control code installs this pointer into the RAM table, not ROM. */
void raw_vid_rp_complete_tap(void *canon_arg);

#endif
