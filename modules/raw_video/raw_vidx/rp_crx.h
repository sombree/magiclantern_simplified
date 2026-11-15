#ifndef _RAW_VIDX_RP_CRX_H_
#define _RAW_VIDX_RP_CRX_H_

#include <stdint.h>

#define RP_CRX_RESULT_MAX 3u

struct rp_crx_result
{
    uint32_t handles[RP_CRX_RESULT_MAX];
    uint32_t count;
    uint32_t submit_rc;
    uint32_t encode_us;

    /* Exact route actually submitted to Canon.  handle[1] mirrors the
     * descriptor type, not the requested BPP.  The dynamic-only allocator may
     * therefore encode 10/12/14-bit through the same physical D route. */
    uint32_t descriptor_type;
    uint32_t encoder_id;
};

struct rp_crx_stats
{
    uint32_t submitted;
    uint32_t completed;
    uint32_t submit_errors;
    uint32_t timeouts;
    uint32_t unexpected_callbacks;
    uint32_t total_encode_us;
    uint32_t max_encode_us;
    uint32_t poisoned;
    uint32_t encoder_latched_bits;
    uint32_t free_pending;
    uint32_t free_completed;
    uint32_t free_callback_errors;
    uint32_t reclaim_attempts;
    uint32_t reclaim_successes;
    int32_t reclaim_last_rc;
    uint32_t reclaim_last_path; /* 1=manager post-event4, 2=worker fallback */
    uint32_t reclaim_route_type;
    uint32_t reclaim_route_id;
};

/* Begin a new recorder session.  A timed-out encoder job poisons the backend
 * for the lifetime of the loaded module; fresh boot is then required. */
void raw_vid_rp_crx_request_reclaim(void);
int raw_vid_rp_crx_preflight_mode(uint32_t requested_bits, int lossy);
int raw_vid_rp_crx_preflight(uint32_t requested_bits);
int raw_vid_rp_crx_begin_session_mode(int lossy);
int raw_vid_rp_crx_begin_session(void);
/* Standard tight-input API retained for all legacy latch/retry/cursor
 * experiments. It is intentionally unchanged semantically. */
int raw_vid_rp_crx_encode(void *tight_src, uint32_t width, uint32_t height,
                          uint32_t bits, struct rp_crx_result *out);

/* v20.2 standard-capture input path.
 * Allows Canon AlgsMgr to read a logical ROI directly from a padded W3 source.
 * This is used ONLY by the normal capture worker when latch/session/cursor
 * experiments are disabled. */
int raw_vid_rp_crx_encode_source_mode(void *source, uint32_t source_width,
                                      uint32_t x, uint32_t y,
                                      uint32_t width, uint32_t height,
                                      uint32_t bits, int lossy,
                                      struct rp_crx_result *out);
int raw_vid_rp_crx_encode_source(void *source, uint32_t source_width,
                                 uint32_t x, uint32_t y,
                                 uint32_t width, uint32_t height,
                                 uint32_t bits, struct rp_crx_result *out);
int raw_vid_rp_crx_free(uint32_t handle);
int raw_vid_rp_crx_end_session(void);
/* v14: bounded experimental 1 x retry.  After first clean stop CLEAR_ONCE
 * sets exactly one latch to zero, then arms exactly one further CRX session
 * with the same bit depth and at most one frame.  No second latch reset. */
#define RP_CRX_RETRY_NONE      0
#define RP_CRX_RETRY_ARMED     1
#define RP_CRX_RETRY_RUNNING   2
#define RP_CRX_RETRY_FINISHED  3
int raw_vid_rp_crx_retry_state(void);
int raw_vid_rp_crx_retry_bits(void);
/* Worker-only after the second fully drained stop; logs allocation deltas and
 * permanently closes the ML retry window regardless of the outcome. */
void raw_vid_rp_crx_retry_finish(int clean_stop);
/* v16: opt-in, bounded (4/6/9) experimental multi-clear session lifecycle.
 * Separate from the v13/v14 one-clear retry implementation. */
int raw_vid_rp_crx_multi_setup(uint32_t budget);
/* Logs a summary of the current session/allocator without any Canon writes. */
void raw_vid_rp_crx_observe_banks(const char *phase, uint32_t session_num);
/* Read-only 120-second Canon still observer; call via task_create, not the worker. */
void raw_vid_rp_crx_stillwatch_task(void *unused);
/* v17: persistent idle task, opt-in read-only Canon CRX survey. */
void raw_vid_rp_crx_nativewatch_task(void *unused);
/* v18; call after Canon FreeEncBuffer and RP backend cleanup, BEFORE any
 * optional legacy latch-clear experiment. Does not write Canon state. */
void raw_vid_rp_crx_tail_audit_after_stop(int clean_stop);
/* Call in worker task ONLY after clean end_session, MLV writer/file closure,
 * RAW-LV teardown and RP backend cleanup. */
int raw_vid_rp_crx_latch_after_stop(uint32_t bits, int mode);
void raw_vid_rp_crx_get_stats(struct rp_crx_stats *out);

#endif
