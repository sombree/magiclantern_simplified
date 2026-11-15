#ifndef RP_W3_MATRIX_H
#define RP_W3_MATRIX_H

#include <stdint.h>

/* Distinct read-only snapshot. Read from TASK context, never IRQ. */
struct rp_w3_hw_snapshot {
    uint32_t mem1_state, cfg[5];
    uint32_t pu20, pu24, pu28;
    uint32_t w350, w3a0, w3d0;
    uint32_t cb, cb_arg;
    uint32_t owner, complete, drops, overlap, bad_complete, post_fail, fatal;
};

/* Called by menu task / RP physical REC handler */
void rp_w3_matrix_launch(void); /* backwards-compatible: checkpointed */
void rp_w3_matrix_launch_mode(int no_restart); /* 0=checkpointed, 1=uninterrupted */
void rp_w3_matrix_launch_single(int case_index);
void rp_w3_matrix_launch_pair(void); /* isolated A00->A01 pair; no restarts */ /* isolated single trial, no checkpoint */
void rp_w3_matrix_stage(int code, int detail); /* TASK worker: RAM-only stage breadcrumb */
int rp_w3_matrix_running(void);
int rp_w3_matrix_interlocked(void);
void rp_w3_matrix_abort_user(void);

/* Called only by existing raw_vidx event/worker paths */
int rp_w3_matrix_case_active(void);
int rp_w3_matrix_requires_linear(void);
int rp_w3_matrix_field_index(void);
int rp_w3_matrix_field_value(void);
const char *rp_w3_matrix_video_pattern(void);
void rp_w3_matrix_file(const char *name);
void rp_w3_matrix_capture_pre(void);
void rp_w3_matrix_apply_result(int rc);
void rp_w3_matrix_on_arm(void);
void rp_w3_matrix_capture_end(void);
void rp_w3_matrix_on_frame(uint32_t frame, int slot, uint32_t time_us);
void rp_w3_matrix_restore_result(int rc);
void rp_w3_matrix_pre_cleanup(int stop_rc, int drain_rc, int frames,
                              int io_fail, int post_fail);
void rp_w3_matrix_on_stop(int cleanup_rc, int stop_rc, int drain_rc);
void rp_w3_matrix_storage_status(int io_fail, int post_fail);

#endif
