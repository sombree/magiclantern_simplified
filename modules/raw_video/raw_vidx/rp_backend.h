#ifndef _RAW_VIDX_RP_BACKEND_H_
#define _RAW_VIDX_RP_BACKEND_H_

#include "dryos.h"
#include "rp_w3_matrix.h"
#include "rp_w3_transport.h"

/*
 * EOS RP 1.6.0 RAW-video backend.
 *
 * This experimental raw_vidx build is RP-only.  Ordinary module code remains
 * ARM state; only Canon/IRQ entry points are isolated in a Thumb hot object.
 */
int raw_vid_rp_init(struct msg_queue *event_q);
int raw_vid_rp_start(uint32_t crop_x, uint32_t crop_y,
                     uint32_t crop_w, uint32_t crop_h);
int raw_vid_rp_prepare(uint32_t crop_x, uint32_t crop_y,
                       uint32_t crop_w, uint32_t crop_h);
int raw_vid_rp_arm(void);
uint32_t raw_vid_rp_capture_buffer_bytes(void);
int raw_vid_rp_prepare_capture_backend(void);
int raw_vid_rp_session_preflight(void);
int raw_vid_rp_sync_raw_info(void);
int raw_vid_rp_set_linear_packing(void);
int raw_vid_rp_restore_linear_packing(void);
int raw_vid_rp_set_bit_depth(int bpp);
int raw_vid_rp_restore_bit_depth(void);
int raw_vid_rp_probe_start(uint32_t crop_w, uint32_t crop_h, int stage);
int raw_vid_rp_probe_m2m(int stage);
int raw_vid_rp_m2m_crop_once(const void *src_frame, uint32_t src_pitch,
                              uint32_t crop_x, uint32_t crop_y,
                              uint32_t crop_w, uint32_t crop_h,
                              void *dst, uint32_t dst_size,
                              uint32_t *elapsed_us);
int raw_vid_rp_m2m_session_start(void);
int raw_vid_rp_m2m_crop_frame(const void *src_frame, uint32_t src_pitch,
                              uint32_t crop_x, uint32_t crop_y,
                              uint32_t crop_w, uint32_t crop_h,
                              void *dst, uint32_t dst_size,
                              uint32_t *elapsed_us);
int raw_vid_rp_m2m_session_stop(void);
int raw_vid_rp_m2m_copy_sg(const uint32_t *src_addr, const uint32_t *src_size,
                           void *const *dst_addr, uint32_t count,
                           uint32_t *bytes_done, uint32_t *blocks_done,
                           uint32_t *elapsed_us);
void raw_vid_rp_request_stop(void);
int raw_vid_rp_stop_before_raw_off(void);
void raw_vid_rp_write_report(int stop_rc);
int raw_vid_rp_cleanup_after_raw_off(void);
void raw_vid_rp_release_buffer(int slot);
/* b0=1 diagnostics: detect writes beyond nominal W3 frame length.
 * Guard marks live inside an OVERALLOCATED source ring, never beyond malloc.
 * All reporting performed in task context; this does NOT prove DMA safety. */
struct rp_w3_guard_status {
    uint32_t enabled, nominal, nominal16, allocated_per_slot;
    uint32_t checked, nominal_changed, full16_changed, after16_changed;
    uint32_t first_nominal_slot, first_after16_slot;
};
void raw_vid_rp_guard_get_status(struct rp_w3_guard_status *out);
/* Read-only model of expected physical W3 byte transport. Does not activate
 * LE16 DMA. Use only while RAW-LV is active; no MMIO access after RAW off. */
int raw_vid_rp_transport_get_plan(struct rp_w3_transport_plan *out);
uint32_t raw_vid_rp_wire_payload_bytes(void);
uint32_t raw_vid_rp_wire_pitch_bytes(void);

/* W3 matrix: all MMIO reads task-context; no IRQ side effects. */
int raw_vid_rp_matrix_apply(int field_index, int field_value);
int raw_vid_rp_matrix_validate_pre_arm(void);
int raw_vid_rp_matrix_restore(void);
void raw_vid_rp_matrix_snapshot(struct rp_w3_hw_snapshot *out);
void raw_vid_rp_matrix_counts(uint32_t *owner, uint32_t *complete);

#endif
