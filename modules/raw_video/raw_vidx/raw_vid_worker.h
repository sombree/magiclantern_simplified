#ifndef _raw_vid_worker_h_
#define _raw_vid_worker_h_

#include "dryos.h"

int init_worker(struct msg_queue *q);
enum recording_state get_recording_state(void);

// Priority for the worker tasks.  WORKER_PRIO controls the queue monitor.
// WORKER_WRITE_PRIO only the task that flushes data to disk.
// This is a compromise.  Lower values are higher priority.
// Too high priority and the GUI becomes very laggy.
// Too low and it limits effective write bandwidth.
// These specific values were empirically determined on 200D and are
// unlikely to work well on all cams.  More testing is needed.
#define WORKER_PRIO 0x11
#define WORKER_WRITE_PRIO 0x9

// Used to get space for our local buffers.  This is the initial
// size attempted, we reduce from their until success.
// Pick a nice round value; this is the aligned size, not underlying size.
#define INITIAL_CROP_BUF_SIZE (16 * 1024 * 1024)
#define CROP_BUF_ALIGN 0x1000 // aligned for when FIO_Write() reads from here (best alignment untested)
#define MIN_CROP_BUF_SIZE (4 * 1024 * 1024)
#define CROP_BUF_STEP_SIZE (1 * 1024 * 1024)


/* Read-only RP FIO drain/idle audit for the W3 automatic experiment scheduler.
 * Measured rate is FIO payload bytes / FIO_WriteFile call time, not guaranteed
 * physical SD write speed. Sampling is intended for TASK context only. */
struct raw_vid_rp_writer_status {
    uint32_t queued, done, queue_depth, header_pending;
    uint32_t io_fail, post_fail, batches, max_batch_bytes, max_write_us;
    uint32_t rate_mb_s_x100;
    uint32_t ingress_mb_s_x100, drain_mb_s_x100;
    uint32_t adaptive_target_bytes, adaptive_allowed_bytes;
    int file_open, idle;
};
void raw_vid_rp_writer_get_status(struct raw_vid_rp_writer_status *status);

#endif // _raw_vid_worker_h_
