#ifndef _raw_vid_pusher_h_
#define _raw_vid_pusher_h_

#include "dryos.h"
#include "raw_vid.h"

int init_event_pusher(struct msg_queue *event_q);
struct raw_vid_event *get_empty_event(void);
void release_event(struct raw_vid_event *event);
void allow_recording(void);

/* Task-context stop request used by platform-specific producer backends. */
void raw_vid_request_stop_async(void);
int rp_w3_matrix_send_start(int bpp);

#endif // _raw_vid_pusher_h_
