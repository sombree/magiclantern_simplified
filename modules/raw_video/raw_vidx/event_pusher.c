#include "module.h"
#include "dryos.h"
#include "raw.h" // raw_info global, raw_lv_redirect_edmac()
#include "zebra.h" // liveview_display_idle()
#include "menu.h"

// Ugly - we use propvalues.c to hold various globals
// and functions that are nothing to do with properties,
// just global state for various camera things,
// including raw video stuff.
// E.g. is_movie_mode(), set_recording_custom(),
// various RECORDING macros.
#include "propvalues.h"
#include "property.h"

#include "raw_vid.h"
#include "raw_vid_event_pusher.h"
#include "raw_vid_worker.h"
#include "rp_backend.h"
#include "rp_w3_matrix.h"

#if RAW_VID_Q_LEN < 5
    #error "RAW_VID_Q_LEN too short"
#endif

// This is about 56MB/s at 24fps
//#define MLV_3_CROP_WIDTH 1600
//#define MLV_3_CROP_HEIGHT 900

// This is 64.3MB/s at 24fps (legacy/default path).
#define MLV_3_CROP_WIDTH 1792
#define MLV_3_CROP_HEIGHT 896

// This is 67.8MB/s at 24fps
//#define MLV_3_CROP_WIDTH 1888
//#define MLV_3_CROP_HEIGHT 896

// This is 70.2MB/s at 24fps
//#define MLV_3_CROP_WIDTH 2048
//#define MLV_3_CROP_HEIGHT 856

// This is 73.5MB/s at 24fps
// (and nice round numbers)
//#define MLV_3_CROP_WIDTH 2048
//#define MLV_3_CROP_HEIGHT 896

static struct msg_queue *event_q = NULL;

// We post these events to the msg queue.  Since we push a pointer,
// we need some storage that will live until the msg has been consumed.
//
// Don't access these events directly, only use get_event() / release_event().
// This handles the sem to avoid races.
// Only the worker receives or releases events; single consumer reduces code complexity.
static struct raw_vid_event events[RAW_VID_Q_LEN] = {0};
static struct semaphore *events_sem = NULL;

// Local var to rapidly stop pushing image data to worker.
// Should not be accessed directly.  send_stop_command() uses this.
//
// Should not need a sem, since this side only ever changes it 0 -> 1,
// only worker changes 1 -> 0, via allow_recording().  And worker
// flushes any queued events before making that change.
static int prevent_recording = 1;

static void send_stop_command(void); // forward declaration

// Returns a raw_vid_event in the PREPARING state,
// or returns NULL if all events are in use.
// That should only happen if the worker cannot service our requests fast enough,
// probably because we can't save to card fast enough.
//
// We use a static array of events, and .type == EMPTY to signify
// if the event can be re-used.  Therefore, caller must never set type
// of the obtained event to EMPTY, as this means it can be reassigned
// in a racey manner.
//
// Caller can modify other parts of the event before posting.
// Caller must never modify any part of an event after posting;
// the receiver is responsible for releasing the event
// (which it will do with release_event()).
struct raw_vid_event *get_empty_event(void)
{
    static size_t last_index = 0;
    struct raw_vid_event *event = NULL;

    int err = take_semaphore(events_sem, 2000);
    if (err)
    {
        SEND_LOG_DATA_STR("ERROR: get_next_event() sem timeout\n");
        task_create("stop_com", WORKER_PRIO + 1, 0x800, send_stop_command, NULL);
        return NULL;
    }

    // This is an optimisation; we test the event after the last one
    // we assigned.  Most of the time usage should be sequential.
    if (last_index < RAW_VID_Q_LEN - 1)
        last_index++;
    else
        last_index = 0;

    if (events[last_index].type == EMPTY)
    {
        event = &events[last_index];
    }
    else
    {
        // simple linear search, it's a small contiguous array
        int i = 0;
        while (i < RAW_VID_Q_LEN)
        {
            if (events[i].type == EMPTY)
            {
                event = &events[i];
                break;
            }
            i++;
        }
    }

    if (event != NULL)
    {
        memset(event, 0, sizeof(*event));
        event->type = PREPARING;
        event->capture_slot = -1;
    }

    give_semaphore(events_sem);
    return event;
}

// Allows the provided event to be re-used.
// After calling this, the caller must not use
// the event again.
void release_event(struct raw_vid_event *event)
{
    // Here we only zero the type.  This allows get_event() to re-use the event,
    // and that func zeroes the complete event.
    // This removes a source of race conditions due to ARM weak memory model.
    event->type = EMPTY;
}

// This should only be called by worker.c,
// when it wishes to allow this file to start a new recording.
void allow_recording(void)
{
    prevent_recording = 0;
}



int init_event_pusher(struct msg_queue *q)
{
    event_q = q;
    events_sem = create_named_semaphore("raw_vid_events", SEM_CREATE_UNLOCKED);
    return events_sem ? 0 : -1;
}

// Send stop command, handling the possiblity that the queue
// may be full.
//
// Also sets the flag to immediately disallow sending image data
// from this file to the queue.  This allows the queue to empty,
// should it be full.
//
// Probably works if called directly, but it's expected you'd
// use task_create().  That allows this function to be slow as it
// attempts retries, etc, without blocking whatever context you
// trigger it from.  We need to handle problems encountered in
// timing critical CBR contexts.
static void send_stop_command(void)
{
    if (is_camera("RP", "1.6.0"))
        raw_vid_rp_request_stop();
    prevent_recording = 1;
    SEND_LOG_DATA_STR("Sending stop command to worker\n");

    const int max_tries = 10;
    int tries = 0;
    struct raw_vid_event *event = get_empty_event();
    while (event == NULL && tries < max_tries)
    { // should happen only if the queue is completely full,
      // we must wait and hope the worker can process some events.
        msleep(50); // more time than one frame at 24 fps,
                    // probably enough we only wait once.
        event = get_empty_event();
        tries++;
    }
    if (tries == max_tries)
    {
        SEND_LOG_DATA_STR("ERROR: could not get event for stop command\n");
        return;
    }

    event->creation_time = get_ms_clock();
    event->type = COMMAND_STOP;

    tries = 0;
    int err = msg_queue_post(event_q, (uint32_t)event);
    while (err && tries < max_tries)
    {
        msleep(50);
        // FIXME: use msg_queue_post_with_timeout()?
        err = msg_queue_post(event_q, (uint32_t)event);
        tries++;
    }
    if (tries == max_tries)
    {
        SEND_LOG_DATA_STR("ERROR: could not post stop command event\n");
    }
}

void raw_vid_request_stop_async(void)
{
    task_create("stop_com", WORKER_PRIO + 1, 0x800, send_stop_command, NULL);
}

/* v111: Still Photo LiveView is a normal recorder session.  The physical
 * REC key is used when forwarded by RP; the Movie-menu Start/Stop action is
 * a fallback. The watcher only requests the existing orderly worker STOP. */
static volatile int still_session_pending = 0;
static volatile int still_session_stop_sent = 0;
static volatile uint32_t still_session_requested_ms = 0;
static volatile uint32_t still_session_active_ms = 0;
static volatile uint32_t still_session_last_frame_ms = 0;
static volatile uint32_t still_session_frame_count = 0;
static volatile int still_session_start_shootmode = 0;
static volatile int raw_record_menu_pending = 0;
static volatile int still_session_watch_ready = 0;

#define STILL_SESSION_START_TIMEOUT_MS 15000u
#define STILL_SESSION_NO_FRAME_TIMEOUT_MS 7000u
#define STILL_MENU_CLOSE_TIMEOUT_MS 5000u

int raw_vid_still_mode_ok(void)
{
    /* Only the physical photo position, never an H264/movie session.
     * Do not use liveview_display_idle(): it is not validated on RP. */
    return still_session_watch_ready && lv && !is_movie_mode() &&
           shooting_mode != SHOOTMODE_MOVIE && !RECORDING_H264;
}

void raw_vid_still_session_stop_once(void)
{
    if (still_session_pending && !still_session_stop_sent)
    {
        still_session_stop_sent = 1;
        DryosDebugMsg(0, 15, "RPVIDX STILL STOP request frame=%d", (int)still_session_frame_count);
        /* Check task creation, unlike the historical generic async helper.
         * A failed STOP task must not permanently disable the REC fallback. */
        uint32_t tid = task_create("stop_com", WORKER_PRIO + 1, 0x800,
                                   send_stop_command, NULL);
        if (!tid)
        {
            still_session_stop_sent = 0;
            DryosDebugMsg(0, 15, "RPVIDX STILL STOP task create FAILED");
        }
    }
}

void raw_vid_still_session_frame_seen(uint32_t frames)
{
    if (still_session_pending)
    {
        still_session_frame_count = frames;
        still_session_last_frame_ms = get_ms_clock();
    }
}

void raw_vid_still_session_finish(void)
{
    if (still_session_pending)
        DryosDebugMsg(0, 15, "RPVIDX STILL END frame=%d state=%d",
                      (int)still_session_frame_count, (int)get_recording_state());
    still_session_pending = 0;
    still_session_stop_sent = 0;
    still_session_active_ms = 0;
}

static void raw_vid_still_watch_task(void *unused)
{
    (void)unused;
    TASK_LOOP
    {
        msleep(250);
        if (!still_session_pending) continue;
        uint32_t now = get_ms_clock();
        enum recording_state state = get_recording_state();
        if (state == ACTIVE)
        {
            if (!still_session_active_ms)
            {
                still_session_active_ms = now;
                still_session_last_frame_ms = now;
                DryosDebugMsg(0, 15, "RPVIDX STILL watchdog ACTIVE mode=%d",
                              (int)still_session_start_shootmode);
            }
            if (!still_session_stop_sent &&
                (!raw_vid_still_mode_ok() ||
                shooting_mode != still_session_start_shootmode ||
                now - still_session_last_frame_ms > STILL_SESSION_NO_FRAME_TIMEOUT_MS))
            {
                DryosDebugMsg(0, 15, "RPVIDX STILL watchdog STOP mode=%d lv=%d frames=%d age=%d",
                              shooting_mode, lv, (int)still_session_frame_count,
                              (int)(now - still_session_last_frame_ms));
                raw_vid_still_session_stop_once();
            }
        }
        else if (!still_session_stop_sent && !still_session_active_ms &&
                 now - still_session_requested_ms > STILL_SESSION_START_TIMEOUT_MS)
        {
            /* A failed COMMAND_START is handled by worker cleanup.  For a
             * stalled start enqueue STOP once; never block the watcher. */
            DryosDebugMsg(0, 15, "RPVIDX STILL START timeout; requesting STOP");
            raw_vid_still_session_stop_once();
        }
    }
}

void raw_vid_still_watch_start(void)
{
    still_session_watch_ready = !!task_create("rp_stillwatch", WORKER_PRIO + 1,
                                             0x1000, raw_vid_still_watch_task, NULL);
    DryosDebugMsg(0, 15, "RPVIDX STILL watcher ready=%d", still_session_watch_ready);
}

/* EOS RP native REC event 0x28, handled for both Movie and photo LV.
 * Some photo GUI modes may not forward this key; the normal menu action
 * supplies an alternative without enabling any test mode. */
#define RP_RAWVID_REC_PRESS 0x28u

/* Historical W3 matrix launcher is retained for source compatibility;
 * ordinary REC never calls it. */
int rp_w3_matrix_send_start(int bpp)
{
    if (!event_q || get_recording_state() != INACTIVE || prevent_recording)
        return -1;
    struct raw_vid_event *e = get_empty_event();
    if (!e) return -2;
    e->creation_time = get_ms_clock();
    e->type = COMMAND_START;
    e->requested_bpp = bpp;
    e->wire_le16 = 0; /* Matrix Single selects its own wire mode. */
    e->resolution_mode = RAW_VID_RES_W3_TRANSPORT;
    e->codec_mode = RAW_VID_CODEC_RAW; /* never touch Canon CRX persistent banks */
    e->crx_frame_limit = 0;
    e->crx_latch_stop_mode = RAW_VID_CRX_LATCH_OFF;
    e->crx_session_budget = 0;
    e->crx_stillwatch = 0;
    e->crx_allow_id0_still = 0;
    if (msg_queue_post(event_q, (uint32_t)e))
    {
        release_event(e);
        return -3;
    }
    return 0;
}

/* v111: one REC path for Movie and Still Photo LiveView. The session type
 * is separate from any frame-limit policy. No diagnostic opt-in gates exist. */
static unsigned int raw_vid_rp_rec_press(void)
{
    const int movie_mode = !!is_movie_mode();
    const int still_mode = !movie_mode;
    const enum recording_state state = get_recording_state();

    if (!lv || gui_menu_shown() || RECORDING_H264 || !event_q)
        return 1; /* Leave unrelated Canon key handling unchanged. */

    if (state == ACTIVE)
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 STOP still=%d", !!still_session_pending);
        if (still_session_pending)
            raw_vid_still_session_stop_once();
        else if (!task_create("stop_com", WORKER_PRIO + 1, 0x800,
                              send_stop_command, NULL))
            DryosDebugMsg(0, 15, "RPVIDX REC111 STOP task failed");
        return 0;
    }
    if (state != INACTIVE || still_session_pending || prevent_recording)
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 busy st=%d pending=%d blocked=%d",
                      state, still_session_pending, prevent_recording);
        return 0; /* Never forward a second REC to Canon mid-transition. */
    }
    if (still_mode && !raw_vid_still_mode_ok())
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 still guard lv=%d mode=%d",
                      lv, shooting_mode);
        return 1;
    }

    struct raw_vid_event *event = get_empty_event();
    if (!event)
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 no free event");
        return 0;
    }
    event->creation_time = get_ms_clock();
    event->type = COMMAND_START;
    event->still_session = still_mode;
    event->requested_bpp = raw_vid_get_requested_bpp();
    event->wire_le16 = 0;
    event->resolution_mode = raw_vid_get_requested_resolution_mode();
    event->codec_mode = raw_vid_get_requested_codec();
    event->crx_frame_limit = raw_vid_get_requested_crx_frame_limit();
    event->crx_latch_stop_mode = RAW_VID_CRX_LATCH_OFF;
    event->crx_session_budget = 0;
    event->crx_stillwatch = 0;
    event->crx_allow_id0_still = 0;
    event->x = event->y = event->w = event->h = 0;
    const int codec = event->codec_mode;
    const int bpp = event->requested_bpp;
    const int res = event->resolution_mode;

    if (still_mode)
    {
        still_session_pending = 1;
        still_session_stop_sent = 0;
        still_session_requested_ms = get_ms_clock();
        still_session_active_ms = 0;
        still_session_last_frame_ms = still_session_requested_ms;
        still_session_frame_count = 0;
        still_session_start_shootmode = shooting_mode;
    }
    const int rc = msg_queue_post(event_q, (uint32_t)event);
    DryosDebugMsg(0, 15,
                  "RPVIDX REC111 START rc=%d still=%d codec=%d bpp=%d res=%d",
                  rc, still_mode, codec, bpp, res);
    if (rc)
    {
        release_event(event);
        if (still_mode) raw_vid_still_session_finish();
    }
    return 0;
}

/* The menu is an alternative control for photo modes where Canon does not
 * forward GUI event 0x28. It waits for the menu to close before START. */
static void raw_vid_record_menu_task(void *unused)
{
    (void)unused;
    const uint32_t started = get_ms_clock();
    while (gui_menu_shown() && get_ms_clock() - started < STILL_MENU_CLOSE_TIMEOUT_MS)
        msleep(100);
    if (!gui_menu_shown() && lv && !RECORDING_H264 &&
        get_recording_state() == INACTIVE && !still_session_pending)
        raw_vid_rp_rec_press();
    else
        DryosDebugMsg(0, 15, "RPVIDX REC111 menu canceled lv=%d mode=%d",
                      lv, shooting_mode);
    raw_record_menu_pending = 0;
}

void raw_vid_record_menu_toggle(void)
{
    if (get_recording_state() == ACTIVE)
    {
        if (still_session_pending)
            raw_vid_still_session_stop_once();
        else
            raw_vid_request_stop_async();
        return;
    }
    if (!lv || RECORDING_H264 || still_session_pending ||
        raw_record_menu_pending || get_recording_state() != INACTIVE)
        return;
    raw_record_menu_pending = 1;
    if (!task_create("rp_rec_menu", WORKER_PRIO + 1, 0x1000,
                     raw_vid_record_menu_task, NULL))
        raw_record_menu_pending = 0;
}

static unsigned int raw_vid_keypress_cbr_raw(unsigned int raw_event)
{
    struct event *event = (struct event *)raw_event;
    if (!event)
        return 1;

    if ((uint32_t)event->param != RP_RAWVID_REC_PRESS)
        return 1;

    return raw_vid_rp_rec_press();
}

/*
 * Legacy raw_vidx used CBR_VSYNC and raw_lv_redirect_edmac() here.
 * EOS RP 1.6.0 has a separately proven frame-owner + true-Complete backend,
 * so this RP-only build intentionally carries no legacy VSYNC capture path.
 */

MODULE_CBRS_START()
    /* Use the unmodified GUI event on RP.  This bypasses incomplete portable
     * key/movie-mode translation while capture timing still comes entirely
     * from the EvfDevVram owner hook + hardware Complete tap. */
    MODULE_CBR(CBR_KEYPRESS_RAW, raw_vid_keypress_cbr_raw, 0)
MODULE_CBRS_END()
