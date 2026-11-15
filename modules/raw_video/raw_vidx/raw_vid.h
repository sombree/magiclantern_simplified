#ifndef _raw_vid_h_
#define _raw_vid_h_

#undef RAW_VID_LOG
//#define RAW_VID_LOG 1

#ifdef RAW_VID_LOG
    #include "log.h"
    #define SEND_LOG_DATA_STR(s) send_log_data_str(s)
#else
    #define SEND_LOG_DATA_STR(s) ;
#endif

// While we do send an event for every frame pushed,
// the queue receiving them doesn't need to be very long,
// because we must keep up, or we will drop frames.
#define RAW_VID_Q_LEN 20

/* Runtime RAW bit depth for the active recording session.
 * RP supports native uncompressed 14/12/10-bit through Canon's D8
 * PackUnpack path; keep a frozen session value so menu changes cannot alter
 * geometry while recording. */
int raw_vid_get_requested_bpp(void);
/* W3 output container, independent from Canon sample precision. */
int raw_vid_get_requested_le16(void);
int raw_vid_get_bpp(void);
void raw_vid_set_active_bpp(int bpp);
#define BPP (raw_vid_get_bpp())

enum raw_vid_codec_mode
{
    RAW_VID_CODEC_RAW = 0,
    RAW_VID_CODEC_CRX,          /* lossless CRX */
    RAW_VID_CODEC_CRAW,         /* lossy Canon C-RAW / CRX transform path */
};

static inline int raw_vid_codec_is_crx(int mode)
{
    return mode == RAW_VID_CODEC_CRX || mode == RAW_VID_CODEC_CRAW;
}

static inline int raw_vid_codec_is_lossy_crx(int mode)
{
    return mode == RAW_VID_CODEC_CRAW;
}

int raw_vid_get_requested_codec(void);

/* 0 = continuous; safe STOP is handled by recorder state machine. */
int raw_vid_get_requested_crx_frame_limit(void);

/* Normal REC supports Movie and photo LiveView; Still sessions are not
 * frame-bounded tests and use the same frozen codec/BPP/resolution settings. */
int raw_vid_still_mode_ok(void);
void raw_vid_record_menu_toggle(void);
void raw_vid_still_watch_start(void);
void raw_vid_still_session_frame_seen(uint32_t frames);
void raw_vid_still_session_finish(void);
void raw_vid_still_session_stop_once(void);

/* v23.2 production CRX policy is fixed: the recorder always selects the
 * first compatible free D-slot and, after a clean STOP, clears that slot's
 * Canon init latch and rewinds only its exact ML-owned allocator tail.
 * There is no Legacy/fixed-route selector in the recorder UI or event ABI. */

/* Retained only as internal compatibility constants for old code paths.
 * v22.3 exposes no latch/session/still/tail diagnostic controls in the menu. */
enum raw_vid_crx_latch_stop_mode
{
    RAW_VID_CRX_LATCH_OFF = 0,
    RAW_VID_CRX_LATCH_OBSERVE,
    RAW_VID_CRX_LATCH_CLEAR_ONCE,
    RAW_VID_CRX_LATCH_CLEAR_MULTI,
    RAW_VID_CRX_LATCH_CLEAR_EVERY, /* v16: reset after EVERY clean stop, incl. last */
};
int raw_vid_get_requested_crx_latch_stop_mode(void);
/* v15: bounded number of native CRX recording sessions in one boot.
 * 0 is the legacy unlimited mode with no cross-profile test waiver;
 * positive values are an explicit diagnostic multi-session experiment. */
int raw_vid_get_requested_crx_session_budget(void);
int raw_vid_get_requested_crx_stillwatch(void);
int raw_vid_get_requested_crx_id0_still(void);
/* v17 Canon-only watcher: non-persisted menu selector, never arms ML REC. */
int raw_vid_get_native_crx_watch_mode(void);
/* v18 nonpersistent opt-in; read-only observational analysis. */
int raw_vid_get_crx_tail_audit(void);
void raw_vid_disarm_native_crx_watch(int expected_mode);

/* RP output resolution is frozen when REC is pressed.  Fixed presets retain
 * the runtime-proven crop origin; Full uses the metadata active area,
 * while W3 transport exposes the entire physical W3 buffer, including OB. */
enum raw_vid_resolution_mode
{
    RAW_VID_RES_720P = 0,
    RAW_VID_RES_1080P,
    RAW_VID_RES_1440P,
    RAW_VID_RES_FULL,
    RAW_VID_RES_W3_TRANSPORT, /* exact entire physical W3, including OB */
    RAW_VID_RES_W3_WIDE_720, /* W3 horizontal extent and centered 720-row ROI */
};

int raw_vid_get_requested_resolution_mode(void);
int raw_vid_resolve_capture_area(int mode, uint32_t *x, uint32_t *y,
                                 uint32_t *w, uint32_t *h);

/* EOS RP recorder MVP crop origin.
 * x=192 is divisible by 32 pixels: packed14 byte offset is 336 (0x150),
 * i.e. 16-byte aligned.  With the proven 4K pitch 6524 (divisible by 4),
 * every following row therefore remains 32-bit aligned.  The earlier x=200
 * produced byte offset 350 (address % 4 == 2) on every row, and the first
 * uncached memcpy probe froze the camera.  Both coordinates remain Bayer-safe. */
#define RP_RAW_VID_CROP_X 192
#define RP_RAW_VID_CROP_Y 100

enum event_type
{
    // Events with these types should never be posted to the queue.
    EMPTY = 0, // We zero the event to init or re-use it, so this value must be explicit.
    PREPARING, // Reserved, but otherwise empty; used to limit time spent in critical section.

    // Events with these types only need to set creation_time and type,
    // the other fields are ignored.
    COMMAND_START, // Producer / event_handler requests start.  Consumer / worker may reject the request.
    COMMAND_STOP,  // As above, but stop.

    // This event must set valid values for all fields.
    IMAGE_DATA, // Some image data to save to file.
};

struct raw_vid_event
{
    uint32_t creation_time;
    enum event_type type;
    char *src; // points to image data if capturing

    uint32_t orig_frame_width;
    uint32_t x; // x offset within the frame, for cropping
    uint32_t y; // y as above
    uint32_t w; // our crop width
    uint32_t h; // our crop height

    int32_t size; // size calculated by producer, in bytes

    /* Frozen RP output-resolution selector for COMMAND_START.
     * IMAGE_DATA events do not consult this field. */
    int32_t requested_bpp; /* sampled atomically with REC settings */
    int32_t wire_le16; /* sampled at REC: 0=packed, 1=16-bit LE */
    int32_t resolution_mode;
    int32_t codec_mode;
    int32_t crx_frame_limit; /* 0 = continuous; frozen at COMMAND_START */
    int32_t crx_latch_stop_mode; /* frozen experimental policy */
    int32_t crx_session_budget;  /* 0 or bounded 4/6/9 sessions */
    int32_t crx_stillwatch;      /* readonly Canon-still snapshots after last STOP */
    int32_t crx_allow_id0_still; /* explicit dangerous opt-in, snapshotted at REC */
    int32_t still_session; /* 1: photo LiveView; 0: Movie LV */

    /*
     * RP capture-ring slot backing src.  -1 for events not owned by the RP
     * capture backend.  The worker releases this slot only after its
     * synchronous CPU crop has finished.
     */
    int32_t capture_slot;
};

enum recording_state
{
    ERROR,
    // RECORDING // can't use this name because propvalues.h #defines it
    INACTIVE,
    ACTIVE
};

#endif // _raw_vid_h_
