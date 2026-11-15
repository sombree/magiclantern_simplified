#include "module.h"
#include "dryos.h"
#include "config.h"
#include "menu.h"
#include "raw.h"
#include "propvalues.h"

#include "raw_vid.h"
#include "raw_vid_event_pusher.h"
#include "raw_vid_worker.h"
#include "rp_crx.h"
#include "rp_w3_matrix.h"

/*
 * EOS RP 1.6.0 raw_vidx v111 integrated recorder.
 *
 * ABI split validated by the v8 load-isolation run:
 * - ordinary raw_vidx module/control/worker code uses the standard ML ARM ABI;
 * - RP control backend is ARMv7-A/ARM;
 * - only the hardware Complete observer entry point is used by normal REC.
 *
 * MODULE_INIT initializes only the queue/event/worker infrastructure already
 * proven independently as v8 S1/S2/S3. RAW mode, full-frame capture buffers,
 * normal REC uses RAM-only Mem1/MNAV steering; no E00DDF7C or E057C364
 * function hook is installed. The ch3 Complete table tap is per-session.
 */


/* RP DIGIC-8 native uncompressed RAW bit depth.  Keep the UI value separate
 * from the active session value: changing a menu item while recording must not
 * silently change packed-row geometry or MLV metadata mid-file. */
static CONFIG_INT("raw_vidx.bit_depth", raw_vid_bpp_mode, 0);
static volatile int raw_vid_active_bpp = 14;
/* v23 normal REC always uses Canon's native 14/12/10-bit W3 transport.
 * The old owner-scoped Full-LE16 path depended on the retired SetSize hook. */
static CONFIG_INT("raw_vidx.resolution", raw_vid_resolution_mode, RAW_VID_RES_720P);
static CONFIG_INT("raw_vidx.codec", raw_vid_codec_mode, RAW_VID_CODEC_RAW);
/* v111: Still LV and Movie share the normal REC / output settings.
 * No test-frame selector is stored in configuration. */

int raw_vid_get_requested_bpp(void)
{
    static const int bpp_by_mode[] = { 14, 12, 10 };
    int mode = raw_vid_bpp_mode;
    if (mode < 0 || mode >= (int)COUNT(bpp_by_mode))
        mode = 0;
    return bpp_by_mode[mode];
}

int raw_vid_get_requested_le16(void)
{
    return 0;
}

int raw_vid_get_bpp(void)
{
    return raw_vid_active_bpp;
}

void raw_vid_set_active_bpp(int bpp)
{
    raw_vid_active_bpp = (bpp == 10 || bpp == 12 || bpp == 14) ? bpp : 14;
}

int raw_vid_get_requested_codec(void)
{
    int mode = raw_vid_codec_mode;
    if (mode < RAW_VID_CODEC_RAW || mode > RAW_VID_CODEC_CRAW)
        mode = RAW_VID_CODEC_RAW;
    return mode;
}

int raw_vid_get_requested_crx_frame_limit(void)
{
    /* Continuous is the production default; STOP is handled by the REC key,
     * normal recording control or existing writer/backend safety checks. */
    return 0;
}

/* v23.2 production UI exposes no CRX latch/session/still/tail controls.
 * Keep zero-return compatibility stubs while historical internal code remains
 * compiled but unreachable from normal REC. */
int raw_vid_get_requested_crx_latch_stop_mode(void) { return RAW_VID_CRX_LATCH_OFF; }
int raw_vid_get_requested_crx_session_budget(void) { return 0; }
int raw_vid_get_requested_crx_stillwatch(void) { return 0; }
int raw_vid_get_requested_crx_id0_still(void) { return 0; }
int raw_vid_get_crx_tail_audit(void) { return 0; }
int raw_vid_get_native_crx_watch_mode(void) { return 0; }
void raw_vid_disarm_native_crx_watch(int expected_mode) { (void)expected_mode; }

int raw_vid_get_requested_resolution_mode(void)
{
    int mode = raw_vid_resolution_mode;
    if (mode < RAW_VID_RES_720P || mode > RAW_VID_RES_W3_WIDE_720)
        mode = RAW_VID_RES_720P;
    return mode;
}

int raw_vid_resolve_capture_area(int mode, uint32_t *x, uint32_t *y,
                                 uint32_t *w, uint32_t *h)
{
    if (!x || !y || !w || !h)
        return -1;

    switch (mode)
    {
        case RAW_VID_RES_720P:
            *x = RP_RAW_VID_CROP_X;
            *y = RP_RAW_VID_CROP_Y;
            *w = 1280;
            *h = 720;
            break;

        case RAW_VID_RES_1080P:
            *x = RP_RAW_VID_CROP_X;
            *y = RP_RAW_VID_CROP_Y;
            *w = 1920;
            *h = 1080;
            break;

        case RAW_VID_RES_1440P:
            *x = RP_RAW_VID_CROP_X;
            *y = RP_RAW_VID_CROP_Y;
            *w = 2560;
            *h = 1440;
            break;

        case RAW_VID_RES_W3_TRANSPORT:
            *x = 0; *y = 0;
            *w = (uint32_t)raw_info.width;
            *h = (uint32_t)raw_info.height;
            break;

        case RAW_VID_RES_W3_WIDE_720:
            /* Sample all horizontal W3 photosites and take a vertically
             * centered, Bayer-aligned 720-row window.  This changes the
             * stored ROI only: it does NOT create a different sensor mode. */
            if (raw_info.width <= 0 || raw_info.height < 720)
                return -6;
            *w = (uint32_t)raw_info.width;
            *h = 720;
            *x = 0;
            *y = ((uint32_t)raw_info.height - 720u) / 2u;
            *y &= ~1u;
            break;

        case RAW_VID_RES_FULL:
        {
            /* EOS RP W3 includes optical-black margins at the left/top.
             * Two independent Full-profile CRX files (2128x1208 and
             * 3728x2052 physical W3) report the same decoded active-area
             * origin x=120,y=44 in raw_info.  Use the runtime metadata
             * rather than hard-coding a physical W3 size so Full means the
             * complete active image, not the transport buffer including OB. */
            int x1 = raw_info.active_area.x1;
            int y1 = raw_info.active_area.y1;
            int x2 = raw_info.active_area.x2;
            int y2 = raw_info.active_area.y2;
            if (x1 < 0 || y1 < 0 ||
                x2 <= x1 || y2 <= y1 ||
                x2 > raw_info.width || y2 > raw_info.height)
            {
                return -5;
            }
            *x = (uint32_t)x1;
            *y = (uint32_t)y1;
            *w = (uint32_t)(x2 - x1);
            *h = (uint32_t)(y2 - y1);
            break;
        }

        default:
            return -2;
    }

    if (!*w || !*h || (*w & 7u) || (*h & 1u))
        return -3;
    if (*x + *w > (uint32_t)raw_info.width ||
        *y + *h > (uint32_t)raw_info.height)
        return -4;

    return 0;
}

/* Recording control is a normal UI action, not a Still LV test.
 * The task in event_pusher waits for the ML menu to close before START. */
static MENU_SELECT_FUNC(raw_vid_record_toggle_select)
{
    (void)priv; (void)delta;
    raw_vid_record_menu_toggle();
}

static struct menu_entry raw_vidx_menu[] =
{
    {
        .name = "RP RAW Start/Stop",
        .select = raw_vid_record_toggle_select,
        .help = "Start or stop RAW video in Movie or photo LiveView.",
        .help2 = "Prefer the physical REC key. If it is not forwarded in photo mode, press SET here and close the ML menu to start. Stop with REC or SET.",
        .depends_on = DEP_LIVEVIEW,
    },
    {
        .name = "RP RAW codec",
        .priv = &raw_vid_codec_mode,
        .max = RAW_VID_CODEC_CRAW,
        .choices = CHOICES("Uncompressed", "CRX lossless", "C-RAW lossy"),
        .help = "EOS RP RAW recorder payload format.",
        .help2 = "CRX lossless and C-RAW always use the v23 RAM-only W3 source backend plus the Canon AlgsMgr encoder. The recorder always chooses the first free compatible D-slot and reclaims only its exact ML-owned allocator tail after a clean STOP; BPP is carried independently in the descriptor.",
        .depends_on = DEP_LIVEVIEW,
    },
    {
        .name = "RP RAW resolution",
        .priv = &raw_vid_resolution_mode,
        .max = RAW_VID_RES_W3_WIDE_720,
        .choices = CHOICES("720p", "1080p", "1440p", "Full active", "Full W3", "W3 width x 720"),
        .help = "Capture crop (must fit actual RAW-LV W3 geometry).",
        .help2 = "Fixed crops use x=192,y=100. Full active excludes optical black. Full W3 includes sensor transport margins. W3 width x 720 uses all W3 columns and centered 720 rows. Unsupported dimensions are refused before capture, never clamped silently.",
        .depends_on = DEP_LIVEVIEW,
    },
    {
        .name = "RP RAW bit depth",
        .priv = &raw_vid_bpp_mode,
        .max = 2,
        .choices = CHOICES("14-bit", "12-bit", "10-bit"),
        .help = "Native Canon W3 / encoder sample precision (10/12/14-bit).",
        .help2 = "10/12-bit use the runtime-proven RAM-only Mem1 fake-width + cfg.b2 path; destinations use MNAV desc.offset latch steering. No owner/SetSize MMU hooks.",
        .depends_on = DEP_LIVEVIEW,
    },
};

static struct msg_queue *event_q = NULL;

static unsigned int init(void)
{
    if (!is_camera("RP", "1.6.0"))
        return CBR_RET_ERROR;

#ifdef RAW_VID_LOG
    static uint8_t *log_buf = NULL;
    log_buf = malloc(MIN_LOG_BUF_SIZE);
    char log_name[20] = {0};
    if (get_numbered_file_name("raw_v%d.log", 99, log_name, sizeof(log_name)) > -1)
        init_log(log_buf, MIN_LOG_BUF_SIZE, log_name);
#endif

    event_q = msg_queue_create("raw_vid_q", RAW_VID_Q_LEN);
    if (!event_q)
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 INIT fail:event_queue");
        return CBR_RET_ERROR;
    }

    if (init_event_pusher(event_q))
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 INIT fail:event_pusher");
        return CBR_RET_ERROR;
    }

    if (init_worker(event_q))
    {
        DryosDebugMsg(0, 15, "RPVIDX REC111 INIT fail:worker");
        return CBR_RET_ERROR;
    }
    DryosDebugMsg(0, 15, "RPVIDX REC111 INIT ready");

    /* Task persists like the worker; observe only, no sensor writes. */
    raw_vid_still_watch_start();
    menu_add("Movie", raw_vidx_menu, COUNT(raw_vidx_menu));
    return 0;
}

static unsigned int deinit(void)
{
    /* raw_v_worker and the RP READY task are persistent once created. Until
     * this experimental branch has a task-destroy protocol, unloading the .mo
     * in place would leave tasks executing freed module code. */
    return CBR_RET_ERROR;
}

MODULE_INFO_START()
    MODULE_INIT(init)
    MODULE_DEINIT(deinit)
MODULE_INFO_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(raw_vid_bpp_mode)
    MODULE_CONFIG(raw_vid_resolution_mode)
    MODULE_CONFIG(raw_vid_codec_mode)
MODULE_CONFIGS_END()
