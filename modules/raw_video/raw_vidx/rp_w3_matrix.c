/*
 * EOS RP 1.6.0 W3 / PU2 matrix, integrated with raw_vidx v22.0.
 * EXPERIMENTAL; unsupported hardware.  All nontrivial work is TASK context.
 * The existing source-ring / true Complete / dynamic RAW MLV writer handles
 * pixel bytes. This task only schedules REC/STOP, applies one Mem1 config
 * parameter via rp_backend, and writes text telemetry. It never touches
 * Canon's IRQ ownership / callback registration.
 *
 * Uncompressed MLV VIDF payload = native W3 transport bytes because internal
 * mode RAW_VID_RES_W3_TRANSPORT uses x=y=0, w/h=entire physical W3.
 * Separate, sequential MLV files use unique 'wmNN.mlv' file numbers.
 */
#include "module.h"
#include "dryos.h"
#include "propvalues.h"
#include "raw_vid.h"
#include "raw_vid_event_pusher.h"
#include "raw_vid_worker.h"
#include "rp_backend.h"
#include "rp_w3_matrix.h"
#include <string.h>
#include <stdio.h>
#include "fio-ml.h"

/* 0: preserve the original cold-boot barriers before undocumented tests.
 * No-reboot mode is explicitly selected in the camera menu at runtime.
 * BOTH modes keep hardware/allocator/restore fail-closed guards. */
#define W3M_STATE_FILE_CHECKPOINT "W3MAT.STA"
#define W3M_STATE_FILE_ALL        "W3ALL.STA"
/* This flag is frozen at task launch; never change it between trials. */
static volatile int g_no_restart = 0;
static volatile int g_single_case = -1; /* -1=campaign; >=0=one isolated trial */
static volatile int g_pair_only = 0; /* A00->A01 diagnostic; separate checkpoint */
static const char *w3m_state_filename(void)
{
    return g_single_case >= 0 ? "NONE(single)" :
           g_pair_only ? "W3PAIR.STA" :
           g_no_restart ? W3M_STATE_FILE_ALL : W3M_STATE_FILE_CHECKPOINT;
}
#define W3M_STATE_MAGIC 0x574D4154u /* WMAT */
#define W3M_STATE_VERSION 1u
#define W3M_READY 1u
#define W3M_INPROGRESS 2u
#define W3M_NEEDS_REBOOT 3u
#define W3M_COMPLETE 4u
#define W3M_FAILED 5u
struct w3m_state {
    uint32_t magic, version, next_case, status, checksum;
};
static uint32_t w3m_state_checksum(const struct w3m_state *s)
{
    return s->magic ^ s->version ^ s->next_case ^ s->status ^ 0x7B946B1Au;
}
static int w3m_state_store(uint32_t next_case,uint32_t status)
{
    struct w3m_state s = {W3M_STATE_MAGIC,W3M_STATE_VERSION,next_case,status,0};
    s.checksum = w3m_state_checksum(&s);
    FILE *f = FIO_CreateFile(w3m_state_filename());
    if (!f) return -1;
    int n=FIO_WriteFile(f,&s,sizeof(s));
    FIO_CloseFile(f);
    return n==(int)sizeof(s)?0:-2;
}
static int w3m_state_load(uint32_t *next_case,uint32_t *status)
{
    if (!is_file(w3m_state_filename())) { *next_case=0; *status=W3M_READY;return 0; }
    FILE *f = FIO_OpenFile(w3m_state_filename(),O_RDONLY|O_SYNC);
    if (!f) return -3;
    struct w3m_state s;
    int n=FIO_ReadFile(f,&s,sizeof(s));
    FIO_CloseFile(f);
    if (n!=(int)sizeof(s) || s.magic!=W3M_STATE_MAGIC ||
        s.version!=W3M_STATE_VERSION ||
        s.checksum!=w3m_state_checksum(&s) || s.next_case>20 ||
        s.status<W3M_READY || s.status>W3M_FAILED) return -4;
    *next_case=s.next_case;
    *status=s.status;
    return 0;
}
/* A/B, C, D baseline, D03, E01, E02 are different cold-boot groups.
 * The state file remains after each group; re-launch the *same* menu entry
 * after a power-off boot. Never auto-resume on boot. */
static int w3m_group_boundary(int index)
{
    return index==10 || index==13 || index==16 || index==17 || index==18;
}
#define W3M_LEGACY_CASES 20
#define W3M_CASES 49
#define W3M_MAX_FRAMES 32
#define W3M_WAIT_START_MS 8000u
#define W3M_WAIT_STOP_MS 18000u
#define W3M_CAPTURE_MS 500u
#define W3M_FRAME_SOFT_LIMIT 12u
#define W3M_INTERCASE_COOLDOWN_MS 3500u
/* No preflight polling in the gap: each session START already preflights. */
#define W3M_STEP_US 15000u

#define W3M_LOG(...) DryosDebugMsg(0, 15, "W3MAT " __VA_ARGS__)

/* -1=keep Canon baseline. field maps to b0(0), b1(1), b4(4).
 * Every line captures the COMPLETE PHYSICAL W3 (not a crop). BPP config and
 * matching pitch are changed only by the proven raw_vid_rp_set_bit_depth().
 * Cases with changed unknown fields retain a 500ms/12-frame ceiling, and fail closed on
 * any capture/drop/restore error. No XMODE / PU+0x28 writes are attempted. */
struct w3m_case {
    const char *id;
    int bpp, linear, field, value;
    int unknown;
};
static const struct w3m_case w3m_cases[W3M_CASES] = {
    {"A00",14,0,-1,-1,0},
    {"A01",14,1,-1,-1,0},
    {"A02",14,0,-1,-1,0},
    {"A03",12,1,-1,-1,0},
    {"A04",10,1,-1,-1,0},
    {"B01",14,1, 1, 0,0},
    {"B02",14,1, 1, 1,0},
    {"B03",14,0, 1, 0,0},
    {"B04",14,0, 1, 1,0},
    {"B05",12,1, 1, 0,0},
    {"B06",10,1, 1, 0,0},
    {"C00",14,1, 0, 0,1},
    {"C01",14,1, 0, 1,1},
    {"C02",14,1, 0, 0,1},
    {"D00",14,1, 4, 0,1},
    {"D01",14,1, 4, 1,1},
    {"D02",14,1, 4, 0,1},
    {"D03",14,1, 4, 2,1},
    {"E01",12,1, 0, 1,1},
    {"E02",12,1, 4, 1,1},
    /* Research v9: SINGLE ONLY. These entries never extend W3MAT/ALL/PAIR.
     * A/B/A controls make each change interpretable. Unproven b4 modes use
     * a 16-bit-sized guard ring and a short 4-frame maximum. */
    {"U00",14,1, 4, 0,1}, /* RAW14 b4=0 pre-control */
    {"U01",14,1, 4, 3,1}, /* RAW14 b4=3 UNKNOWN; direct Single */
    {"U02",14,1, 4, 0,1}, /* RAW14 b4=0 post-control */
    {"U03",12,1, 4, 0,1}, /* RAW12 b4=0 pre-control */
    {"U04",12,1, 4, 2,1}, /* RAW12 b4=2 */
    {"U05",12,1, 4, 3,1}, /* RAW12 b4=3 UNKNOWN; direct Single */
    {"U06",12,1, 4, 0,1}, /* RAW12 b4=0 post-control */
    {"U07",10,1, 4, 0,1}, /* RAW10 b4=0 pre-control */
    {"U08",10,1, 4, 1,1}, /* RAW10 b4=1 */
    {"U09",10,1, 4, 0,1}, /* RAW10 b4=0 post-control */
    {"U10",12,1, 1, 1,0}, /* RAW12 b1=1; compare old B05 b1=0 */
    {"U11",10,1, 1, 1,0}, /* RAW10 b1=1; compare old B06 b1=0 */
    /* v10: Single ONLY. Existing cases and numbering preserved.
     * V00..V05: bright-scene b4=3/RAW14 vs b4=3/RAW12 paired probes.
     * V06..V10: RAW10 b4=2/3 with bracketing controls.
     * V11..V16: same-session b1 A/B/A for RAW12 and RAW10. */
    {"V00",14,1, 4, 3,1}, /* bright RAW14 b4=3 reference; experimental */
    {"V01",12,1, 4, 3,1}, /* bright RAW12 b4=3 (possible overflow); experimental */
    {"V02",14,1, 4, 3,1}, /* bright RAW14 b4=3 return reference; experimental */
    {"V03",12,1, 4, 0,1}, /* bright RAW12 control */
    {"V04",12,1, 4, 3,1}, /* bright RAW12 b4=3 replicate; experimental */
    {"V05",12,1, 4, 0,1}, /* bright RAW12 return control */
    {"V06",10,1, 4, 0,1}, /* RAW10 b4=0 reference */
    {"V07",10,1, 4, 2,1}, /* RAW10 b4=2 unknown; experimental */
    {"V08",10,1, 4, 0,1}, /* RAW10 b4=0 return reference */
    {"V09",10,1, 4, 3,1}, /* RAW10 b4=3 unknown; experimental */
    {"V10",10,1, 4, 0,1}, /* RAW10 b4=0 second return reference */
    {"V11",12,1, 1, 0,0}, /* RAW12 b1 A=0 */
    {"V12",12,1, 1, 1,0}, /* RAW12 b1 B=1 */
    {"V13",12,1, 1, 0,0}, /* RAW12 b1 A=0 return */
    {"V14",10,1, 1, 0,0}, /* RAW10 b1 A=0 */
    {"V15",10,1, 1, 1,0}, /* RAW10 b1 B=1 */
    {"V16",10,1, 1, 0,0}, /* RAW10 b1 A=0 return */
};

struct w3m_frame {
    uint32_t us;
    uint32_t frame;
    int slot;
    uint32_t owner, complete;
};

/* Producer: matrix task. Consumer: existing ML worker.  Single run at a time. */
static volatile int g_running = 0;
/* Once a trial fails, all ML REC is blocked until a real cold boot. */
static volatile int g_interlock = 0;
static volatile int g_abort = 0;
static volatile int g_index = -1;
static volatile int g_apply_rc = -999;
static volatile int g_restore_rc = -999;
static volatile int g_stop_clean = -999;
static volatile int g_stop_transfer_rc = -999;
static volatile int g_stop_drain_rc = -999;
static volatile int g_stop_frame_count = 0;
static struct rp_w3_transport_plan g_wireplan;
static int g_wireplan_valid = 0;
static volatile int g_writer_io = 0;
static volatile int g_writer_post = 0;
static volatile uint32_t g_frames = 0;
static struct w3m_frame g_meta[W3M_MAX_FRAMES];
static struct rp_w3_hw_snapshot g_pre, g_applied, g_armed, g_during_end, g_before_off,
                                  g_restored, g_mid, g_last;
static volatile int g_has_mid = 0, g_has_last = 0;
static char g_video_name[24];
static FILE *g_log = NULL;
static volatile int g_log_error=0;
static void w3m_text(const char *line); /* task-only writer, defined below */
/* TASK-only producer (worker), TASK-only consumer (campaign runner).
 * No I/O in producer and never touches the E00D/E057 IRQ path. */
#define W3M_STAGE_CAP 96u
struct w3m_stage_event { volatile uint32_t ms; volatile int code, detail; };
static struct w3m_stage_event g_stage_events[W3M_STAGE_CAP];
static volatile uint32_t g_stage_count = 0u;
static uint32_t g_stage_read = 0u;

void rp_w3_matrix_stage(int code, int detail)
{
    if (!rp_w3_matrix_case_active()) return;
    uint32_t k = g_stage_count;
    if (k >= W3M_STAGE_CAP) return;
    g_stage_events[k].ms = get_ms_clock();
    g_stage_events[k].code = code;
    g_stage_events[k].detail = detail;
    __asm__ volatile ("dmb sy" ::: "memory");
    g_stage_count = k + 1u;
}
static const char *w3m_stage_label(int code)
{
    switch (code) {
    case 20: return "MLV_FILE_OPEN";
    case 30: return "MLV_SESSION_STARTED";
    case 40: return "MLV_HEADER_WRITTEN";
    case 50: return "OWNER_PREPARE_BEGIN";
    case 51: return "OWNER_PREPARE_END";
    case 60: return "BPP_APPLY_BEGIN";
    case 61: return "BPP_APPLY_END";
    case 70: return "LINEAR_APPLY_BEGIN";
    case 71: return "LINEAR_APPLY_END";
    case 72: return "PU_MATRIX_APPLY_BEGIN";
    case 73: return "PU_MATRIX_APPLY_END";
    case 90: return "SOURCE_RING_PREPARE_BEGIN";
    case 91: return "SOURCE_RING_PREPARE_END";
    case 100: return "W3_ARM_BEGIN";
    case 101: return "W3_ARM_END";
    case 110: return "RECORDING_ACTIVE";
    case 200: return "STOP_BEGIN";
    case 210: return "PU_BPP_RESTORE_BEGIN";
    case 211: return "PU_BPP_RESTORE_END";
    case 220: return "RAW_LV_RELEASE_BEGIN";
    case 221: return "RAW_LV_RELEASE_END";
    case 250: return "STOP_INACTIVE";
    default: return "OTHER";
    }
}
static void w3m_stage_drain(void)
{
    char b[160];
    __asm__ volatile ("dmb sy" ::: "memory");
    while (g_stage_read < g_stage_count && g_stage_read < W3M_STAGE_CAP)
    {
        uint32_t k = g_stage_read++;
        int code = g_stage_events[k].code;
        int detail = g_stage_events[k].detail;
        unsigned ms = g_stage_events[k].ms;
        snprintf(b,sizeof(b),"STAGE #%d ms=%d code=%d name=%s detail=%d\n",
                 (int)k,(int)ms,code,w3m_stage_label(code),detail);
        w3m_text(b);
    }
}


static uint32_t now_ms(void) { return get_ms_clock(); }
static uint32_t elapsed_ms(uint32_t base) { return now_ms() - base; }

/* Only matrix task writes log file. Worker logs to rpconlog. */
static void w3m_text(const char *line)
{
    if (g_log && line)
        if (FIO_WriteFile(g_log, line, (uint32_t)strlen(line)) !=
            (int)strlen(line)) g_log_error=1;
}

static void w3m_snapshot_text(const char *label,
                              const struct rp_w3_hw_snapshot *s)
{
    char b[512];
    snprintf(b,sizeof(b),
      "%s state=%08x cfg=%08x,%08x,%08x,%08x,%08x "
      "pu20=%08x pu24=%08x pu28=%08x w350=%08x w3a0=%08x w3d0=%08x "
      "cb=%08x arg=%08x owner=%d complete=%d drops=%d overlap=%d "
      "bad=%d post=%d fatal=%d\n",
      label,(int)s->mem1_state,(int)s->cfg[0],(int)s->cfg[1],
      (int)s->cfg[2],(int)s->cfg[3],(int)s->cfg[4],
      (int)s->pu20,(int)s->pu24,(int)s->pu28,(int)s->w350,
      (int)s->w3a0,(int)s->w3d0,(int)s->cb,(int)s->cb_arg,
      (int)s->owner,(int)s->complete,(int)s->drops,(int)s->overlap,
      (int)s->bad_complete,(int)s->post_fail,(int)s->fatal);
    w3m_text(b);
}

/* Register differences from ALREADY CAPTURED task-context snapshots.
 * This performs no new MMIO accesses or IRQ logging. PU24 changed bits are
 * not interpreted as functionality without an accompanying pixel analysis. */
static void w3m_delta_text(const char *name,
                          const struct rp_w3_hw_snapshot *a,
                          const struct rp_w3_hw_snapshot *b)
{
    char line[320];
    snprintf(line,sizeof(line),
        "REGDELTA %s pu20=%08x pu24=%08x pu28=%08x "
        "w350=%08x w3a0=%08x w3d0=%08x "
        "cfg=%08x,%08x,%08x,%08x,%08x\n",
        name,
        (int)(a->pu20^b->pu20),(int)(a->pu24^b->pu24),
        (int)(a->pu28^b->pu28),(int)(a->w350^b->w350),
        (int)(a->w3a0^b->w3a0),(int)(a->w3d0^b->w3d0),
        (int)(a->cfg[0]^b->cfg[0]),(int)(a->cfg[1]^b->cfg[1]),
        (int)(a->cfg[2]^b->cfg[2]),(int)(a->cfg[3]^b->cfg[3]),
        (int)(a->cfg[4]^b->cfg[4]));
    w3m_text(line);
}

int rp_w3_matrix_running(void) { return g_running != 0; }
int rp_w3_matrix_interlocked(void) { return g_interlock != 0; }
int rp_w3_matrix_case_active(void) { return g_running && g_index >= 0; }
int rp_w3_matrix_requires_linear(void)
{
    return rp_w3_matrix_case_active() ? w3m_cases[g_index].linear : 0;
}
int rp_w3_matrix_field_index(void)
{
    return rp_w3_matrix_case_active() ? w3m_cases[g_index].field : -1;
}
int rp_w3_matrix_field_value(void)
{
    return rp_w3_matrix_case_active() ? w3m_cases[g_index].value : -1;
}
void rp_w3_matrix_abort_user(void)
{
    if (g_running)
    {
        g_abort = 1;
        W3M_LOG("USER ABORT requested");
    }
}
const char *rp_w3_matrix_video_pattern(void)
{
    return g_single_case >= 0 ? "ws%02d.mlv" :
           g_pair_only ? "wp%02d.mlv" :
           g_no_restart ? "wa%02d.mlv" : "wm%02d.mlv";
}
void rp_w3_matrix_file(const char *name)
{
    if (!rp_w3_matrix_case_active()) return;
    snprintf(g_video_name, sizeof(g_video_name), "%s", name ? name : "");
    W3M_LOG("VIDEO case=%s file=%s", w3m_cases[g_index].id, g_video_name);
}
void rp_w3_matrix_capture_pre(void)
{
    if (rp_w3_matrix_case_active()) raw_vid_rp_matrix_snapshot(&g_pre);
}
void rp_w3_matrix_apply_result(int rc)
{
    if (!rp_w3_matrix_case_active()) return;
    g_apply_rc = rc;
    raw_vid_rp_matrix_snapshot(&g_applied);
    /* The source sample BPP and the W3 wire container width are distinct.
     * This is informational: NO descriptor change is performed here. */
    struct rp_w3_transport_plan p;
    int plan_rc = raw_vid_rp_transport_get_plan(&p);
    g_wireplan_valid = plan_rc == 0;
    if (g_wireplan_valid) g_wireplan = p;
    if (!plan_rc)
    {
        W3M_LOG("WIREPLAN case=%s sample=%d wire=%d nominal=%d/%d target=%d/%d seen_pitch=%d hw_ready=%d",
                w3m_cases[g_index].id,(int)p.sample_bpp,(int)p.wire_bpp,
                (int)p.nominal_pitch,(int)p.nominal_bytes,
                (int)p.target_pitch,(int)p.target_bytes,
                (int)p.observed_pitch,(int)p.hw_full_width);
    }
    else W3M_LOG("WIREPLAN FAIL case=%s rc=%d",w3m_cases[g_index].id,plan_rc);
    W3M_LOG("APPLIED id=%s rc=%d pu=%x cfg=%x,%x,%x,%x,%x",
       w3m_cases[g_index].id,rc,(int)g_applied.pu24,
       (int)g_applied.cfg[0],(int)g_applied.cfg[1],(int)g_applied.cfg[2],
       (int)g_applied.cfg[3],(int)g_applied.cfg[4]);
}
void rp_w3_matrix_on_arm(void)
{
    if (rp_w3_matrix_case_active())
    {
        raw_vid_rp_matrix_snapshot(&g_armed);
        struct rp_w3_transport_plan p;
        int plan_rc = raw_vid_rp_transport_get_plan(&p);
        if (!plan_rc)
        {
            g_wireplan = p;
            g_wireplan_valid = 1;
            W3M_LOG("WIREPLAN_ARM sample=%d wire=%d src_cap=%d target=%d hw_full=%d alloc_full=%d",
                    (int)p.sample_bpp,(int)p.wire_bpp,(int)p.source_capacity,
                    (int)p.target_bytes,(int)p.hw_full_width,(int)p.buffer_full_width);
        }
        /* Historical ARM line is retained for baseline log parsers. */
        W3M_LOG("ARM_BASE id=%s pu=%x pitch=%d",w3m_cases[g_index].id,
                (int)g_armed.pu24, (int)(g_armed.w350 & 0xffffu));
    }
}
void rp_w3_matrix_capture_end(void)
{
    if (!rp_w3_matrix_case_active()) return;
    raw_vid_rp_matrix_snapshot(&g_during_end);
    W3M_LOG("END_BEFORE_RESTORE case=%s PU24=%x owner=%d complete=%d",
       w3m_cases[g_index].id,(int)g_during_end.pu24,
       (int)g_during_end.owner,(int)g_during_end.complete);
}
void rp_w3_matrix_on_frame(uint32_t frame, int slot, uint32_t us)
{
    if (!rp_w3_matrix_case_active()) return;
    if (g_frames < W3M_MAX_FRAMES)
    {
        g_meta[g_frames].frame=frame;
        g_meta[g_frames].slot=slot;
        g_meta[g_frames].us=us;
        raw_vid_rp_matrix_counts(&g_meta[g_frames].owner,
                                  &g_meta[g_frames].complete);
    }
    g_frames=frame;
}
void rp_w3_matrix_restore_result(int rc)
{
    if (!rp_w3_matrix_case_active()) return;
    g_restore_rc = rc;
    raw_vid_rp_matrix_snapshot(&g_restored);
    W3M_LOG("RESTORE id=%s rc=%d",w3m_cases[g_index].id,rc);
}
void rp_w3_matrix_pre_cleanup(int stop_rc,int drain_rc,int frames,
                              int io_fail,int post_fail)
{
    if (!rp_w3_matrix_case_active()) return;
    raw_vid_rp_matrix_snapshot(&g_before_off);
    g_stop_transfer_rc=stop_rc;
    g_stop_drain_rc=drain_rc;
    g_stop_frame_count=frames;
    g_writer_io=io_fail;
    g_writer_post=post_fail;
}
void rp_w3_matrix_storage_status(int io_fail,int post_fail)
{
    if (!rp_w3_matrix_case_active()) return;
    g_writer_io=io_fail;g_writer_post=post_fail;
}

void rp_w3_matrix_on_stop(int cleanup_rc,int stop_rc,int drain_rc)
{
    if (!rp_w3_matrix_case_active()) return;
    g_stop_clean=cleanup_rc;
    g_stop_transfer_rc=stop_rc;
    g_stop_drain_rc=drain_rc;
    W3M_LOG("STOP id=%s frames=%d clean=%d stop=%d drain=%d drop=%d fatal=%d",
      w3m_cases[g_index].id,g_stop_frame_count,cleanup_rc,stop_rc,drain_rc,
      (int)g_before_off.drops,(int)g_before_off.fatal);
}

static int w3m_case_ok(void)
{
    if (g_apply_rc || g_restore_rc || g_stop_clean || g_stop_transfer_rc ||
        g_stop_drain_rc || g_writer_io || g_writer_post ||
        g_stop_frame_count < 2 || g_before_off.drops ||
        g_before_off.overlap || g_before_off.bad_complete ||
        g_before_off.post_fail || g_before_off.fatal)
        return 0;
    if (g_applied.mem1_state == 0u || g_armed.mem1_state == 0u ||
        g_pre.mem1_state != g_applied.mem1_state ||
        g_before_off.cb != 0xE037951Du || g_before_off.cb_arg != 0u ||
        g_armed.w3a0 == 0u ||
        g_during_end.w3a0 == 0u ||
        g_restored.mem1_state != g_pre.mem1_state)
        return 0;
    const struct w3m_case *c=&w3m_cases[g_index];
    /* Mode follows the selected BPP and linear packing. Do not impose RAW14
     * / PIXBLOCK checks on 12/10-bit and LE serializations. */
    const uint32_t expected_mode = c->bpp == 14 ? 2u :
                                   c->bpp == 12 ? 1u : 0u;
    if (c->bpp == 14)
    {
        if ((g_during_end.cfg[2] & 0xffu) != 2u &&
            (g_during_end.cfg[2] & 0xffu) != 3u) return 0;
    }
    else if ((g_during_end.cfg[2] & 0xffu) != expected_mode) return 0;
    if ((g_during_end.cfg[3] & 0xffu) != (uint32_t)c->linear)
        return 0;
    for (int i=0;i<5;++i)
    {
        /* The apply routine must change ONE logical field's low byte only. */
        if ((g_applied.cfg[i] & ~0xffu) != (g_pre.cfg[i] & ~0xffu))
            return 0;
        if (i == c->field)
        {
            if ((g_applied.cfg[i] & 0xffu) != (uint32_t)c->value)
                return 0;
        }
        else if (g_applied.cfg[i] != g_pre.cfg[i])
            return 0;
    }
    /* Restore is observed before RAW-LV release, after Canon BPP/serializer
     * restore. Ensure native RAW14 + PIXBLOCK are back, not still experimental. */
    if (((g_restored.cfg[2] & 0xffu) != 2u &&
         (g_restored.cfg[2] & 0xffu) != 3u) ||
        (g_restored.cfg[3] & 0xffu) != 0u)
        return 0;
    if (!g_restored.mem1_state ||
        g_restored.cfg[0] != g_pre.cfg[0] ||
        g_restored.cfg[1] != g_pre.cfg[1] ||
        g_restored.cfg[4] != g_pre.cfg[4])
        return 0;
    /* Before restoration the Canon task should still have the tested values;
     * these checks use END_BEFORE_RESTORE, not LAST_BEFORE_RAW_OFF. */
    if (g_during_end.cfg[0] != g_applied.cfg[0] ||
        g_during_end.cfg[1] != g_applied.cfg[1] ||
        g_during_end.cfg[4] != g_applied.cfg[4])
        return 0;
    return 1;
}

/* v4 diagnostic: emulate the proven manual Single-to-Single transition.
 * STOP already drains and closes its MLV; caller checked writer.idle.
 * Avoid 10-Hz queue polling, persistent preflight traffic, and any MMIO
 * after raw_lv_release(). A passive wait does NOT guarantee Canon teardown;
 * it intentionally changes only the inter-session orchestration path.
 * START worker still performs its ordinary preflight and fail-closed guards. */
static int w3m_wait_inter_session_ready(void)
{
    char b[160];
    snprintf(b,sizeof(b),
        "INTERCASE PASSIVE_BEGIN ms=%d no_poll=1 no_preflight=1\n",
        (int)W3M_INTERCASE_COOLDOWN_MS);
    w3m_text(b);
    W3M_LOG("INTERCASE PASSIVE_BEGIN ms=%d", (int)W3M_INTERCASE_COOLDOWN_MS);
    msleep(W3M_INTERCASE_COOLDOWN_MS);
    if (g_abort || !lv || get_recording_state()!=INACTIVE)
    {
        snprintf(b,sizeof(b),"INTERCASE PASSIVE_FAIL lv=%d state=%d abort=%d\n",
                 (int)lv,(int)get_recording_state(),(int)g_abort);
        w3m_text(b);
        return -1;
    }
    /* One final writer check; unlike the v3 10-Hz loop this does not
     * continuously exercise DryOS message-queue internals between sessions. */
    struct raw_vid_rp_writer_status st;
    raw_vid_rp_writer_get_status(&st);
    snprintf(b,sizeof(b),
        "INTERCASE PASSIVE_END writer_idle=%d queued=%d done=%d q=%d file=%d io=%d post=%d\n",
        st.idle,(int)st.queued,(int)st.done,(int)st.queue_depth,
        st.file_open,(int)st.io_fail,(int)st.post_fail);
    w3m_text(b);
    W3M_LOG("INTERCASE PASSIVE_END idle=%d", st.idle);
    return st.idle ? 0 : -2;
}
static void w3m_runner(void *unused)
{
    (void)unused;
    int failed=0;
    char name[24]={0},b[512];
    uint32_t first=0,status=0;
    const int run_limit = g_pair_only ? 2 :
                          g_single_case >= 0 ? W3M_CASES : W3M_LEGACY_CASES;
    if (g_single_case < 0 && (w3m_state_load(&first,&status) ||
        (g_no_restart && first != 0u) ||
        (status!=W3M_READY && status!=W3M_NEEDS_REBOOT) ||
        first >= (uint32_t)run_limit))
    {
        W3M_LOG("REFUSE state not restartable file=%s; inspect/archive it on PC", w3m_state_filename());
        g_interlock=1;g_running=0;return;
    }
    if (g_single_case >= 0) first=(uint32_t)g_single_case;
    if (get_numbered_file_name((g_single_case >= 0 ?
            "w3s%02d.txt" :
                     g_pair_only ? "w3p%02d.txt" :
                     g_no_restart ? "w3a%02d.txt" : "w3m%02d.txt"),99,name,sizeof(name)) < 0)
    { W3M_LOG("ABORT no log filename");g_interlock=1;g_running=0;return; }
    g_log=FIO_CreateFile(name);
    if (!g_log) { W3M_LOG("ABORT log open name=%s",name);g_interlock=1;g_running=0;return; }
    g_log_error=0;
    snprintf(b,sizeof(b),"W3MAT v10; EOS RP 1.6.0; source raw_vidx v22.0; log=%s\n"
      "EXPERIMENTAL/UNTESTED: 20 legacy + 29 Single-only; max 500ms/12 frames; "
      "W3 physical transport uncompressed MLV. Values are runtime, not ROM.\n",name);
    w3m_text(b);
    W3M_LOG("BEGIN log=%s first=%d cases=%d no_restart=%d",
            name,(int)first,run_limit,g_no_restart);
    snprintf(b,sizeof(b),"STATE first_case=%d resume_status=%d state_file=%s no_restart=%d single=%d\n",
             (int)first,(int)status,w3m_state_filename(),(int)g_no_restart,(int)g_single_case);
    w3m_text(b);
    int checkpoint=0;
    for (int i=(int)first;i<(g_single_case >= 0 ? (int)first+1 : run_limit);++i)
    {
        if (g_abort || !lv || get_recording_state()!=INACTIVE)
        { failed=-1; break; }
        /* Conservative remaining-card-space gate, re-evaluated per test.
         * 16k units of 32 KiB = 512 MiB, intentionally ample headroom
         * beyond the ~54 MB expected for 12 RAW14 frames.
         * 0 denotes unavailable/unknown information, not zero bytes. */
        const struct card_info *card=get_shooting_card();
        int free32=card?get_free_space_32k(card):0;
        if (free32<=0)
            W3M_LOG("WARN free-space measurement unknown; do not rely on card capacity gate");
        if (free32>0 && free32<16384)
        { failed=-70;W3M_LOG("ABORT LOW CARD FREE units32k=%d",free32);break; }
        const struct w3m_case *c=&w3m_cases[i];
        /* b0=1 potentially changes the payload to 16-bit words. Require
         * explicit Single test until guard measurements validate the bound. */
        if (c->field==0 && c->value==1 && g_single_case<0)
        {
            failed=-72;
            w3m_text("ABORT B0=1: use explicit SINGLE guarded trial (C01/E01); no ALL until source span is verified\n");
            W3M_LOG("ABORT b0=1 requires SINGLE case=%s",c->id);
            break;
        }
        /* Crash or battery removal during a run leaves INPROGRESS, not
         * a false PASS. Never skip unknown hardware failure automatically. */
        if (g_single_case < 0 && w3m_state_store((uint32_t)i,W3M_INPROGRESS))
        { failed=-90; W3M_LOG("ABORT checkpoint write failed");break; }
        g_index=i;
        g_apply_rc=-999; g_restore_rc=-999;g_stop_clean=-999;
        g_stop_transfer_rc=-999;g_stop_drain_rc=-999;g_stop_frame_count=0;
        g_writer_io=0;g_writer_post=0;g_frames=0;
        g_wireplan_valid=0;
        memset(&g_wireplan,0,sizeof(g_wireplan));
        g_has_mid=0;g_has_last=0;
        g_stage_count=0;g_stage_read=0;
        memset(g_meta,0,sizeof(g_meta));
        memset(&g_pre,0,sizeof(g_pre));
        memset(&g_applied,0,sizeof(g_applied));
        memset(&g_armed,0,sizeof(g_armed));
        memset(&g_during_end,0,sizeof(g_during_end));
        memset(&g_before_off,0,sizeof(g_before_off));
        memset(&g_restored,0,sizeof(g_restored));
        g_video_name[0]=0;
        snprintf(b,sizeof(b),"\nRUN_BEGIN %s ordinal=%d/%d bpp=%d linear=%d field=%d value=%d unknown=%d\n",
                 c->id,i+1,run_limit,c->bpp,c->linear,c->field,c->value,c->unknown);
        w3m_text(b);
        W3M_LOG("RUN_BEGIN id=%s no=%d/%d bpp=%d linear=%d field=%d value=%d",
                c->id,i+1,run_limit,c->bpp,c->linear,c->field,c->value);
        w3m_text("STAGE ARM_REQUEST (scheduler posting COMMAND_START)\n");
        int start_rc=rp_w3_matrix_send_start(c->bpp);
        w3m_stage_drain();
        if (start_rc)
        { failed=-100+start_rc; W3M_LOG("START POST FAIL id=%s rc=%d",c->id,start_rc); break; }
        uint32_t t0=now_ms();
        while (get_recording_state()!=ACTIVE && elapsed_ms(t0)<W3M_WAIT_START_MS)
        {
            if (g_abort) break;
            w3m_stage_drain();
            msleep(20);
        }
        w3m_stage_drain();
        if (get_recording_state()!=ACTIVE)
        {
            /* Unknown partial start ownership: do NOT auto-start another case. */
            failed=-110;
            if (get_recording_state()==ACTIVE) raw_vid_request_stop_async();
            W3M_LOG("ABORT start timeout id=%s apply=%d",c->id,g_apply_rc);
            w3m_text("ABORT START TIMEOUT: manual cold boot required\n");
            break;
        }
        t0=now_ms();
        W3M_LOG("CAPTURE id=%s target_ms=%d",c->id,W3M_CAPTURE_MS);
        const int short_trial = (c->field==0 && c->value==1) ||
                                (g_single_case >= 20 && c->field==4);
        const uint32_t capture_ms = short_trial ? 180u : W3M_CAPTURE_MS;
        const uint32_t frame_limit = short_trial ? 4u : W3M_FRAME_SOFT_LIMIT;
        while (elapsed_ms(t0)<capture_ms && !g_abort)
        {
            w3m_stage_drain();
            if (g_frames >= 6 && !g_has_mid)
            {
                raw_vid_rp_matrix_snapshot(&g_mid);
                g_has_mid=1;
            }
            if (g_frames>=frame_limit) break;
            struct rp_w3_hw_snapshot s;
            raw_vid_rp_matrix_snapshot(&s);
            if (s.drops || s.overlap || s.bad_complete || s.post_fail || s.fatal)
            { failed=-120; break; }
            msleep(20);
        }
        /* Important: capture may have stopped automatically due to a slot
         * starvation, so STOP must be idempotent at this boundary. */
        if (get_recording_state()==ACTIVE)
            raw_vid_request_stop_async();
        t0=now_ms();
        /* Wait for both state transition AND worker's STOP acknowledgement.
         * A stale/missing callback must not be mislabeled as an elapsed timeout. */
        while ((get_recording_state()!=INACTIVE || g_stop_clean==-999) &&
               elapsed_ms(t0)<W3M_WAIT_STOP_MS)
        {
            w3m_stage_drain();
            msleep(30);
        }
        w3m_stage_drain();
        if (get_recording_state()!=INACTIVE || g_stop_clean==-999)
        {
            failed=-130;
            W3M_LOG("ABORT stop incomplete id=%s state=%d clean=%d running=%d index=%d elapsed_ms=%d",
                    c->id,(int)get_recording_state(),g_stop_clean,
                    (int)g_running,(int)g_index,(int)elapsed_ms(t0));
            break;
        }
        /* As INACTIVE follows drain + MLV close, these represent the actual
         * finished FIO batch timing of this case (not nominal video rate). */
        struct raw_vid_rp_writer_status writer;
        raw_vid_rp_writer_get_status(&writer);
        snprintf(b,sizeof(b),
           "FIO queued=%d done=%d q=%d file_open=%d idle=%d pending=%d "
           "io=%d post=%d batches=%d max_batch_kb=%d max_write_ms=%d "
           "batch_rate_mb_s_x100=%d\n",
           (int)writer.queued,(int)writer.done,
           (int)writer.queue_depth,writer.file_open,writer.idle,
           (int)writer.header_pending,(int)writer.io_fail,
           (int)writer.post_fail,(int)writer.batches,
           (int)(writer.max_batch_bytes/1024u),
           (int)(writer.max_write_us/1000u),
           (int)writer.rate_mb_s_x100);
        w3m_text(b);
        W3M_LOG("FIO case=%s idle=%d queued=%d done=%d rate=%d.%02d MB/s",
                c->id,writer.idle,(int)writer.queued,(int)writer.done,
                (int)(writer.rate_mb_s_x100/100u),
                (int)(writer.rate_mb_s_x100%100u));
        if (!writer.idle)
        {
            failed=-131;
            W3M_LOG("ABORT WRITER NOT IDLE after %s",c->id);
            w3m_text("ABORT WRITER NOT IDLE AFTER STOP; cold boot required\n");
            break;
        }
        /* Snapshot named LAST must be taken before RAW-LV teardown; the
         * worker captured g_before_off while the MMIO registers were live. */
        g_last=g_before_off; g_has_last=1;
        /* No MMIO or raw_info reads after RAW-LV is released. The plan was
         * snapshotted in task context during the start/arm phases. */
        if (g_wireplan_valid)
        {
            snprintf(b,sizeof(b),
                "WIREPLAN wh=%d,%d sample_bpp=%d wire_bpp=%d nominal_pitch=%d nominal_bytes=%d target_pitch=%d target_bytes=%d\n",
                (int)g_wireplan.width,(int)g_wireplan.height,
                (int)g_wireplan.sample_bpp,(int)g_wireplan.wire_bpp,
                (int)g_wireplan.nominal_pitch,(int)g_wireplan.nominal_bytes,
                (int)g_wireplan.target_pitch,(int)g_wireplan.target_bytes);
            w3m_text(b);
            snprintf(b,sizeof(b),
                "WIREBUF observed_pitch=%d source_cap=%d full_hw_at_arm=%d alloc_full=%d status=%s\n",
                (int)g_wireplan.observed_pitch,(int)g_wireplan.source_capacity,
                (int)g_wireplan.hw_full_width,(int)g_wireplan.buffer_full_width,
                "MODEL_ONLY");
        }
        else
            snprintf(b,sizeof(b),"WIREPLAN status=MODEL_UNAVAILABLE\n");
        w3m_text(b);
        w3m_snapshot_text("PRE",&g_pre);
        w3m_snapshot_text("APPLIED",&g_applied);
        w3m_snapshot_text("ARMED",&g_armed);
        w3m_snapshot_text("END_BEFORE_RESTORE",&g_during_end);
        w3m_snapshot_text("AFTER_RESTORE",&g_restored);
        if (g_has_mid) w3m_snapshot_text("MID",&g_mid);
        w3m_snapshot_text("LAST_BEFORE_RAW_OFF",&g_last);
        w3m_delta_text("PRE_TO_APPLIED",&g_pre,&g_applied);
        w3m_delta_text("APPLIED_TO_ARMED",&g_applied,&g_armed);
        w3m_delta_text("ARMED_TO_END",&g_armed,&g_during_end);
        w3m_delta_text("END_TO_RESTORED",&g_during_end,&g_restored);
        w3m_delta_text("PRE_TO_RESTORED",&g_pre,&g_restored);
        if (g_has_mid)
            w3m_delta_text("ARMED_TO_MID",&g_armed,&g_mid);
        snprintf(b,sizeof(b),"VIDEO filename=%s frames=%d apply=%d restore=%d cleanup=%d stop=%d drain=%d writer_io=%d writer_post=%d\n",
            g_video_name,g_stop_frame_count,g_apply_rc,g_restore_rc,g_stop_clean,
            g_stop_transfer_rc,g_stop_drain_rc,g_writer_io,g_writer_post);
        w3m_text(b);
        uint32_t fc=g_frames;
        if (fc>W3M_MAX_FRAMES)fc=W3M_MAX_FRAMES;
        for (uint32_t k=0;k<fc;++k)
        {
            snprintf(b,sizeof(b),"FRAME index=%d owner_seq=%d complete_seq=%d slot=%d time_us=%d\n",
                (int)g_meta[k].frame,(int)g_meta[k].owner,
                (int)g_meta[k].complete,g_meta[k].slot,(int)g_meta[k].us);
            w3m_text(b);
        }
        /* Source-buffer canaries from the oversized guarded b0=1 path.
         * A nominal-tail change is expected to be informative, not an
         * automatic failure. Writes past 16-bit capacity force failure. */
        struct rp_w3_guard_status guard;
        memset(&guard,0,sizeof(guard));
        raw_vid_rp_guard_get_status(&guard);
        if (guard.enabled)
        {
            snprintf(b,sizeof(b),
                "W3GUARD enabled=%d nominal=%d full16=%d alloc=%d checked=%d nominal_changed=%d full16_changed=%d after16_changed=%d first_nominal_slot=%d first_after16_slot=%d\n",
                (int)guard.enabled,(int)guard.nominal,(int)guard.nominal16,
                (int)guard.allocated_per_slot,(int)guard.checked,
                (int)guard.nominal_changed,(int)guard.full16_changed,
                (int)guard.after16_changed,(int)guard.first_nominal_slot,
                (int)guard.first_after16_slot);
            w3m_text(b);
            W3M_LOG("GUARD case=%s nominal_changed=%d full16_changed=%d after16_changed=%d frames=%d",
                c->id,(int)guard.nominal_changed,(int)guard.full16_changed,
                (int)guard.after16_changed,(int)guard.checked);
            if (!guard.checked || guard.after16_changed)
                failed=-142;
        }
        if (g_log_error || !w3m_case_ok()) failed=-140;
        W3M_LOG("RUN_END id=%s result=%s file=%s frames=%d apply=%d restore=%d",
                c->id,failed?"FAIL":"PASS",g_video_name,g_stop_frame_count,
                g_apply_rc,g_restore_rc);
        w3m_text(failed?"RUN_END FAIL (automatic interlock)\n":"RUN_END PASS\n");
        if (g_log_error && !failed) failed=-141;
        if (failed)break;
        if (g_single_case < 0 && i+1 < run_limit)
        {
            int settle_rc=w3m_wait_inter_session_ready();
            if (settle_rc)
            {
                failed=-160;
                W3M_LOG("SETTLE FAIL after %s rc=%d; no next test",c->id,settle_rc);
                w3m_text("ABORT INTER_SESSION_STABILIZATION_FAILED; cold boot needed\n");
                break;
            }
        }
        /* Flush per-case result before the state advances. The file is
         * deliberately reopened per write to survive power interruption. */
        uint32_t next=(uint32_t)i+1u;
        int boundary=(next < (uint32_t)run_limit && w3m_group_boundary(i) &&
                     !g_no_restart);
        uint32_t status_next = next==(uint32_t)run_limit ? W3M_COMPLETE :
                                boundary ? W3M_NEEDS_REBOOT : W3M_READY;
        if (g_single_case < 0 && w3m_state_store(next,status_next))
        { failed=-150; W3M_LOG("ABORT checkpoint commit failed id=%s",c->id);break; }
        g_index=-1;
        if (boundary)
        {
            checkpoint=1;
            g_interlock=1;
            W3M_LOG("CHECKPOINT after=%s next=%s POWER OFF camera; cold boot; select same menu entry",
                    c->id,w3m_cases[next].id);
            snprintf(b,sizeof(b),"CHECKPOINT after=%s next=%s; POWER OFF; cold boot; reselect W3 PU Auto Matrix\n",
                     c->id,w3m_cases[next].id);
            w3m_text(b);
            break;
        }
        /* v4: inter-case passive gap; no 10-Hz probes or extra preflights. */
    }
    w3m_stage_drain();
    if ((failed || g_abort) && g_single_case < 0)
        (void)w3m_state_store(g_index>=0?(uint32_t)g_index:first,W3M_FAILED);
    snprintf(b,sizeof(b),"\nMATRIX_END status=%s rc=%d abort=%d checkpoint=%d last_index=%d\n",
             failed||g_abort?"ABORT":checkpoint?"CHECKPOINT":"PASS",
             failed,(int)g_abort,checkpoint,(int)g_index);
    w3m_text(b);
    W3M_LOG("MATRIX_END rc=%d abort=%d checkpoint=%d last=%d",failed,
            (int)g_abort,checkpoint,(int)g_index);
    FIO_CloseFile(g_log);
    g_log=NULL;
    if (failed || g_abort || checkpoint || g_log_error) g_interlock=1;
    g_index=-1;
    g_running=0;
}

/* DryOS task_create() returns a uint32_t encoded token, NOT a signed errno.
 * The low 16 bits of (token >> 1) contain the task ID; high bits are opaque.
 * An observed working task returned 0x806C015E (task_id=0x00AF), which
 * compares negative if incorrectly stored in int. Keep this interpretation
 * centralized for ALL / SINGLE / PAIR W3MAT launchers. */
static int w3m_create_runner_task(const char *task_name)
{
    uint32_t token = task_create(task_name, 0x13, 0x3000, w3m_runner, NULL);
    uint32_t id = (token >> 1) & 0xffffu;
    W3M_LOG("TASK_CREATE name=%s token=%x id=%d", task_name,
            (unsigned int)token, (int)id);
    /* Zero token / no task ID cannot identify a created task. Do not mistake
     * the token sign bit for an error. All other failures remain fail-closed
     * via the ordinary START timeout / next-session interlock. */
    return (token == 0u || token == 0xffffffffu || id == 0u) ? -1 : 0;
}

void rp_w3_matrix_launch_mode(int no_restart)
{
    if (g_running) { rp_w3_matrix_abort_user(); return; }
    if (g_interlock || !is_camera("RP","1.6.0") || !lv ||
        get_recording_state()!=INACTIVE)
    { W3M_LOG("REFUSE require RP 1.6.0, LV and idle recorder");return; }
    g_single_case = -1;
    g_pair_only = 0;
    g_no_restart = no_restart ? 1 : 0; /* freeze BEFORE task creation */
    W3M_LOG("MENU_START mode=%s state=%s cases=%d",
            g_no_restart ? "ALL_NO_REBOOT" : "CHECKPOINTED",
            w3m_state_filename(), W3M_CASES);
    g_running=1;g_abort=0;g_index=-1;
    int rc=w3m_create_runner_task(g_no_restart?"w3all":"w3matrix");
    if (rc) {g_running=0;W3M_LOG("ABORT task create rc=%d",rc);}
}
void rp_w3_matrix_launch(void) { rp_w3_matrix_launch_mode(0); }
void rp_w3_matrix_launch_single(int case_index)
{
    if (case_index < 0 || case_index >= W3M_CASES) return;
    if (g_running) { rp_w3_matrix_abort_user(); return; }
    if (g_interlock || !is_camera("RP","1.6.0") || !lv ||
        get_recording_state()!=INACTIVE) return;
    /* Does NOT load, create or modify W3ALL.STA/W3MAT.STA. */
    g_single_case=case_index;
    g_pair_only=0;
    g_no_restart=1;
    g_running=1;g_abort=0;g_index=-1;
    W3M_LOG("SINGLE_START case=%s index=%d",w3m_cases[case_index].id,case_index);
    int rc=w3m_create_runner_task("w3single");
    if (rc) {g_running=0;W3M_LOG("SINGLE task create rc=%d",rc);}
}

/* A00/A01 automated pair reproducer, independent of W3ALL.STA.
 * Will refuse to replay an interrupted W3PAIR.STA without explicit
 * removal after inspection, avoiding an automatic retry of a crash. */
void rp_w3_matrix_launch_pair(void)
{
    if (g_running) { rp_w3_matrix_abort_user(); return; }
    if (g_interlock || !is_camera("RP","1.6.0") || !lv ||
        get_recording_state()!=INACTIVE)
    { W3M_LOG("PAIR REFUSE: require RP, LV and idle recorder"); return; }
    g_single_case=-1;
    g_pair_only=1;
    g_no_restart=1;
    g_running=1;g_abort=0;g_index=-1;
    W3M_LOG("PAIR_START A00->A01 checkpoint=W3PAIR.STA");
    int rc=w3m_create_runner_task("w3pair");
    if (rc) {g_running=0;W3M_LOG("PAIR task create rc=%d",rc);}
}
