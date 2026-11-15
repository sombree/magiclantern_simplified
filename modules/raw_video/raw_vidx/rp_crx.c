/*
 * EOS RP 1.6.0 native CRX producer for raw_vidx.
 *
 * This file intentionally contains only the AlgsMgr StartEncode/FreeEncBuffer
 * bridge.  The large CRX result scatter is detached by rp_backend.c through
 * the already runtime-proven RD46 -> WR13 mem2mem path.
 *
 * Evidence boundary:
 *   - descriptor ABI, StartEncode ABI and result callback ABI: static-proven;
 *   - custom lossless raw_vidx StartEncode at 10/12/14-bit: runtime-proven;
 *   - custom C-RAW type2/ID2/mode3 at 14-bit 1080p: runtime-proven;
 *   - this revision extends C-RAW to the paired type6/ID10 and type7/ID11
 *     D-encoder routes for 12/10-bit while keeping the same direct-W3 ABI.
 */

#include "module.h"
#include "dryos.h"
#include "rp_crx.h"
#include "raw_vid.h"
#include "rp_crx_format.h"
#include "rp_crx_wait_budget.h"

#include <stdint.h>
#include <string.h>

#define RP_CRX_START_ENCODE             0xE0651986u
#define RP_CRX_FREE_ENC_BUFFER          0xE0651EF2u
#define RP_CRX_ALGSMGR_CHANGE_MODE       0xE024CC18u
#define RP_CRX_ALGSMGR_OWNER_SLOT       0x00005A30u
/* ROM: E0272FF0/E02B734C both contain global backend pointer slot 0x12684.
 * backend+0x14: shared cursor; +0x08/+0x0c: arena base/size;
 * +0x28: pointer to 0x288-stride contexts.  Read-only throughout v14. */
#define RP_CRX_BACKEND_SLOT             0x00012684u
#define RP_CRX_CONTEXT_STRIDE           0x288u
#define RP_CRX_ALGSMGR_TIMER_OFFSET     0x24u
#define RP_CRX_ALGSMGR_MODE_OFFSET      0x2Cu
#define RP_CRX_ALGSMGR_SCRATCH_OFFSET   0xB8u
#define RP_CRX_ALGSMGR_LATCH_OFFSET     0x124u
#define RP_CRX_ALGSMGR_LATCH_COUNT      0x16u
#define RP_CRX_MODE_LOSSLESS            0x00001001u
#define RP_CRX_TIMER_IDLE_SENTINEL       0x00000007u
#define RP_CRX_MODE_ALT                 0x00002000u
#define RP_CRX_ENCODE_TIMEOUT_US        1000000u
/* Set to 0 when comparing against the original 1-ms polling path.
 * Default to completion notification through a DryOS semaphore. */
#ifndef RP_CRX_EVENT_WAIT
#define RP_CRX_EVENT_WAIT 1
#endif
/* A bounded wake interval preserves the timeout and allows observation of
 * g_done even if Canon/DryOS does not deliver a semaphore notification. */
#define RP_CRX_EVENT_WAIT_SLICE_MS      10u
#define RP_CRX_FREE_TIMEOUT_US          1000000u
#define RP_CRX_DESC_TYPE_14BIT          RP_CRX_PROFILE_14BIT
#define RP_CRX_DESC_TYPE_12BIT          RP_CRX_PROFILE_12BIT
#define RP_CRX_DESC_TYPE_10BIT          RP_CRX_PROFILE_10BIT
#define RP_CRX_ENCODER_ID_14BIT         0u
#define RP_CRX_ENCODER_ID_12BIT         8u
#define RP_CRX_ENCODER_ID_10BIT         9u
#define RP_CRAW_DESC_TYPE_14BIT          RP_CRAW_PROFILE_14BIT
#define RP_CRAW_DESC_TYPE_12BIT          RP_CRAW_PROFILE_12BIT
#define RP_CRAW_DESC_TYPE_10BIT          RP_CRAW_PROFILE_10BIT
#define RP_CRAW_ENCODER_ID_14BIT         2u
#define RP_CRAW_ENCODER_ID_12BIT         10u
#define RP_CRAW_ENCODER_ID_10BIT         11u
#define RP_CRX_LOW_ALLOC_SEM_SLOT        0x0001B96Cu
#define RP_THUMB_FN(addr, type)         ((type)(uintptr_t)((addr) | 1u))

#define RP_CRX_TRACE(fmt, ...) \
    DryosDebugMsg(0, 15, "RPVIDX CRX " fmt, ##__VA_ARGS__)

struct rp_crx_descriptor
{
    uint32_t type;
    uint32_t width;
    uint32_t height;
    uint32_t x;
    uint32_t y;
    uint32_t bits;
    uint32_t source;
    uint32_t source_width;
    uint32_t aux0;
    uint32_t aux1;
    uint32_t aux2;
};
typedef char rp_crx_descriptor_must_be_44_bytes[
    (sizeof(struct rp_crx_descriptor) == 0x2Cu) ? 1 : -1
];

typedef uint32_t (*rp_crx_start_fn)(void *, uint32_t, uint32_t, uint32_t);
typedef uint32_t (*rp_crx_free_fn)(uint32_t, uint32_t, uint32_t);
typedef void (*rp_crx_change_mode_fn)(void *, uint32_t, uint32_t, uint32_t);

static volatile uint32_t g_active = 0;
static volatile uint32_t g_done = 0;
#if RP_CRX_EVENT_WAIT
/* Created in the recorder task before StartEncode, never in a callback.
 * Module teardown still requires all outstanding native jobs to be drained.
 * The original timeout/poison policy remains authoritative. */
static struct semaphore *g_completion_sem = 0;
#endif
static volatile uint32_t g_active_token = 0;
static volatile uint32_t g_callback_token = 0;
static volatile uint32_t g_callback_count = 0;
static volatile uint32_t g_callback_handles[RP_CRX_RESULT_MAX] = {0};
static volatile uint32_t g_poisoned = 0;
static volatile uint32_t g_free_pending = 0;
static volatile uint32_t g_free_completed = 0;
static volatile uint32_t g_free_callback_errors = 0;
/* Canon AlgsMgr keeps initialized codec state per encoder ID across
 * FreeEncBuffer.  v12 avoids cross-BPP reuse by assigning one existing
 * RAW-like D encoder bank to each synthetic BPP.  Software ownership is
 * tracked per AlgsMgr encoder ID; the legacy scalar is retained only as the
 * last successfully completed BPP for existing stats/reporting ABI. */
static volatile uint32_t g_encoder_latched_bits = 0;
/* First successfully initialized geometry per bank. Canon keeps persistent
 * workspace/state behind each AlgsMgr encoder ID.  Runtime v21.3 evidence
 * showed that reusing ID10 after a 3728x2052 -> 2128x1208 profile shift can
 * enter repeated RetryMargin/DirectEncStart cycles long enough to exhaust the
 * three-buffer W3 source ring.  Therefore any geometry change on an already
 * initialized bank now fails closed; cold boot (or a future proven bank reset)
 * is required before changing geometry for the same encoder ID. */
static uint32_t g_bank_first_width[RP_CRX_ALGSMGR_LATCH_COUNT] = {0};
static uint32_t g_bank_first_height[RP_CRX_ALGSMGR_LATCH_COUNT] = {0};
static volatile uint32_t g_encoder_bank_bits[RP_CRX_ALGSMGR_LATCH_COUNT] = {0};
/* v13 post-stop init-latch experiment: preserve session identity across the
 * complete native FreeEncBuffer -> RP cleanup boundary.  Deliberately does
 * NOT alter the global arena cursor, HW lease or bank workspace pointers. */
static uint32_t g_session_manager = 0;
static uint32_t g_session_manager_mode = 0;
static uint32_t g_session_lossy = 0;
static uint32_t g_session_bits = 0;
static uint32_t g_session_first_hw_latch = 0xffffffffu;
static uint32_t g_session_first_sw_bits = 0xffffffffu;
static uint32_t g_session_encode_attempts = 0;
static uint32_t g_session_good_encodes = 0;
static uint32_t g_session_clean_end = 0;
static uint32_t g_session_multi_bits = 0;
static uint32_t g_latch_reset_attempted = 0; /* one write maximum / boot */
static uint32_t g_retry_state = RP_CRX_RETRY_NONE;
static uint32_t g_retry_bits = 0u;
static uint32_t g_retry_width = 0u;
static uint32_t g_retry_height = 0u;
static uint32_t g_last_good_width = 0u;
static uint32_t g_last_good_height = 0u;
static uint32_t g_retry_id = 0xffffffffu;
static uint32_t g_retry_manager = 0u;
static uint32_t g_retry_mode = 0u;
static uint32_t g_first_alloc_bytes = 0u;
static uint32_t g_first_alloc_valid = 0u;
/* One read-only snapshot immediately after the first initialization, before
 * the latch clear.  Workspace pointers are compared against retry results. */
struct rp_crx_bank_snapshot
{
    uint32_t backend, arena_base, arena_size, cursor;
    uint32_t context, pointers[5], hw_active, latch, mode;
};
static struct rp_crx_bank_snapshot g_preclear_snapshot;
static struct rp_crx_bank_snapshot g_first_init_after;
static uint32_t g_first_init_after_valid = 0u;
static struct rp_crx_bank_snapshot g_first_encode_before;
static uint32_t g_first_encode_before_valid = 0u;
/* v16: independent bounded repeated-latch experiment. No global cursor rewinds,
 * workspace frees, ChangeMode calls, or changes to Canon's still path. */
static uint32_t g_multi_enabled = 0u;
static uint32_t g_multi_budget = 0u;
static uint32_t g_multi_clears = 0u;
static uint32_t g_multi_manager = 0u;
static uint32_t g_multi_backend = 0u;
static uint32_t g_multi_mode = 0u;
static uint32_t g_multi_arena_base = 0u;
static uint32_t g_multi_arena_size = 0u;
static uint32_t g_multi_inflight = 0u;
static uint32_t g_multi_current_id = 0xffffffffu;
static uint32_t g_multi_pending[RP_CRX_ALGSMGR_LATCH_COUNT] = {0};
static struct rp_crx_bank_snapshot g_multi_cleared[RP_CRX_ALGSMGR_LATCH_COUNT];
static struct rp_crx_bank_snapshot g_multi_encode_after;
static uint32_t g_multi_encode_after_valid = 0u;

/* v18: read-only one-session allocation checkpoint. It is NOT an ownership
 * token and cannot exclude interleaved Canon allocations/hidden DMA refs. */
struct rp_crx_tail_audit
{
    uint32_t enabled, id, bits, width, height;
    uint32_t before_valid, after_valid;
    struct rp_crx_bank_snapshot before, after;
};
static struct rp_crx_tail_audit g_tail_audit;

/* v19: multi-session read-only ownership/range survey. The previous STOP
 * snapshot is retained to identify Canon-native changes BETWEEN ML sessions.
 * All derived ranges are estimates from LOW_D_ALLOC, not lifetime proofs. */
struct rp_crx_v19_range
{
    uint32_t lo, hi, p0, p1, p2, p3, p4;
    uint32_t k, n, opt, latch, hw;
    uint32_t present, valid;
};
static struct rp_crx_v19_range g_v19_pre[RP_CRX_ALGSMGR_LATCH_COUNT];
static struct rp_crx_v19_range g_v19_prev[RP_CRX_ALGSMGR_LATCH_COUNT];
static uint32_t g_v19_prev_valid, g_v19_prev_mgr, g_v19_prev_backend;
static uint32_t g_v19_prev_mode, g_v19_prev_cursor, g_v19_prev_base;
static uint32_t g_v19_session, g_v19_active, g_v19_pre_valid;
static uint32_t g_v19_pre_mgr, g_v19_pre_backend, g_v19_pre_mode;
static uint32_t g_v19_pre_cursor;
static uint32_t g_v19_between_changed, g_v19_epoch_suspect;

/* v22 guarded tail reclaim.  This is the first branch that may write the
 * manager init latch and backend cursor.  It deliberately does not call
 * ChangeMode/BACKEND_MODE/HW_DISABLE and never clears context workspace
 * pointers.  A candidate exists only when this module observed a fresh
 * latch-0 -> initialized allocation and can later prove that exact allocation
 * is still the global arena tail. */
struct rp_crx_route
{
    uint32_t type;
    uint32_t id;
};

static const struct rp_crx_route g_v22_lossless_routes[] =
{
    { 0u, 0u }, { 1u, 1u }, { 4u, 8u }, { 5u, 9u },
};
static const struct rp_crx_route g_v22_craw_routes[] =
{
    { 2u, 2u }, { 3u, 3u }, { 6u, 10u }, { 7u, 11u },
};

static uint32_t g_route_valid = 0u;
static uint32_t g_route_bits = 0u;
static uint32_t g_route_lossy = 0u;
static uint32_t g_route_type = 0xffffffffu;
static uint32_t g_route_id = 0xffffffffu;
static uint32_t g_reclaim_candidate_valid = 0u;
static struct rp_crx_bank_snapshot g_reclaim_pre;
static struct rp_crx_bank_snapshot g_reclaim_tail;
static volatile uint32_t g_reclaim_requested = 0u;
static volatile uint32_t g_reclaim_in_progress = 0u;
static volatile uint32_t g_reclaim_attempted = 0u;
static volatile int32_t g_reclaim_last_rc = 0;
static volatile uint32_t g_reclaim_last_path = 0u;
static volatile uint32_t g_reclaim_write_started = 0u;

/* Long-lived AlgsMgr state callback wrapper.  v64/v65 established that
 * state+0x0C is synchronously invoked by the manager dispatcher.  We forward
 * every event to Canon.  Only after one of OUR FreeEncBuffer event-4 payloads
 * has returned do we count the post-handler boundary and, when STOP has
 * already crossed the worker FIFO barrier, attempt the guarded reclaim. */
static volatile uint32_t g_session_state_installed = 0u;
static uint32_t g_session_state = 0u;
static uint32_t g_session_state_orig_cb = 0u;
static volatile uint32_t g_session_state_hits = 0u;
static volatile uint32_t g_session_state_event4 = 0u;
static volatile uint32_t g_free_post_completed = 0u;

static uint32_t g_next_token = 1;
static struct rp_crx_stats g_stats = {0};

/* v11a2: passive AlgsMgr state-dispatch probe without a third ROM/MMU patch.
 * E0651736 dispatches the current encode state through an object at
 * manager+0x0c; E058F910 then BLXes state+0x0c.  We replace that RAM callback
 * pointer for exactly one dispatch, restore it inside the wrapper before
 * calling Canon, and record the raw dispatch ABI.  No Canon state is changed. */
typedef uint32_t (*rp_crx_state_cb_fn)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
static volatile uint32_t g_state_probe_armed = 0;
static volatile uint32_t g_state_probe_hits = 0;
static volatile uint32_t g_state_probe_matches = 0;
static volatile uint32_t g_state_probe_mismatches = 0;
static volatile uint32_t g_state_probe_req_type = 0xffffffffu;
static volatile uint32_t g_state_probe_req_bits = 0;
static volatile uint32_t g_state_probe_req_src = 0;
static volatile uint32_t g_state_probe_state = 0;
static volatile uint32_t g_state_probe_orig_cb = 0;
static volatile uint32_t g_state_probe_a0 = 0;
static volatile uint32_t g_state_probe_a1 = 0;
static volatile uint32_t g_state_probe_a2 = 0;
static volatile uint32_t g_state_probe_a3 = 0;
static volatile uint32_t g_state_probe_a4 = 0;
static volatile uint32_t g_state_probe_job_type = 0xffffffffu;
static volatile uint32_t g_state_probe_job_bits = 0;
static volatile uint32_t g_state_probe_job_src = 0;

static inline void rp_crx_barrier(void)
{
    __asm__ volatile("dmb" ::: "memory");
}

static int rp_crx_ram_ptr(uint32_t ptr);
static int rp_crx_bank_snapshot(uint32_t manager, uint32_t id,
                                struct rp_crx_bank_snapshot *s);
static int rp_crx_snapshot_arena_valid(const struct rp_crx_bank_snapshot *s);
static void rp_crx_free_complete_cb(uint32_t opaque, uint32_t status);
static int rp_crx_v22_try_reclaim(uint32_t path);

/* Keep each synthetic BPP on a different Canon RAW-like D encoder bank.
 * output111/v12 static audit: type 0/4/5 all map through the same RAW class
 * field (1); downstream INIT_D/D_CONFIG/DIRECT_START/header logic does not
 * distinguish 4/5, while type 0 is the already runtime-proven baseline. */
static int rp_crx_select_bank(uint32_t bits, uint32_t *type, uint32_t *enc_id)
{
    if (!type || !enc_id)
        return -1;

    switch (bits)
    {
        case 14u:
            *type = RP_CRX_DESC_TYPE_14BIT;
            *enc_id = RP_CRX_ENCODER_ID_14BIT;
            return 0;
        case 12u:
            *type = RP_CRX_DESC_TYPE_12BIT;
            *enc_id = RP_CRX_ENCODER_ID_12BIT;
            return 0;
        case 10u:
            *type = RP_CRX_DESC_TYPE_10BIT;
            *enc_id = RP_CRX_ENCODER_ID_10BIT;
            return 0;
        default:
            return -1;
    }
}

/* Canon-native D-route matrix:
 *   RAW full pair   : types 0/1 -> IDs 0/1
 *   CRAW full pair  : types 2/3 -> IDs 2/3
 *   RAW crop pair   : types 4/5 -> IDs 8/9
 *   CRAW crop pair  : types 6/7 -> IDs 10/11
 *
 * raw_vidx already uses 0/4/5 as independent lossless 14/12/10-bit banks.
 * C-RAW therefore uses the paired codec-family routes 2/6/7 for 14/12/10-bit.
 * Type2/ID2/mode3 is runtime-proven on custom 14-bit W3 input; type6/type7
 * share the same StartDEncode/source/ROI ABI and are validated by result
 * profile + exact CMP1 before any frame reaches the writer. */
static int rp_crx_select_route(uint32_t bits, int lossy,
                               uint32_t *type, uint32_t *enc_id)
{
    if (!lossy)
        return rp_crx_select_bank(bits, type, enc_id);
    if (!type || !enc_id)
        return -1;

    switch (bits)
    {
        case 14u:
            *type = RP_CRAW_DESC_TYPE_14BIT;
            *enc_id = RP_CRAW_ENCODER_ID_14BIT;
            return 0;
        case 12u:
            *type = RP_CRAW_DESC_TYPE_12BIT;
            *enc_id = RP_CRAW_ENCODER_ID_12BIT;
            return 0;
        case 10u:
            *type = RP_CRAW_DESC_TYPE_10BIT;
            *enc_id = RP_CRAW_ENCODER_ID_10BIT;
            return 0;
        default:
            return -1;
    }
}


static int rp_crx_v22_pick_free_route(uint32_t manager, uint32_t bits, int lossy,
                                      uint32_t *type, uint32_t *enc_id)
{
    if (!type || !enc_id || !rp_crx_ram_ptr(manager))
        return -1;

    const struct rp_crx_route *routes = lossy ? g_v22_craw_routes : g_v22_lossless_routes;
    uint32_t count = lossy
        ? (uint32_t)(sizeof(g_v22_craw_routes) / sizeof(g_v22_craw_routes[0]))
        : (uint32_t)(sizeof(g_v22_lossless_routes) / sizeof(g_v22_lossless_routes[0]));

    /* Dynamic policy is intentionally BPP-independent.  Select the first
     * truly free physical D route in the codec family; descriptor.bits carries
     * 10/12/14 independently.  After a successful tail reclaim the just-used
     * lowest route becomes latch-0 again and is naturally reused for a later
     * BPP, demonstrating the v62 TYPE_MAP result rather than preserving the
     * historical per-BPP mapping. */
    for (uint32_t i = 0u; i < count; i++)
    {
        struct rp_crx_bank_snapshot b;
        if (rp_crx_bank_snapshot(manager, routes[i].id, &b) != 0 ||
            !rp_crx_snapshot_arena_valid(&b) ||
            b.mode != RP_CRX_MODE_LOSSLESS ||
            b.latch != 0u || b.hw_active != 0u ||
            g_encoder_bank_bits[routes[i].id] != 0u)
            continue;
        *type = routes[i].type;
        *enc_id = routes[i].id;
        return 0;
    }

    (void)bits;
    return -2;
}

static int rp_crx_route_for_session(uint32_t bits, int lossy,
                                    uint32_t *type, uint32_t *enc_id)
{
    if (!g_route_valid || bits != g_route_bits ||
        (uint32_t)(lossy ? 1 : 0) != g_route_lossy ||
        !type || !enc_id)
        return -1;
    *type = g_route_type;
    *enc_id = g_route_id;
    return 0;
}

/* v20 READ-ONLY policy probe.  The public StartEncode ABI takes a descriptor
 * type, not an explicit encoder ID.  Runtime has proven only these three
 * type->ID routes.  v20 therefore computes the first currently-free route
 * from this KNOWN pool, but does NOT use it for submission.  In particular,
 * type/BPP decoupling is still unproven and must be closed by v61 before an
 * active dynamic-ID build. */
struct rp_crx_known_route
{
    uint32_t type;
    uint32_t id;
};

static const struct rp_crx_known_route g_v20_known_routes[] =
{
    /* TYPE_MAP @ E024CB80, D branch only.  Types 8/9 use a distinct
     * manager-init selector (MovieDS in observed Canon use), and 10/11
     * enter Q mode, so v20.1 deliberately excludes them. */
    { 0u,  0u },
    { 1u,  1u },
    { 2u,  2u },
    { 3u,  3u },
    { 4u,  8u },
    { 5u,  9u },
    { 6u, 10u },
    { 7u, 11u },
};

static int rp_crx_v20_pick_known_free(uint32_t manager, uint32_t bits,
                                      uint32_t legacy_type, uint32_t legacy_id,
                                      uint32_t *pick_type, uint32_t *pick_id)
{
    if (!pick_type || !pick_id || !rp_crx_ram_ptr(manager))
        return -1;

    /* Passive runtime correlation for v62.  manager+0x1C is the manager
     * semaphore handle seen in algsmgrChangeMode; 0x1B96C is the global
     * holding LOW_D_ALLOC's semaphore handle.  Read only. */
    uint32_t mgr_sem = *(volatile uint32_t *)(uintptr_t)(manager + 0x1cu);
    uint32_t low_sem = *(volatile uint32_t *)(uintptr_t)RP_CRX_LOW_ALLOC_SEM_SLOT;
    RP_CRX_TRACE("V20 LOCKS mgrsem=%x lowsem=%x same=%d READ_ONLY",
                 mgr_sem,low_sem,(int)(mgr_sem != 0u && mgr_sem == low_sem));

    for (unsigned i = 0; i < sizeof(g_v20_known_routes)/sizeof(g_v20_known_routes[0]); i++)
    {
        uint32_t id = g_v20_known_routes[i].id;
        struct rp_crx_bank_snapshot b;
        if (rp_crx_bank_snapshot(manager, id, &b) ||
            !rp_crx_snapshot_arena_valid(&b) ||
            b.mode != RP_CRX_MODE_LOSSLESS)
            continue;

        /* A free latch plus no live HW lease is the minimum candidate rule.
         * Stale workspace pointers are allowed in this dry-run because the
         * future architecture intentionally reinitializes a released slot. */
        if (b.latch == 0u && b.hw_active == 0u)
        {
            unsigned stale = 0;
            for (unsigned k = 0; k < 5; k++)
                if (b.pointers[k]) stale++;
            *pick_type = g_v20_known_routes[i].type;
            *pick_id = id;
            RP_CRX_TRACE("V20 PICK D0_7 bits=%d id=%d type=%d stale=%d legacy=%d/%d READ_ONLY",
                         (int)bits,(int)id,(int)*pick_type,(int)stale,
                         (int)legacy_id,(int)legacy_type);
            if (id != legacy_id || *pick_type != legacy_type)
                RP_CRX_TRACE("V20 PICK differs_from_legacy: type/BPP decoupling UNPROVEN");
            return 0;
        }
    }

    RP_CRX_TRACE("V20 PICK BLOCK no free D0_7 route bits=%d legacy=%d/%d",
                 (int)bits,(int)legacy_id,(int)legacy_type);
    return -2;
}

static uint32_t rp_crx_hw_latch(uint32_t manager, uint32_t enc_id)
{
    if (manager < 0x1000u || manager >= 0xE0000000u ||
        (manager & 3u) || enc_id >= RP_CRX_ALGSMGR_LATCH_COUNT)
        return 0xffffffffu;

    return *(volatile uint32_t *)(uintptr_t)
        (manager + RP_CRX_ALGSMGR_LATCH_OFFSET + enc_id * 4u);
}

/* Pointer provenance from output148/output149; do not probe guessed MMIO.
 * A zero/implausible backend or context is a hard failure for v14's write.
 * These reads do not establish exclusive ownership of the Canon arena. */
static int rp_crx_ram_ptr(uint32_t ptr)
{
    return ptr >= 0x1000u && ptr < 0xC0000000u && (ptr & 3u) == 0u;
}

static int rp_crx_bank_snapshot(uint32_t manager, uint32_t id,
                                struct rp_crx_bank_snapshot *s)
{
    if (!s || !rp_crx_ram_ptr(manager) || id >= RP_CRX_ALGSMGR_LATCH_COUNT)
        return -1;
    memset(s, 0, sizeof(*s));
    s->backend = *(volatile uint32_t *)(uintptr_t)RP_CRX_BACKEND_SLOT;
    if (!rp_crx_ram_ptr(s->backend))
        return -2;
    volatile uint32_t *b = (volatile uint32_t *)(uintptr_t)s->backend;
    s->arena_base = b[2];             /* backend+0x08 */
    s->arena_size = b[3];             /* backend+0x0c */
    s->cursor = b[5];                 /* backend+0x14 */
    uint32_t ctx_base = b[10];        /* backend+0x28 */
    if (!rp_crx_ram_ptr(ctx_base) ||
        ctx_base > 0xBFFFFFFFu - (RP_CRX_ALGSMGR_LATCH_COUNT * RP_CRX_CONTEXT_STRIDE))
        return -3;
    s->context = ctx_base + id * RP_CRX_CONTEXT_STRIDE;
    volatile uint32_t *c = (volatile uint32_t *)(uintptr_t)s->context;
    s->pointers[0] = c[3];            /* context+0x0c */
    s->pointers[1] = c[4];            /* context+0x10 */
    s->pointers[2] = c[5];            /* context+0x14 */
    s->pointers[3] = c[6];            /* context+0x18 */
    s->pointers[4] = c[7];            /* context+0x1c */
    s->hw_active = c[0x280 / 4];
    s->latch = rp_crx_hw_latch(manager, id);
    s->mode = *(volatile uint32_t *)(uintptr_t)(manager + RP_CRX_ALGSMGR_MODE_OFFSET);
    return 0;
}

static void rp_crx_log_snapshot(const char *phase, uint32_t id,
                                const struct rp_crx_bank_snapshot *s)
{
    RP_CRX_TRACE("RETRY %s id=%d latch=%d lease=%d mode=%x",
                 phase, (int)id, (int)s->latch, (int)s->hw_active, s->mode);
    RP_CRX_TRACE("RETRY %s backend=%x base=%x size=%x cur=%x",
                 phase, s->backend, s->arena_base, s->arena_size, s->cursor);
    RP_CRX_TRACE("RETRY %s ctx=%x p0=%x p1=%x", phase,
                 s->context, s->pointers[0], s->pointers[1]);
    RP_CRX_TRACE("RETRY %s p2=%x p3=%x p4=%x", phase,
                 s->pointers[2], s->pointers[3], s->pointers[4]);
}

static int rp_crx_snapshot_arena_valid(const struct rp_crx_bank_snapshot *s);

/* v19 32-bit range construction uses 64-bit intermediates to avoid wrap.
 * Context fields are touched only after the v18 bank snapshot validates the
 * mapped D-context region. They can still change concurrently (non-atomic). */
static int rp_crx_v19_read_bank(uint32_t mgr, uint32_t id,
                                const struct rp_crx_bank_snapshot *root,
                                struct rp_crx_v19_range *out)
{
    memset(out, 0, sizeof(*out));
    struct rp_crx_bank_snapshot b;
    if (rp_crx_bank_snapshot(mgr, id, &b) || b.backend != root->backend ||
        b.mode != RP_CRX_MODE_LOSSLESS || b.cursor != root->cursor ||
        b.arena_base != root->arena_base || b.arena_size != root->arena_size)
        return -1;
    out->latch = b.latch;
    out->hw = b.hw_active;
    out->p0=b.pointers[0]; out->p1=b.pointers[1];
    out->p2=b.pointers[2]; out->p3=b.pointers[3]; out->p4=b.pointers[4];
    out->present = b.latch || b.hw_active || b.pointers[0] || b.pointers[1] ||
                   b.pointers[2] || b.pointers[3] || b.pointers[4];
    if (!out->present) { out->valid=1u; return 0; }
    volatile uint8_t *c8=(volatile uint8_t *)(uintptr_t)b.context;
    volatile uint32_t *c32=(volatile uint32_t *)(uintptr_t)b.context;
    out->n = c8[0x272];
    out->k = c32[0x278/4];
    volatile uint32_t *bk=(volatile uint32_t *)(uintptr_t)b.backend;
    out->opt=(bk[0x1c/4] == 1u) ? 1u : 0u;
    /* LOW_D_ALLOC forces N=8 when backend+0x10 bit 13 is set. */
    if ((bk[0x10/4] & 0x2000u) != 0u) out->n=8u;
    uint64_t arena_end = (uint64_t)b.arena_base+b.arena_size;
    if (out->k == 0u || out->k > 2048u || out->n == 0u ||
        out->n > 255u || !out->p0 || !out->p1 || !out->p2 ||
        !out->p3 || !out->p4 ||
        out->p0 > out->p1 || out->p1 > out->p3 ||
        out->p3 > out->p2 || out->p2 > out->p4)
        return -2;
    /* Last base pointer is END of K*12, followed by K*4 and optional
     * two 0x400 blocks. Optional pre-alignment is 1..32 B, not 0..31. */
    uint64_t end64=(uint64_t)out->p4+(uint64_t)out->k*4u;
    if (out->opt)
    {
        end64+=(uint64_t)(32u-((uint32_t)end64 & 31u))+0x800u;
        /* Optional pointers must point into the same workspace. */
        if (!c32[0x258/4] || !c32[0x25c/4] ||
            c32[0x258/4] >= c32[0x25c/4]) return -3;
        if ((uint64_t)c32[0x25c/4]+0x400u > end64) return -4;
    }
    end64=(end64+31u)&~31ull;
    if (end64 > 0xFFFFFFFFull || end64 > arena_end ||
        out->p0 < b.arena_base || end64 > b.cursor || end64 <= out->p0)
        return -5;
    out->lo=out->p0;
    out->hi=(uint32_t)end64;
    out->valid=1u;
    /* Detect an allocator/mode change during this non-atomic scan. */
    if (*(volatile uint32_t *)(uintptr_t)RP_CRX_BACKEND_SLOT != b.backend ||
        *(volatile uint32_t *)(uintptr_t)(b.backend+0x14u) != b.cursor ||
        *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT != mgr)
    { out->valid=0u; return -6; }
    return 0;
}

/* Main worker task only; 22-bank snapshots are statically allocated to
 * avoid large stack frames and preserve previous STOP across sessions. */
static void rp_crx_v19_map(const char *phase, uint32_t own_id,
                           struct rp_crx_v19_range *dest,
                           uint32_t *unknown_out, uint32_t *active_out)
{
    uint32_t mgr=*(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    struct rp_crx_bank_snapshot root;
    uint32_t unknown=0u, active=0u, empty=0u;
    if (!rp_crx_ram_ptr(mgr) ||
        rp_crx_bank_snapshot(mgr, own_id, &root) ||
        !rp_crx_snapshot_arena_valid(&root) ||
        root.mode != RP_CRX_MODE_LOSSLESS)
    {
        RP_CRX_TRACE("V19 badmap p=%s s=%d mgr=%x", phase,
                     (int)g_v19_session,mgr);
        memset(dest,0,RP_CRX_ALGSMGR_LATCH_COUNT*sizeof(*dest));
        if (unknown_out) *unknown_out=RP_CRX_ALGSMGR_LATCH_COUNT;
        if (active_out) *active_out=0u;
        return;
    }
    RP_CRX_TRACE("V19 root p=%s s=%d mgr=%x bk=%x mode=%x",
                 phase,(int)g_v19_session,mgr,root.backend,root.mode);
    RP_CRX_TRACE("V19 cur p=%s s=%d base=%x cur=%x",
                 phase,(int)g_v19_session,root.arena_base,root.cursor);
    for (unsigned i=0;i<RP_CRX_ALGSMGR_LATCH_COUNT;i++)
    {
        int rc=rp_crx_v19_read_bank(mgr,i,&root,&dest[i]);
        struct rp_crx_v19_range *r=&dest[i];
        if (rc || !r->valid)
        {
            unknown++;
            RP_CRX_TRACE("V19 bad p=%s id=%d rc=%d l=%x p0=%x",
                         phase,(int)i,rc,r->latch,r->p0);
            continue;
        }
        if (!r->present) { empty++; continue; }
        active++;
        RP_CRX_TRACE("V19 cfg p=%s id=%d l=%d hw=%d n=%d k=%d opt=%d",
                     phase,(int)i,(int)r->latch,
                     (int)r->hw,(int)r->n,(int)r->k,(int)r->opt);
        RP_CRX_TRACE("V19 span p=%s id=%d a=%x b=%x",
                     phase,(int)i,r->lo,r->hi);
    }
    RP_CRX_TRACE("V19 map p=%s s=%d act=%d nil=%d bad=%d",
                 phase,(int)g_v19_session,(int)active,(int)empty,(int)unknown);
    if (unknown_out) *unknown_out=unknown;
    if (active_out) *active_out=active;
}

static void rp_crx_v19_pre(uint32_t id)
{
    if (!g_v19_active || id >= RP_CRX_ALGSMGR_LATCH_COUNT) return;
    uint32_t bad=0;
    rp_crx_v19_map("PRE",id,g_v19_pre,&bad,0);
    uint32_t mgr=*(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    struct rp_crx_bank_snapshot s;
    g_v19_pre_valid=0;
    if (bad || rp_crx_bank_snapshot(mgr,id,&s) ||
        !rp_crx_snapshot_arena_valid(&s))
    {
        g_v19_epoch_suspect=1;
        RP_CRX_TRACE("V19 pre_bad s=%d bad=%d",(int)g_v19_session,(int)bad);
        return;
    }
    g_v19_pre_valid=1;
    g_v19_pre_mgr=mgr; g_v19_pre_backend=s.backend;
    g_v19_pre_mode=s.mode; g_v19_pre_cursor=s.cursor;
    if (g_v19_prev_valid)
    {
        if (mgr!=g_v19_prev_mgr || s.backend!=g_v19_prev_backend ||
            s.mode!=g_v19_prev_mode || s.arena_base!=g_v19_prev_base ||
            s.cursor<g_v19_prev_cursor)
            g_v19_epoch_suspect=1;
        RP_CRX_TRACE("V19 between s=%d old=%x now=%x suspect=%d",
                     (int)g_v19_session,g_v19_prev_cursor,s.cursor,(int)g_v19_epoch_suspect);
        for (unsigned i=0;i<RP_CRX_ALGSMGR_LATCH_COUNT;i++)
        {
            const struct rp_crx_v19_range *a=&g_v19_prev[i],*b=&g_v19_pre[i];
            if (a->present!=b->present || a->valid!=b->valid ||
                a->lo!=b->lo || a->hi!=b->hi || a->latch!=b->latch ||
                a->hw!=b->hw)
            {
                g_v19_between_changed++;
                RP_CRX_TRACE("V19 diff id=%d a=%x-%x b=%x-%x",
                             (int)i,a->lo,a->hi,b->lo,b->hi);
            }
        }
        if (s.cursor!=g_v19_prev_cursor && !g_v19_between_changed)
            RP_CRX_TRACE("V19 between_unknown: cursor moved, ranges unchanged");
    }
}

static void rp_crx_v19_post(uint32_t id)
{
    if (!g_v19_active || id >= RP_CRX_ALGSMGR_LATCH_COUNT) return;
    struct rp_crx_bank_snapshot s;
    uint32_t mgr=*(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    if (!rp_crx_bank_snapshot(mgr,id,&s) &&
        rp_crx_snapshot_arena_valid(&s))
        RP_CRX_TRACE("V19 post s=%d id=%d old=%x now=%x",
                     (int)g_v19_session,(int)id,g_v19_pre_cursor,s.cursor);
    else RP_CRX_TRACE("V19 post_bad s=%d id=%d",(int)g_v19_session,(int)id);
}

static void rp_crx_v19_stop(int clean_stop)
{
    if (!g_v19_active) return;
    g_v19_active=0u;
    uint32_t id=g_tail_audit.id;
    if (id>=RP_CRX_ALGSMGR_LATCH_COUNT) id=RP_CRX_ENCODER_ID_12BIT;
    static struct rp_crx_v19_range stop_map[RP_CRX_ALGSMGR_LATCH_COUNT];
    uint32_t bad=0u, active=0u, hits=0u;
    rp_crx_v19_map("STOP",id,stop_map,&bad,&active);
    uint32_t a=g_tail_audit.before.cursor,b=g_tail_audit.after.cursor;
    if (!g_tail_audit.before_valid || !g_tail_audit.after_valid ||
        !g_v19_pre_valid || b<a || !clean_stop || bad ||
        g_v19_epoch_suspect)
    {
        RP_CRX_TRACE("V19 stop_bad s=%d clean=%d pre=%d a=%d b=%d bad=%d ep=%d",
                     (int)g_v19_session,clean_stop,(int)g_v19_pre_valid,
                     (int)g_tail_audit.before_valid,(int)g_tail_audit.after_valid,
                     (int)bad,(int)g_v19_epoch_suspect);
    }
    else
    {
        uint32_t now_mgr=*(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
        struct rp_crx_bank_snapshot now;
        const struct rp_crx_bank_snapshot *before=&g_tail_audit.before;
        const struct rp_crx_bank_snapshot *after=&g_tail_audit.after;
        if (now_mgr!=g_v19_pre_mgr ||
            rp_crx_bank_snapshot(now_mgr,id,&now) ||
            !rp_crx_snapshot_arena_valid(&now) ||
            now.cursor!=b || now.backend!=g_v19_pre_backend ||
            now.backend!=before->backend || now.backend!=after->backend ||
            now.arena_base!=before->arena_base ||
            now.arena_base!=after->arena_base ||
            now.mode!=RP_CRX_MODE_LOSSLESS ||
            now.hw_active!=0u || now.latch!=1u ||
            !g_session_clean_end || g_session_good_encodes!=1u ||
            g_active || g_free_pending || g_poisoned)
        {
            RP_CRX_TRACE("V19 stop_bad roots/leases s=%d",
                         (int)g_v19_session);
        }
        else if (b==a)
            RP_CRX_TRACE("V19 reuse s=%d id=%d cur=%x",
                         (int)g_v19_session,(int)id,b);
        else
        {
        for (unsigned i=0;i<RP_CRX_ALGSMGR_LATCH_COUNT;i++)
        {
            const struct rp_crx_v19_range *r=&stop_map[i];
            if (i==id || !r->present || !r->valid) continue;
            if (r->lo < b && a < r->hi)
            {
                hits++;
                RP_CRX_TRACE("V19 overlap id=%d lo=%x hi=%x",
                             (int)i,r->lo,r->hi);
            }
        }
        RP_CRX_TRACE("V19 stop %s s=%d id=%d hits=%d",
                     hits ? "BLOCK" : "UNPROVEN",
                     (int)g_v19_session,(int)id,(int)hits);
        RP_CRX_TRACE("V19 target s=%d a=%x b=%x",(int)g_v19_session,a,b);
        }
    }
    uint32_t mgr=*(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    struct rp_crx_bank_snapshot s;
    if (!bad && !rp_crx_bank_snapshot(mgr,id,&s) &&
        rp_crx_snapshot_arena_valid(&s))
    {
        memcpy(g_v19_prev,stop_map,sizeof(g_v19_prev));
        g_v19_prev_valid=1u;
        g_v19_prev_mgr=mgr; g_v19_prev_backend=s.backend;
        g_v19_prev_mode=s.mode; g_v19_prev_cursor=s.cursor;
        g_v19_prev_base=s.arena_base;
    }
    else g_v19_prev_valid=0u;
    RP_CRX_TRACE("V19 no_write s=%d no ownership proof",
                 (int)g_v19_session);
}

static int rp_crx_snapshot_arena_valid(const struct rp_crx_bank_snapshot *s)
{
    if (!rp_crx_ram_ptr(s->arena_base) || s->arena_size == 0u ||
        s->arena_size > 0x200000u ||
        s->arena_base > 0xBFFFFFFFu - s->arena_size)
        return 0;
    return s->cursor >= s->arena_base &&
           s->cursor < s->arena_base + s->arena_size;
}

/* The v16 multi-clear probe is opt-in and cannot be retrofitted to an
 * already-running v14 retry. Caller is the worker before S2 preflight. */
int raw_vid_rp_crx_multi_setup(uint32_t budget)
{
    if (!is_camera("RP", "1.6.0") || (budget != 4u && budget != 6u && budget != 9u) ||
        g_latch_reset_attempted || g_retry_state != RP_CRX_RETRY_NONE ||
        g_poisoned || g_active || g_free_pending)
        return -60;
    uint32_t mgr = *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    struct rp_crx_bank_snapshot s;
    if (rp_crx_bank_snapshot(mgr, RP_CRX_ENCODER_ID_10BIT, &s) ||
        !rp_crx_snapshot_arena_valid(&s))
        return -61;
    if (g_multi_enabled)
        return budget == g_multi_budget && mgr == g_multi_manager &&
               s.backend == g_multi_backend && s.mode == g_multi_mode &&
               s.arena_base == g_multi_arena_base &&
               s.arena_size == g_multi_arena_size ? 0 : -62;
    g_multi_manager = mgr;
    g_multi_backend = s.backend;
    g_multi_mode = s.mode;
    g_multi_arena_base = s.arena_base;
    g_multi_arena_size = s.arena_size;
    g_multi_budget = budget;
    g_multi_enabled = 1u;
    RP_CRX_TRACE("V16 MULTICLEAR ARM budget=%d mgr=%x arena=%x size=%x",
                 (int)budget, mgr, s.arena_base, s.arena_size);
    return 0;
}

/* No concurrent modes or arena rebuilds allowed during the experiment.
 * Do not require global cursor equality: another bank may have allocated. */
static int rp_crx_multi_validate(uint32_t mgr, uint32_t id,
                                 struct rp_crx_bank_snapshot *s)
{
    if (!g_multi_enabled || !rp_crx_ram_ptr(mgr) || id >= RP_CRX_ALGSMGR_LATCH_COUNT ||
        mgr != g_multi_manager || rp_crx_bank_snapshot(mgr, id, s) ||
        !rp_crx_snapshot_arena_valid(s) || s->backend != g_multi_backend ||
        s->mode != g_multi_mode || s->arena_base != g_multi_arena_base ||
        s->arena_size != g_multi_arena_size || s->hw_active != 0u)
        return -63;
    if (g_multi_pending[id])
    {
        const struct rp_crx_bank_snapshot *old = &g_multi_cleared[id];
        if (s->latch != 0u || s->context != old->context ||
            s->cursor < old->cursor ||
            memcmp(s->pointers, old->pointers, sizeof(s->pointers)) != 0)
            return -64;
    }
    return 0;
}

static int rp_crx_multi_after_stop(uint32_t bits, uint32_t id,
                                   uint32_t manager)
{
    struct rp_crx_bank_snapshot s;
    int rc = rp_crx_multi_validate(manager, id, &s);
    if (rc) return rc;
    /* STOP must follow a successful one-frame encode and native free;
     * the end-of-encode workspace state must have survived cleanup. */
    if (!g_multi_inflight || g_multi_current_id != id ||
        g_multi_clears >= g_multi_budget || g_active || g_free_pending ||
        g_poisoned || !g_session_clean_end ||
        g_session_good_encodes != 1u || g_session_encode_attempts != 1u ||
        g_session_multi_bits || g_session_bits != bits ||
        g_session_first_hw_latch != 0u ||
        !g_multi_encode_after_valid || s.latch != 1u ||
        s.context != g_multi_encode_after.context ||
        s.cursor != g_multi_encode_after.cursor ||
        memcmp(s.pointers, g_multi_encode_after.pointers, sizeof(s.pointers)) != 0)
        return -65;
    uint32_t end = s.arena_base + s.arena_size;
    uint32_t spare = end - s.cursor;
    /* Reserve room for native Canon still and any background CRX client.
     * This is only a heuristic; no exclusive arena ownership is implied. */
    if (spare < 0x4000u)
    {
        RP_CRX_TRACE("V16 LOW HEADROOM remain=%x; refuse clear", spare);
        return -66;
    }
    volatile uint32_t *latch = (volatile uint32_t *)(uintptr_t)
        (manager + RP_CRX_ALGSMGR_LATCH_OFFSET + 4u * id);
    uint32_t sequence = g_multi_clears + 1u;
    RP_CRX_TRACE("V16 RESET BEFORE seq=%d id=%d bits=%d cur=%x free=%x",
                 (int)sequence, (int)id, (int)bits, s.cursor, spare);
    /* Intent and snapshot recorded before the write, for fail-closed retry. */
    g_multi_inflight = 0u;
    g_multi_pending[id] = 1u;
    g_multi_cleared[id] = s;
    g_multi_clears++;
    rp_crx_barrier();
    *latch = 0u;
    rp_crx_barrier();
    uint32_t after = *latch;
    RP_CRX_TRACE("V16 RESET AFTER seq=%d id=%d was=%d now=%d total=%d",
                 (int)sequence, (int)id, (int)s.latch, (int)after,
                 (int)g_multi_clears);
    if (after != 0u)
    {
        g_poisoned = 1u;
        g_stats.poisoned = 1u;
        return -67;
    }
    rp_crx_log_snapshot("v16-postclear-preimage", id, &s);
    return 0;
}

int raw_vid_rp_crx_retry_state(void) { return (int)g_retry_state; }
int raw_vid_rp_crx_retry_bits(void) { return (int)g_retry_bits; }

void raw_vid_rp_crx_request_reclaim(void)
{
    g_reclaim_requested = 1u;
    rp_crx_barrier();
}

static int rp_crx_v22_snapshot_matches_tail(const struct rp_crx_bank_snapshot *now)
{
    if (!now || !g_reclaim_candidate_valid || !g_route_valid)
        return 0;
    const struct rp_crx_bank_snapshot *pre = &g_reclaim_pre;
    const struct rp_crx_bank_snapshot *tail = &g_reclaim_tail;
    if (!rp_crx_snapshot_arena_valid(now) ||
        *(volatile uint32_t *)(uintptr_t)
            (g_session_manager + RP_CRX_ALGSMGR_TIMER_OFFSET) != RP_CRX_TIMER_IDLE_SENTINEL ||
        now->backend != pre->backend || now->backend != tail->backend ||
        now->arena_base != pre->arena_base || now->arena_base != tail->arena_base ||
        now->arena_size != pre->arena_size || now->arena_size != tail->arena_size ||
        now->context != pre->context || now->context != tail->context ||
        now->mode != RP_CRX_MODE_LOSSLESS ||
        now->cursor != tail->cursor || tail->cursor <= pre->cursor ||
        now->latch != 1u || now->hw_active != 0u)
        return 0;
    for (uint32_t i = 0u; i < 5u; i++)
    {
        if (now->pointers[i] != tail->pointers[i])
            return 0;
        /* Every persistent workspace base captured after our fresh init must
         * lie inside the allocation interval being reclaimed. */
        if (tail->pointers[i] < pre->cursor || tail->pointers[i] >= tail->cursor)
            return 0;
    }
    return 1;
}

/* Guarded transaction.  path=1 runs after OUR final event-4 handler has
 * returned in the AlgsMgr task; path=2 is the worker fallback after all own
 * event-4 returns have been observed.  The LOW allocator semaphore prevents
 * a concurrent D_ALLOC cursor writer during the actual mutation. */
static void rp_crx_v22_record_reclaim(int rc, uint32_t path)
{
    g_reclaim_attempted = 1u;
    g_reclaim_last_rc = rc;
    g_reclaim_last_path = path;
    g_stats.reclaim_attempts++;
    g_stats.reclaim_last_rc = rc;
    g_stats.reclaim_last_path = path;
    g_stats.reclaim_route_type = g_route_type;
    g_stats.reclaim_route_id = g_route_id;
    if (rc != 0 && g_reclaim_write_started)
    {
        g_poisoned = 1u;
        g_stats.poisoned = 1u;
    }
    rp_crx_barrier();
}

static int rp_crx_v22_try_reclaim(uint32_t path)
{
    int rc = -90;
    if (g_reclaim_in_progress)
        return -108;
    if (g_reclaim_attempted)
        return g_reclaim_last_rc;

    /* Once the final manager-boundary attempt has observed a hard mismatch,
     * do not retry later merely because the bump cursor happens to equal the
     * old numerical value again (ABA).  Only LOW-semaphore contention is a
     * retryable path-1 condition. */
    if (!g_reclaim_requested || !g_reclaim_candidate_valid || !g_route_valid)
    {
        rc = -91;
        rp_crx_v22_record_reclaim(rc, path);
        return rc;
    }
    if (g_active || g_free_pending || g_free_callback_errors ||
        g_stats.unexpected_callbacks || g_poisoned ||
        g_route_id >= RP_CRX_ALGSMGR_LATCH_COUNT)
    {
        rc = -92;
        rp_crx_v22_record_reclaim(rc, path);
        return rc;
    }
    if (g_session_multi_bits || g_session_good_encodes == 0u)
    {
        rc = -93;
        rp_crx_v22_record_reclaim(rc, path);
        return rc;
    }

    uint32_t manager = *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    struct rp_crx_bank_snapshot before;
    if (manager != g_session_manager ||
        rp_crx_bank_snapshot(manager, g_route_id, &before) != 0 ||
        !rp_crx_v22_snapshot_matches_tail(&before))
    {
        rc = -94;
        rp_crx_v22_record_reclaim(rc, path);
        return rc;
    }

    uint32_t low_sem = *(volatile uint32_t *)(uintptr_t)RP_CRX_LOW_ALLOC_SEM_SLOT;
    if (low_sem == 0u)
    {
        rc = -95;
        rp_crx_v22_record_reclaim(rc, path);
        return rc;
    }
    if (take_semaphore_now((struct semaphore *)(uintptr_t)low_sem) != 0)
    {
        /* At the manager post-event boundary a busy LOW allocator can settle
         * before worker end_session; permit exactly that one fallback retry.
         * A worker-side failure is terminal for this session. */
        rc = -96;
        if (path != 1u)
            rp_crx_v22_record_reclaim(rc, path);
        return rc;
    }

    g_reclaim_in_progress = 1u;
    rp_crx_barrier();
    rc = -97;

    /* Re-read everything while holding exactly the semaphore used by
     * LOW_D_ALLOC.  Do not trust the pre-lock observation. */
    struct rp_crx_bank_snapshot locked;
    manager = *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    if (manager != g_session_manager ||
        rp_crx_bank_snapshot(manager, g_route_id, &locked) != 0 ||
        !rp_crx_v22_snapshot_matches_tail(&locked))
        goto out_unlock;

    volatile uint32_t *latch = (volatile uint32_t *)(uintptr_t)
        (manager + RP_CRX_ALGSMGR_LATCH_OFFSET + g_route_id * 4u);
    volatile uint32_t *cursor = (volatile uint32_t *)(uintptr_t)
        (locked.backend + 0x14u);

    /* Safer partial-failure order: clear the init latch first.  If anything
     * fails before cursor rewind, Canon will reinitialize on the next use but
     * the old tail remains reserved.  Cursor-first with latch=1 could expose
     * live stale pointers to a newly reused allocation. */
    g_reclaim_write_started = 1u;
    *latch = 0u;
    rp_crx_barrier();
    if (*latch != 0u || *cursor != locked.cursor ||
        *(volatile uint32_t *)(uintptr_t)RP_CRX_BACKEND_SLOT != locked.backend ||
        *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT != manager ||
        *(volatile uint32_t *)(uintptr_t)(manager + RP_CRX_ALGSMGR_MODE_OFFSET) != RP_CRX_MODE_LOSSLESS ||
        *(volatile uint32_t *)(uintptr_t)(manager + RP_CRX_ALGSMGR_TIMER_OFFSET) != RP_CRX_TIMER_IDLE_SENTINEL)
    {
        rc = -98;
        goto out_unlock;
    }

    *cursor = g_reclaim_pre.cursor;
    rp_crx_barrier();
    if (*cursor != g_reclaim_pre.cursor || *latch != 0u)
    {
        rc = -99;
        goto out_unlock;
    }

    g_encoder_bank_bits[g_route_id] = 0u;
    g_bank_first_width[g_route_id] = 0u;
    g_bank_first_height[g_route_id] = 0u;
    g_encoder_latched_bits = 0u;
    g_reclaim_candidate_valid = 0u;
    g_stats.reclaim_successes++;
    rc = 0;

out_unlock:
    give_semaphore((struct semaphore *)(uintptr_t)low_sem);
    g_reclaim_in_progress = 0u;
    rp_crx_barrier();
    rp_crx_v22_record_reclaim(rc, path);
    return rc;
}

static uint32_t rp_crx_session_state_cb(uint32_t a0, uint32_t a1,
                                        uint32_t a2, uint32_t a3, uint32_t a4)
{
    uint32_t orig = g_session_state_orig_cb;
    uint32_t is_our_free = 0u;

    /* The event-4 payload is {handle, callback, opaque}.  Identify ours before
     * Canon frees that payload; do not rely on counts of unrelated Canon
     * FreeEncBuffer events. */
    if (a2 == 4u && a4 == 0x0cu && rp_crx_ram_ptr(a3))
    {
        volatile uint32_t *payload = (volatile uint32_t *)(uintptr_t)a3;
        if (payload[1] == (uint32_t)(uintptr_t)&rp_crx_free_complete_cb)
            is_our_free = 1u;
    }

    g_session_state_hits++;
    rp_crx_barrier();
    uint32_t rc = orig ? ((rp_crx_state_cb_fn)(uintptr_t)orig)(a0,a1,a2,a3,a4) : 7u;

    if (a2 == 4u)
        g_session_state_event4++;
    if (is_our_free)
    {
        if (g_reclaim_requested && g_free_pending == 0u && !g_active &&
            !g_reclaim_attempted)
            (void)rp_crx_v22_try_reclaim(1u);
        /* Publish post-handler completion only after a possible manager-path
         * reclaim has completed, so worker fallback cannot race the write. */
        g_free_post_completed++;
        rp_crx_barrier();
    }
    rp_crx_barrier();
    return rc;
}

static int rp_crx_v22_install_state_wrapper(void)
{
    uint32_t manager = *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    if (!rp_crx_ram_ptr(manager) || manager != g_session_manager || g_state_probe_armed)
        return -100;
    uint32_t state = *(volatile uint32_t *)(uintptr_t)(manager + 0x0cu);
    if (!rp_crx_ram_ptr(state))
        return -101;
    volatile uint32_t *st = (volatile uint32_t *)(uintptr_t)state;
    uint32_t orig = st[3];
    uint32_t hook = (uint32_t)(uintptr_t)&rp_crx_session_state_cb;
    if (!orig || orig == hook || g_session_state_installed)
        return -102;
    g_session_state = state;
    g_session_state_orig_cb = orig;
    g_session_state_hits = 0u;
    g_session_state_event4 = 0u;
    g_free_post_completed = 0u;
    rp_crx_barrier();
    st[3] = hook;
    rp_crx_barrier();
    if (st[3] != hook)
    {
        g_session_state = 0u;
        g_session_state_orig_cb = 0u;
        return -103;
    }
    g_session_state_installed = 1u;
    rp_crx_barrier();
    return 0;
}

static int rp_crx_v22_restore_state_wrapper(void)
{
    if (!g_session_state_installed)
        return 0;
    if (!rp_crx_ram_ptr(g_session_state) || !g_session_state_orig_cb)
        return -104;
    volatile uint32_t *st = (volatile uint32_t *)(uintptr_t)g_session_state;
    uint32_t hook = (uint32_t)(uintptr_t)&rp_crx_session_state_cb;
    if (st[3] != hook)
        return -105;
    st[3] = g_session_state_orig_cb;
    rp_crx_barrier();
    if (st[3] != g_session_state_orig_cb)
        return -106;
    g_session_state_installed = 0u;
    g_session_state = 0u;
    g_session_state_orig_cb = 0u;
    rp_crx_barrier();
    return 0;
}

static int rp_crx_v22_wait_post_free(uint32_t target)
{
    uint32_t t0 = get_us_clock();
    while (g_free_post_completed < target)
    {
        if ((uint32_t)(get_us_clock() - t0) >= RP_CRX_FREE_TIMEOUT_US)
            return -107;
        msleep(1);
        rp_crx_barrier();
    }
    return 0;
}

static uint32_t rp_crx_state_probe_cb(uint32_t a0, uint32_t a1,
                                      uint32_t a2, uint32_t a3, uint32_t a4)
{
    uint32_t state = g_state_probe_state;
    uint32_t orig = g_state_probe_orig_cb;

    g_state_probe_a0 = a0;
    g_state_probe_a1 = a1;
    g_state_probe_a2 = a2;
    g_state_probe_a3 = a3;
    g_state_probe_a4 = a4;
    g_state_probe_hits++;

    /* One-shot self-unhook before entering Canon.  E058F910 ABI is the
     * DryOS state-object transition ABI:
     *   a0 = state object, a1 = event x/owner, a2 = input/event number,
     *   a3 = event payload, a4 = payload size.
     * Restore the generic transition callback before doing any inspection so
     * an assert/early return cannot leave our wrapper installed. */
    if (state != 0u && a0 == state && orig != 0u)
    {
        volatile uint32_t *st = (volatile uint32_t *)(uintptr_t)state;
        if (st[3] == (uint32_t)(uintptr_t)&rp_crx_state_probe_cb)
            st[3] = orig;
    }
    g_state_probe_armed = 0u;
    rp_crx_barrier();

    /* Passive proof that event 2 is our synthetic StartEncode job.  Public
     * E0651986 posts (manager, 2, job, 0x70).  The job owns a copied 0x2c
     * descriptor array at job[2], and job[5] is the active descriptor index. */
    if (a2 == 2u && a3 >= 0x1000u && a3 < 0xE0000000u &&
        (a3 & 3u) == 0u && a4 == 0x70u)
    {
        volatile uint32_t *job = (volatile uint32_t *)(uintptr_t)a3;
        uint32_t idx = job[5];
        uint32_t desc_base = job[2];
        if (idx < 4u && desc_base >= 0x1000u && desc_base < 0xE0000000u &&
            (desc_base & 3u) == 0u)
        {
            volatile uint32_t *d =
                (volatile uint32_t *)(uintptr_t)(desc_base + idx * 0x2cu);
            g_state_probe_job_type = d[0];
            g_state_probe_job_bits = d[5];
            g_state_probe_job_src = d[6];
            if (d[0] == g_state_probe_req_type &&
                d[5] == g_state_probe_req_bits &&
                d[6] == g_state_probe_req_src)
                g_state_probe_matches++;
            else
                g_state_probe_mismatches++;
        }
        else
        {
            g_state_probe_mismatches++;
        }
    }
    else
    {
        g_state_probe_mismatches++;
    }
    rp_crx_barrier();

    if (orig == 0u)
        return 7u;

    return ((rp_crx_state_cb_fn)(uintptr_t)orig)(a0, a1, a2, a3, a4);
}

static int rp_crx_arm_state_probe(uint32_t type, uint32_t bits, uint32_t src)
{
    uint32_t manager = *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    if (manager < 0x1000u || manager >= 0xE0000000u || (manager & 3u))
        return -40;

    uint32_t state = *(volatile uint32_t *)(uintptr_t)
        (manager + 0x0cu);
    if (state < 0x1000u || state >= 0xE0000000u || (state & 3u))
        return -41;

    volatile uint32_t *st = (volatile uint32_t *)(uintptr_t)state;
    uint32_t orig = st[3];
    uint32_t hook = (uint32_t)(uintptr_t)&rp_crx_state_probe_cb;
    if (orig == 0u || orig == hook)
        return -42;

    /* Refuse to overwrite an earlier unresolved probe. */
    if (g_state_probe_armed)
        return -43;

    g_state_probe_req_type = type;
    g_state_probe_req_bits = bits;
    g_state_probe_req_src = src;
    g_state_probe_state = state;
    g_state_probe_orig_cb = orig;
    g_state_probe_a0 = 0u;
    g_state_probe_a1 = 0u;
    g_state_probe_a2 = 0u;
    g_state_probe_a3 = 0u;
    g_state_probe_a4 = 0u;
    g_state_probe_job_type = 0xffffffffu;
    g_state_probe_job_bits = 0u;
    g_state_probe_job_src = 0u;
    g_state_probe_armed = 1u;
    rp_crx_barrier();

    st[3] = hook;
    rp_crx_barrier();
    if (st[3] != hook)
    {
        g_state_probe_armed = 0u;
        rp_crx_barrier();
        return -44;
    }

    RP_CRX_TRACE("STATEPROBE arm mgr=%x state=%x cb=%x hook=%x",
                 manager, state, orig, hook);
    return 0;
}

/* Called by Canon AlgsMgr completion context through BLX.  Keep this bounded
 * RAM-only: no logging, allocation, waits, semaphores, FIO or M2M. */
static void rp_crx_free_complete_cb(uint32_t opaque, uint32_t status)
{
    (void)opaque;
    if (status != 0u)
        g_free_callback_errors++;
    if (g_free_pending != 0u)
        g_free_pending--;
    g_free_completed++;
    rp_crx_barrier();
}

static void rp_crx_complete_cb(uint32_t opaque, uint32_t *handles, uint32_t count)
{
    if (!g_active || opaque != g_active_token)
    {
        g_stats.unexpected_callbacks++;
        rp_crx_barrier();
        return;
    }

    uint32_t n = count;
    if (n > RP_CRX_RESULT_MAX)
        n = RP_CRX_RESULT_MAX;

    g_callback_token = opaque;
    g_callback_count = count;
    for (uint32_t i = 0; i < RP_CRX_RESULT_MAX; i++)
        g_callback_handles[i] = (handles && i < n) ? handles[i] : 0u;

    rp_crx_barrier();
    g_done = 1;
    rp_crx_barrier();
#if RP_CRX_EVENT_WAIT
    /* DryOS declares give_semaphore IRQ-safe on RP. No blocking, allocation
     * or log work is performed in Canon's completion context. */
    if (g_completion_sem)
        (void)give_semaphore(g_completion_sem);
#endif
}

int raw_vid_rp_crx_preflight_mode(uint32_t requested_bits, int lossy)
{
    if (!is_camera("RP", "1.6.0"))
        return -1;
    if (g_active)
        return -2;
    if (g_poisoned)
        return -3;
    if (requested_bits != 10u && requested_bits != 12u && requested_bits != 14u)
        return -4;
    if (g_free_pending != 0u)
        return -6;

    /* v23.2 production policy is dynamic-only.  There is no fixed-route /
     * persistent-bank fallback: every CRX/C-RAW session must start on the
     * first compatible latch-0 D-slot and must later prove an exact tail that
     * can be reclaimed. */
    rp_crx_barrier();

    uint32_t manager =
        *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    if (g_state_probe_armed)
        return -81;
    if (!rp_crx_ram_ptr(manager))
        return -82;

    uint32_t type = 0u;
    uint32_t enc_id = 0u;
    if (rp_crx_v22_pick_free_route(manager, requested_bits, lossy,
                                   &type, &enc_id) != 0)
        return -83;

    struct rp_crx_bank_snapshot s;
    if (rp_crx_bank_snapshot(manager, enc_id, &s) != 0 ||
        !rp_crx_snapshot_arena_valid(&s) ||
        s.mode != RP_CRX_MODE_LOSSLESS ||
        s.latch != 0u || s.hw_active != 0u)
        return -84;

    /* The selected slot must be software-free too.  Stale workspace pointer
     * fields are allowed because Canon reinitializes them when latch==0. */
    if (g_encoder_bank_bits[enc_id] != 0u)
        return -85;

    g_route_valid = 1u;
    g_route_bits = requested_bits;
    g_route_lossy = lossy ? 1u : 0u;
    g_route_type = type;
    g_route_id = enc_id;
    g_reclaim_candidate_valid = 0u;
    g_stats.reclaim_route_type = type;
    g_stats.reclaim_route_id = enc_id;
    g_bank_first_width[enc_id] = 0u;
    g_bank_first_height[enc_id] = 0u;

    RP_CRX_TRACE("V23.2 PREFLIGHT dynamic bits=%d lossy=%d route=%d/%d cur=%x",
                 (int)requested_bits, lossy ? 1 : 0,
                 (int)type, (int)enc_id, s.cursor);
    return 0;
}

int raw_vid_rp_crx_preflight(uint32_t requested_bits)
{
    return raw_vid_rp_crx_preflight_mode(requested_bits, 0);
}

int raw_vid_rp_crx_begin_session_mode(int lossy)
{
    if (!is_camera("RP", "1.6.0"))
        return -1;
    if (g_active)
        return -2;
    if (g_poisoned)
        return -3;
    if (g_free_pending != 0u)
        return -4;

    /* Historical one-shot retry/multi-clear experiments are not part of the
     * v23.2 recorder lifecycle and cannot consume or veto a normal session. */

    uint32_t manager =
        *(volatile uint32_t *)(uintptr_t)RP_CRX_ALGSMGR_OWNER_SLOT;
    if (manager < 0x1000u || manager >= 0xE0000000u || (manager & 3u))
        return -8;

    g_session_manager = manager;
    g_session_manager_mode = *(volatile uint32_t *)(uintptr_t)
        (manager + RP_CRX_ALGSMGR_MODE_OFFSET);
    g_session_lossy = lossy ? 1u : 0u;
    g_session_bits = 0u;
    g_session_first_hw_latch = 0xffffffffu;
    g_session_first_sw_bits = 0xffffffffu;
    g_session_encode_attempts = 0u;
    g_session_good_encodes = 0u;
    g_session_clean_end = 0u;
    memset(&g_tail_audit, 0, sizeof(g_tail_audit));
    g_tail_audit.enabled = 0u;
    g_v19_active = 0u;
    g_v19_pre_valid = 0u;
    g_v19_between_changed = 0u;
    g_v19_epoch_suspect = 0u;
    g_session_multi_bits = 0u;
    g_reclaim_requested = 0u;
    g_reclaim_in_progress = 0u;
    g_reclaim_attempted = 0u;
    g_reclaim_last_rc = 0;
    g_reclaim_last_path = 0u;
    g_reclaim_write_started = 0u;
    g_reclaim_candidate_valid = 0u;
    g_free_post_completed = 0u;
    g_first_encode_before_valid = 0u;
    /* Always reset per-session allocation observations.  Old retry/multi
     * globals are intentionally ignored by the production recorder. */
    g_first_alloc_valid = 0u;
    g_first_alloc_bytes = 0u;
    g_first_init_after_valid = 0u;
    g_last_good_width = 0u;
    g_last_good_height = 0u;
    memset(&g_stats, 0, sizeof(g_stats));
    g_free_completed = 0;
    g_free_callback_errors = 0;
    g_done = 0;
    g_callback_token = 0;
    g_callback_count = 0;
    for (uint32_t i = 0; i < RP_CRX_RESULT_MAX; i++)
        g_callback_handles[i] = 0;
    if (!g_route_valid || g_route_lossy != (uint32_t)(lossy ? 1 : 0))
        return -11;
#if RP_CRX_EVENT_WAIT
    if (!g_completion_sem)
    {
        g_completion_sem = create_named_semaphore("rp_crx_done", SEM_CREATE_LOCKED);
        if (!g_completion_sem)
            RP_CRX_TRACE("EVENT semaphore unavailable, using poll fallback");
    }
#endif
    int wrc = rp_crx_v22_install_state_wrapper();
    if (wrc)
        return wrc;
    RP_CRX_TRACE("V23.2 WRAP install route=%d/%d state=%x orig=%x",
                 (int)g_route_type,(int)g_route_id,
                 g_session_state,g_session_state_orig_cb);

    RP_CRX_TRACE("SESSION begin lossy=%d mgr=%x mode_before=%x",
                 (int)g_session_lossy, manager, g_session_manager_mode);
    rp_crx_barrier();
    return 0;
}
