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
