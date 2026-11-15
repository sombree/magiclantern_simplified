#include "module.h"
#include "dryos.h"
#include <string.h>

#include "raw.h"
#include "edmac-memcpy.h"
#include "fps.h"
#include "exmem.h"
#include "propvalues.h"  /* lv */

#include "raw_vid.h"
#include "raw_vid_worker.h"
#include "raw_vid_event_pusher.h"
#include "rp_backend.h"
#include "rp_crx.h"
#include "rp_crx_format.h"
#include "rp_w3_matrix.h"
#include "mem_defs.h"
#include "rp_cache_ops.h"
#include "mlv_3.h"
#include "rp_adaptive_writer.h"
#include "rp_pressure_v112.h"
#include "rp_pressure_v113.h"

/*
 * EOS RP start-isolation ladder.
 *
 * v11a proved raw REC event ownership.  Increase this one value one step per
 * hardware test; stages below 6 deliberately unwind before capture can start.
 *
 *   0 = COMMAND_START reaches worker only
 *   1 = + allocate/free the two crop accumulation buffers
 *   2 = + raw_vid_rp_init() (post semaphore + READY task + RAW pBlock bind)
 *   3 = + raw_lv_request()/raw_lv_release() and RAW geometry update
 *   4 = + create/close the MLV file
 *   5 = + create/write MLV session headers
 *   6 = + allocate/free 1 full-frame capture buffer
 *   7 = + allocate/free 2 full-frame capture buffers
 *   8 = + allocate/free 3 full-frame capture buffers
 *  10 = read-only Canon ch3 Complete callback-table RAM-tap probe
 *  14 = full backend + READY->queue->worker, consume/release only (no crop)
 *  15 = one CPU crop, uncached -> uncached, with 16-byte-aligned x=192
 *  16 = one CPU crop, uncached -> cacheable, x=192; selective D-cache clean
 *  17 = full recorder, x=192, cacheable crop accumulation buffers
 *  18 = full recorder, x=192, uncached crop accumulation buffers (control)
 *  19 = RP mem2mem resource lock only
 *  20 = + RD46/WR13/Boomer/Complete setup, no address/geometry
 *  21 = + linear 4 KiB address/geometry/mode, no start
 *  22 = + linear 4 KiB transfer and byte-for-byte verify
 *  23 = RAW-like 2D 2240x720 stride geometry, no start
 *  24 = + actual 2D transfer, byte-for-byte crop verify + dst guard
 *  26..31 = synthetic rows x stride/no-stride matrix
 *  32 = one real ch3 RAW frame -> RP mem2mem crop -> one-frame MLV -> auto-stop
 *  33 = continuous real RAW -> persistent RP mem2mem resource session -> MLV
 *  34 = continuous real RAW -> per-frame transient RP mem2mem resources -> MLV
 *  35 = S34 + phase-guard experiment (rejected: worsened 4K timing)
 *  36 = S34 + lower raw_write task priority (0x11 vs legacy 0x9)
 *       v20.2 standard CRX capture: direct held-W3 ROI -> E0651986.
 *       v21.0 storage branch: REV2 exact sample + mlv_lite-style dynamic
 *       compressed slots (shrink/free) + zero-copy contiguous FIO batching.
 *       raw_writer performs FIO only; no writer-side EDMAC and no batch arena.
 *       Latch/retry/session and v18/v19/v20 cursor-audit experiments keep the
 *       original W3 -> R46/W13 tight-buffer -> E0651986 input path.
 */
#ifndef RP_START_PROBE_STAGE
#define RP_START_PROBE_STAGE 36
#endif

#define RP_START_TRACE(fmt, ...) \
    DryosDebugMsg(0, 15, "RPVIDX " fmt, ##__VA_ARGS__)

/* Historical diagnostic stage IDs have gaps. Reject ALL undefined values
 * before acquiring any capture resources; do not reinterpret them as REC. */
static int rp_start_stage_supported(void)
{
    return (RP_START_PROBE_STAGE >= 0 && RP_START_PROBE_STAGE <= 8) ||
           RP_START_PROBE_STAGE == 10 ||
           (RP_START_PROBE_STAGE >= 14 && RP_START_PROBE_STAGE <= 24) ||
           (RP_START_PROBE_STAGE >= 26 && RP_START_PROBE_STAGE <= 36);
}

static struct msg_queue *event_q = NULL;
static FILE *raw_vid_fp = NULL;

/* RP v21.0: return the storage side to the proven mlv_lite model.
 *
 * Canon ownership / true Complete / the 3-slot W3 source ring and CRX/EDMAC
 * frontend remain unchanged.  shoot_malloc_suite(0) supplies the output pool.
 * Each chunk is represented by full-size FREE slots plus RESERVED metadata
 * entries.  After the actual compressed size is known, a CRX VIDF is sector
 * aligned with leading frameSpace and shrink_slot-style pointer redistribution
 * gives the unused tail to the following FREE/RESERVED slots.  This preserves
 * physical adjacency between variable-size frames.
 *
 * The persistent raw_writer never performs EDMAC/M2M and never copies frames
 * into a second batch arena.  It only groups already-adjacent FULL slots and
 * calls FIO_WriteFile(first->start, sum(slot->used)). */
static struct msg_queue *rp_write_q = NULL;
static volatile uint32_t rp_writer_jobs_queued = 0;
static volatile uint32_t rp_writer_jobs_done = 0;
static volatile uint32_t rp_writer_post_fail = 0;
static volatile uint32_t rp_writer_max_us = 0;
static volatile uint32_t rp_writer_batches = 0;
static volatile uint32_t rp_writer_max_batch_frames = 0;
static volatile uint32_t rp_writer_max_batch_bytes = 0;
static volatile uint32_t rp_writer_rate_x100 = 0; /* decimal MB/s * 100 */
static volatile uint32_t rp_writer_io_fail = 0;
static volatile uint32_t rp_writer_coalesce_waits = 0;
static volatile uint32_t rp_writer_coalesce_hits = 0;
static volatile uint32_t rp_writer_coalesce_timeouts = 0;
static volatile uint32_t rp_writer_coalesce_max_us = 0;
static volatile uint32_t rp_writer_coalesce_total_us = 0;
static volatile uint32_t rp_slot_starves = 0;
static volatile uint32_t rp_slot_min_free = 0;
/* Adaptive tuning is writer-owned; the two rate windows also have producer
 * updates and are sampled read-only by the stop summary/status API. */
static struct rp_adapt_writer rp_adapt;
static struct rp_v113_history rp_v113_io_history;
static volatile uint32_t rp_writer_drain_requested = 0u;
/* Latched across sessions. On a drain timeout the writer may still own a
 * shoot-suite buffer and FILE*: preserve both and require a cold reboot. */
static volatile uint32_t rp_writer_storage_lockout = 0u;
static volatile uint32_t rp_writer_ingress_rate_x100 = 0u;
static volatile uint32_t rp_writer_drain_rate_x100 = 0u;
static volatile uint32_t rp_writer_recent_frame_bytes = 0u;
static uint32_t rp_ingress_window_t0 = 0u;
static uint64_t rp_ingress_window_bytes = 0u;
static uint32_t rp_drain_window_t0 = 0u;
static uint64_t rp_drain_window_bytes = 0u;


/* v110 output pool: two explicit 32-MiB MEM3 suites, not shoot-all-RAM.
 * SSRaw is a separately gated FIO experiment: 0 until source-address
 * compatibility is demonstrated. A core-resident broker owns Canon callbacks. */
#define RP_V110_MEM3_2X32 1
#define RP_V110_SSRAM_OUTPUT 1
#define RP_V110_MEM3_BLOCK (32u * 1024u * 1024u)
#define RP_FRAME_SLOT_MAX 1536
/* Preserve v26's known-working queue budget: 1536 slot descriptors do not
 * require 1536 pending FIO jobs. A larger queue can fail module init before
 * CBR_KEYPRESS_RAW is registered, making Canon own REC again. */
#define RP_V110_WRITE_QUEUE_MAX 256
#define RP_FRAME_SLOT_MIN 3
#define RP_FRAME_SLOT_ALIGN 512u
#define RP_FRAME_CHUNK_MAX 32
/* 128 MiB is an experimental FIO request ceiling, not a confirmed RP DMA
 * descriptor limit.  Adaptive stage unlock starts at the old ~31 MiB range. */
#define RP_FIO_MAX_BATCH_BYTES RP_ADAPT_MAX_BATCH_BYTES
/* Canon AlgsMgr reserves roughly 1.25x the packed input for CRX result
 * buffers on RP.  The previous fixed +512 KiB reserve was too small for a
 * runtime-observed 1920x1080/12 result: 3,110,400 input -> 3,727,968 CRX.
 * Reserve 25% expansion plus a small container/header margin instead. */
#define RP_CRX_SLOT_EXPANSION_DIV   4u
#define RP_CRX_SLOT_META_HEADROOM   (64u * 1024u)
#define RP_CRX_RING_MAX            256u
/* CRX lossless on RP may use only ~25-40% of the conservative max slot.
 * Extra RESERVED descriptors cost no shoot memory; they only provide enough
 * metadata for shrink_slot() to expose that recovered RAM as additional
 * frames.  mlv_lite uses one RESERVED per max slot; CRX needs more because
 * its compression ratio is substantially deeper than the historical LJ92
 * case. */
#define RP_CRX_RESERVED_PER_FULL    3u
#define RP_CRAW_RESERVED_PER_FULL   7u
#define RP_RAW_RESERVED_PER_FULL    1u

/* Pixel-exact direct-W3 -> CRX round-trip has already been proven on hardware.
 * Keep the old verifier code available, but production/continuous v21.0 must
 * not reserve a reference slot or force a one-frame recording. */
#define RP_CRX_ROUNDTRIP_VERIFY     0
#define RP_CRX_REF_VERSION          1u
#define RP_CRX_REF_HEADER_BYTES     64u

/* CONFIG_MEMORY_SRM_NOT_WORKING remains enabled.  SSRaw A is obtained
 * only by the optional native 0x2C broker, NOT the generic ML SRM heap. */
#define RP_V3D_USE_SRM 0

/* Adaptive writer v24: buffer-pressure limit is always stronger than the
 * learned performance target; STOP cancels coalescing immediately. */
#define RP_COALESCE_START_FREE 8u
#define RP_COALESCE_ABORT_FREE 5u

enum rp_frame_slot_state
{
    RP_FRAME_SLOT_UNUSED = 0,
    RP_FRAME_SLOT_FREE,
    RP_FRAME_SLOT_CAPTURING,
    RP_FRAME_SLOT_FULL,
    RP_FRAME_SLOT_WRITING,
    RP_FRAME_SLOT_REFERENCE,
    RP_FRAME_SLOT_RESERVED,
};

struct rp_frame_slot
{
    char *start;             /* 512-aligned uncached slot start */
    char *end;               /* exclusive slot end */
    uint32_t used;           /* complete padded VIDF block size */
    uint32_t frame_number;   /* recorder-local frame number */
    uint16_t chunk_index;
    uint16_t chunk_pos;
    volatile uint32_t state;
};

struct rp_crx_ref_header
{
    char magic[8];              /* "RPCRXR1" + NUL */
    uint32_t version;
    uint32_t width;
    uint32_t height;
    uint32_t bits_per_pixel;
    uint32_t row_bytes;
    uint32_t payload_bytes;
    uint32_t frame_number;
    uint32_t copy_us;
    uint32_t reserved[6];
};

typedef char rp_crx_ref_header_must_be_64_bytes[
    (sizeof(struct rp_crx_ref_header) == RP_CRX_REF_HEADER_BYTES) ? 1 : -1
];

struct rp_frame_chunk
{
    char *_buf;              /* legacy backing ptr; NULL for v3D shoot suite */
    char *start;             /* 512-aligned uncached chunk start */
    uint32_t bytes;          /* bytes represented by slots */
    uint32_t first_slot;
    uint32_t slot_count;
};

static struct rp_frame_slot rp_frame_slots[RP_FRAME_SLOT_MAX];
static struct rp_frame_chunk rp_frame_chunks[RP_FRAME_CHUNK_MAX];
static struct memSuite *rp_frame_suite = NULL; /* legacy fallback only */
static struct memSuite *rp_v110_mem3[2] = {NULL, NULL};
static uint32_t rp_v110_ssraw_addr = 0;
static uint32_t rp_v110_ssraw_bytes = 0;
static volatile uint32_t rp_v110_owner_quarantine = 0;
#if RP_V110_SSRAM_OUTPUT
/* Implemented in resident src/debug.c after RPSS103/v105 patches.
 * A live lease is valid only after callback & native ownership confirmation. */
extern int rp_ssraw_v110_acquire(uint32_t *base, uint32_t *bytes);
extern int rp_ssraw_v110_release(void);
extern uint32_t rp_ssraw_v110_state(void);
#endif
static uint32_t rp_frame_slot_count = 0;
static uint32_t rp_frame_chunk_count = 0;
static uint32_t rp_frame_slot_cursor = 0;
/* Last output frame placement; used only as a spatial locality hint. */
static uint32_t rp_v112_last_capture = RP_FRAME_SLOT_MAX;
static volatile uint32_t rp_v112_tail_hits = 0, rp_v112_fallbacks = 0;
static uint32_t rp_frame_slot_stride = 0;
static uint32_t rp_frame_suite_bytes = 0;
static uint32_t rp_frame_suite_chunks = 0;
static uint32_t rp_frame_output_bytes = 0;
static uint32_t rp_frame_waste_bytes = 0;

/* v21.0 dynamic compressed-slot allocator.
 * Metadata moves are tiny but are shared by raw_v_worker and raw_writer. */
static struct semaphore *rp_slot_sem = NULL;
static uint32_t rp_slot_shrink_ok = 0;
static uint32_t rp_slot_shrink_deferred = 0;
static uint32_t rp_slot_free_rebuilds = 0;
static uint32_t rp_slot_reserved_peak = 0;

/* CRXI v2 becomes available only after frame 0 completes CRX encode.
 * Queue one special writer token before the first VIDF so all hot-path file
 * I/O still lives in raw_writer. */
#define RP_WRITE_HEADER_TOKEN ((struct rp_frame_slot *)(uintptr_t)1u)
static volatile uint32_t rp_header_write_pending = 0;
static volatile uint32_t rp_header_write_done = 0;
static volatile uint32_t rp_header_write_failed = 0;
static char *rp_header_write_ptr = NULL;
static uint32_t rp_header_write_size = 0;

// ensures we never attempt an edmac copy while the last is in progress
static struct semaphore *edmac_sem = NULL;

static volatile enum recording_state recording_state = INACTIVE;

/* RP worker-path isolation telemetry.  Only normal task context touches this. */
static uint32_t rp_worker_probe_frames = 0;
static uint32_t rp_worker_m2m_failed = 0;
static int rp_record_codec = RAW_VID_CODEC_RAW;
/* Actual serialized W3 container, frozen at normal REC start. */
static int rp_record_wire_bpp = 14;
/* v23.2: native CRX/C-RAW is dynamic-only and always stays on the
 * standard direct-W3 input path; there is no fixed-route allocator mode. */
static uint32_t rp_crx_frame_limit = 1u; /* 0 = continuous; frozen per session */
static uint32_t rp_still_session_active = 0u; /* photo LV session, any duration */
static uint32_t rp_crx_frames_ok = 0;
static uint32_t rp_crx_frames_failed = 0;
static uint32_t rp_crx_total_input_us = 0;
static uint32_t rp_crx_total_encode_us = 0;
static uint32_t rp_crx_total_detach_us = 0;
static uint32_t rp_crx_total_compact_us = 0;
static uint32_t rp_crx_total_payload_bytes = 0;
static uint32_t rp_crx_total_m2m_blocks = 0;
static uint32_t rp_crx_release_failures = 0;

/* MLV CRX container revision 2 stream state.
 * The exact Canon CMP1 is latched from the first successful encoder result,
 * copied once into CRXI v2, then compared byte-for-byte on later frames.
 * Current direct/staged encoder input is in the same numeric domain as RAWI,
 * therefore valueShift is explicitly zero; a CMP1 nBits mismatch fails closed. */
static uint8_t rp_crx_cmp1_cache[RP_CRX_META_MAX_BYTES];
static uint32_t rp_crx_cmp1_size = 0;
static int32_t rp_crx_value_shift = RP_CRX_VALUE_SHIFT_CURRENT;

/* v20.2 input-path telemetry. "direct" means the encoder consumed the held
 * completed W3 source directly; "staged" is the legacy tight R46/W13 input
 * path retained for latch/retry/session/cursor experiments. */
static uint32_t rp_crx_direct_w3_frames = 0;
static uint32_t rp_crx_staged_input_frames = 0;

static int rp_crx_standard_direct_w3_allowed(void)
{
    /* v23.2 dynamic-only recorder: normal CRX/C-RAW always uses the proven
     * held-W3 direct input path. The old latch/retry/session/tail menu paths
     * are no longer user-armable and cannot force staged M2M. */
    return 1;
}

/* One exact pre-StartEncode input reference for the v6 round-trip test.
 * rp_crx_ref_slot is never queued to the writer while capture is active. */
static struct rp_frame_slot *rp_crx_ref_slot = NULL;
static uint32_t rp_crx_ref_width = 0;
static uint32_t rp_crx_ref_height = 0;
static uint32_t rp_crx_ref_bits = 0;
static uint32_t rp_crx_ref_row_bytes = 0;
static uint32_t rp_crx_ref_payload_bytes = 0;
static uint32_t rp_crx_ref_frame_number = 0;
static uint32_t rp_crx_ref_copy_us = 0;

/* Single worker task owns these scratch arrays.  Keep CRX scatter planning
 * off the 0x1000-byte raw_v_worker stack.  v21.4 batches all tiny alignment
 * heads into one R46/W13 SG transaction and all aligned bulk segments into a
 * second SG transaction, instead of creating/destroying the native resource
 * up to twice per Canon segment. */
static uint32_t rp_crx_src_addr[RP_CRX_MAX_SEGMENTS];
static uint32_t rp_crx_src_size[RP_CRX_MAX_SEGMENTS];
static uint32_t rp_crx_head_src[RP_CRX_MAX_SEGMENTS];
static uint32_t rp_crx_head_size[RP_CRX_MAX_SEGMENTS];
static void *rp_crx_head_scratch[RP_CRX_MAX_SEGMENTS];
static uint8_t *rp_crx_head_final[RP_CRX_MAX_SEGMENTS];
static uint32_t rp_crx_bulk_src[RP_CRX_MAX_SEGMENTS];
static uint32_t rp_crx_bulk_size[RP_CRX_MAX_SEGMENTS];
static void *rp_crx_bulk_dst[RP_CRX_MAX_SEGMENTS];

/* Stage 35 writer-phase guard.  The 4K failure signature is a large FIO
 * write entering the storage stack immediately before evfVDInterruptCBR /
 * blanking.  A successful M2M crop completes early enough in the following
 * frame cycle to provide a cheap task-context phase reference without adding
 * any work to Canon IRQ/callback paths. */
static volatile uint32_t rp_last_m2m_complete_us = 0;

static void rp_wait_fio_safe_phase(void)
{
    if (!is_camera("RP", "1.6.0") || RP_START_PROBE_STAGE != 35)
        return;

    const uint32_t fresh_us = 6000u;
    const uint32_t timeout_us = 70000u;
    uint32_t wait_start = get_us_clock();

    while (1)
    {
        uint32_t now = get_us_clock();
        uint32_t last = rp_last_m2m_complete_us;
        uint32_t age = now - last;

        if (last != 0 && age <= fresh_us)
        {
            RP_START_TRACE("S35 FIO phase PASS age=%d wait=%d",
                           (int)age, (int)(now - wait_start));
            return;
        }

        if ((uint32_t)(now - wait_start) >= timeout_us)
        {
            RP_START_TRACE("S35 FIO phase TIMEOUT last=%x age=%d",
                           last, (int)age);
            return;
        }

        msleep(1);
    }
}

static struct mlv_session session = {
    .fps = 0,
    .res_x = 0,
    .res_y = 0,
    .bits_per_pixel = 14,
    .crop_offset_x = 0, // FIXME we want these correct so mlvapp can do FPM stuff, etc
    .crop_offset_y = 0,
    .pan_offset_x = 0,
    .pan_offset_y = 0,
    .image_data_alignment = EDMAC_ALIGNMENT,
    .native_crx = 0
};

enum recording_state get_recording_state(void)
{
    return recording_state;
}

// lv_save_raw trick (which we trigger indirectly via raw.c, raw_lv_request(),
// has given us a buffer of whatever size DryOS wants for fullsize LV.
// This experimental raw_vidx build is RP-only and crops synchronously on CPU;
// RP mem-to-mem EDMAC helpers are intentionally not referenced.
struct crop_buf 
{
    char *_buf; // Unaligned, underlying buffer, used with malloc/free.
    char *start; // Start of aligned buf, for DMA ops, etc.
    char *cur; // Position that data should write to.
    char *end; // Do not write to this location or later.
    volatile int writing; // asynchronous FIO task owns this buffer
};

static int rp_slot_lock(void)
{
    if (rp_slot_sem == NULL)
        return -1;
    return take_semaphore(rp_slot_sem, 1000);
}

static void rp_slot_unlock(void)
{
    if (rp_slot_sem != NULL)
        give_semaphore(rp_slot_sem);
}

static uint32_t rp_slot_region_size_locked(uint32_t i)
{
    if (i >= rp_frame_slot_count ||
        rp_frame_slots[i].start == NULL ||
        rp_frame_slots[i].end < rp_frame_slots[i].start)
    {
        return 0;
    }
    return (uint32_t)(rp_frame_slots[i].end - rp_frame_slots[i].start);
}

static uint32_t rp_count_free_frame_slots_locked(void)
{
    uint32_t free_count = 0;
    for (uint32_t i = 0; i < rp_frame_slot_count; i++)
    {
        if (rp_frame_slots[i].state == RP_FRAME_SLOT_FREE)
            free_count++;
    }
    return free_count;
}

/* Count truly usable FREE bytes, not nominal suites or RESERVED descriptors.
 * Conservative at pressure boundaries: RESERVED bytes require a later merge. */
static uint32_t rp_v112_free_bytes_locked(void)
{
    uint64_t bytes = 0u;
    for (uint32_t i = 0; i < rp_frame_slot_count; ++i)
        if (rp_frame_slots[i].state == RP_FRAME_SLOT_FREE &&
            rp_slot_region_size_locked(i) >= rp_frame_slot_stride)
            bytes += rp_slot_region_size_locked(i);
    return bytes > UINT32_MAX ? UINT32_MAX : (uint32_t)bytes;
}

static uint32_t rp_v112_free_bytes(void)
{
    if (rp_slot_lock()) return 0u;
    uint32_t bytes = rp_v112_free_bytes_locked();
    rp_slot_unlock();
    return bytes;
}

static uint32_t rp_count_reserved_frame_slots_locked(void)
{
    uint32_t reserved = 0;
    for (uint32_t i = 0; i < rp_frame_slot_count; i++)
    {
        if (rp_frame_slots[i].state == RP_FRAME_SLOT_RESERVED)
            reserved++;
    }
    return reserved;
}

static uint32_t rp_count_free_frame_slots(void)
{
    if (rp_slot_lock())
        return 0;
    uint32_t free_count = rp_count_free_frame_slots_locked();
    rp_slot_unlock();
    return free_count;
}

static uint32_t rp_count_reserved_frame_slots(void)
{
    if (rp_slot_lock())
        return 0;
    uint32_t reserved = rp_count_reserved_frame_slots_locked();
    rp_slot_unlock();
    return reserved;
}

static int rp_slot_index(const struct rp_frame_slot *slot)
{
    if (slot < &rp_frame_slots[0] || slot >= &rp_frame_slots[RP_FRAME_SLOT_MAX])
        return -1;
    return (int)(slot - &rp_frame_slots[0]);
}

static struct rp_frame_slot *rp_find_full_frame_slot_locked(uint32_t frame_number)
{
    for (uint32_t i = 0; i < rp_frame_slot_count; i++)
    {
        if (rp_frame_slots[i].state == RP_FRAME_SLOT_FULL &&
            rp_frame_slots[i].frame_number == frame_number)
        {
            return &rp_frame_slots[i];
        }
    }
    return NULL;
}

static struct rp_frame_slot *rp_v112_claim_slot_locked(uint32_t i)
{
    struct rp_frame_slot *slot = &rp_frame_slots[i];
    slot->state = RP_FRAME_SLOT_CAPTURING;
    slot->used = 0u;
    rp_frame_slot_cursor = (i + 1u) % rp_frame_slot_count;
    rp_v112_last_capture = i;
    return slot;
}

/* Capture sequentially through the metadata ring whenever possible.
 *
 * shrink_slot-style redistribution makes the metadata entry immediately after
 * a compressed frame physically adjacent and FREE.  Keeping the cursor on that
 * next entry is what turns variable-size compressed frames into large
 * zero-copy FIO groups. */
static struct rp_frame_slot *rp_get_free_frame_slot(void)
{
    if (rp_frame_slot_count == 0 || rp_slot_lock())
        return NULL;

    /* Prefer the byte immediately following the last *finished* frame,
     * even if a concurrent writer changed the metadata cursor. Validate
     * physical adjacency and FREE ownership under the same slot semaphore. */
    if (rp_v112_last_capture < rp_frame_slot_count)
    {
        uint32_t prev_i = rp_v112_last_capture;
        struct rp_frame_slot *prev = &rp_frame_slots[prev_i];
        if ((prev->state == RP_FRAME_SLOT_FULL ||
             prev->state == RP_FRAME_SLOT_WRITING) && prev->used &&
            prev_i + 1u < rp_frame_slot_count)
        {
            struct rp_frame_slot *next = &rp_frame_slots[prev_i + 1u];
            if (next->chunk_index == prev->chunk_index &&
                next->state == RP_FRAME_SLOT_FREE &&
                next->start == prev->start + prev->used &&
                rp_slot_region_size_locked(prev_i + 1u) >= rp_frame_slot_stride)
            {
                rp_v112_tail_hits++;
                struct rp_frame_slot *out = rp_v112_claim_slot_locked(prev_i + 1u);
                rp_slot_unlock();
                return out;
            }
        }
    }

    /* Fast path: keep rolling through the physical run created by the last
     * shrink, exactly like mlv_lite::choose_next_capture_slot(). */
    if (rp_frame_slot_cursor < rp_frame_slot_count &&
        rp_frame_slots[rp_frame_slot_cursor].state == RP_FRAME_SLOT_FREE &&
        rp_slot_region_size_locked(rp_frame_slot_cursor) >= rp_frame_slot_stride)
    {
        uint32_t i = rp_frame_slot_cursor;
        struct rp_frame_slot *out = rp_v112_claim_slot_locked(i);
        rp_slot_unlock();
        return out;
    }

    /* When the sequential run is blocked, start at the largest physically
     * contiguous FREE run.  A simple first-free scan needlessly fragments
     * compressed batches and was one of the differences from the older
     * stable v3C/mlv_lite allocator. */
    uint32_t best_start = RP_FRAME_SLOT_MAX;
    uint64_t best_bytes = 0u;
    uint32_t best_len = 0u;

    for (uint32_t c = 0; c < rp_frame_chunk_count; c++)
    {
        uint32_t first = rp_frame_chunks[c].first_slot;
        uint32_t count = rp_frame_chunks[c].slot_count;
        uint32_t run_start = first;
        uint32_t run_len = 0u;
        uint64_t run_bytes = 0u;
        char *prev_end = NULL;

        for (uint32_t n = 0; n < count; n++)
        {
            uint32_t i = first + n;
            if (i >= rp_frame_slot_count)
                break;

            struct rp_frame_slot *slot = &rp_frame_slots[i];
            int usable_free =
                slot->state == RP_FRAME_SLOT_FREE &&
                rp_slot_region_size_locked(i) >= rp_frame_slot_stride;

            if (usable_free)
            {
                if (run_len == 0u || slot->start != prev_end)
                {
                    run_start = i;
                    run_len = 1u;
                    run_bytes = 0u;
                }
                else
                {
                    run_len++;
                }
                run_bytes += rp_slot_region_size_locked(i);
                prev_end = slot->end;

                /* Prefer actual writable bytes, not metadata cardinality.
                 * Stable tie-break on descriptor count for equal sizes. */
                if (run_bytes > best_bytes ||
                    (run_bytes == best_bytes && run_len > best_len))
                {
                    best_bytes = run_bytes;
                    best_len = run_len;
                    best_start = run_start;
                }
            }
            else
            {
                run_len = 0u;
                run_bytes = 0u;
                prev_end = NULL;
            }
        }
    }

    if (best_start >= rp_frame_slot_count)
    {
        rp_slot_unlock();
        return NULL;
    }

    rp_v112_fallbacks++;
    struct rp_frame_slot *out = rp_v112_claim_slot_locked(best_start);
    rp_slot_unlock();
    return out;
}

/* Try to give the unused tail of a just-compressed slot to the following
 * FREE/RESERVED metadata entries, as mlv_lite::shrink_slot() does.
 *
 * Return 1 if the physical region was shrunk, 0 if it deliberately stayed at
 * its old size because the next region is busy, <0 on invalid input.
 *
 * The "deferred" case is safe: slot->used is still the exact on-disk VIDF
 * size, so the writer simply cannot coalesce across the RAM-only gap.  No
 * memory is lost and free_slot() later restores the full region. */
static int rp_shrink_frame_slot_locked(uint32_t i, uint32_t new_size)
{
    if (i >= rp_frame_slot_count ||
        rp_frame_slots[i].state != RP_FRAME_SLOT_CAPTURING ||
        new_size == 0u || (new_size & (RP_FRAME_SLOT_ALIGN - 1u)))
    {
        return -1;
    }

    uint32_t old_size = rp_slot_region_size_locked(i);
    if (new_size > old_size)
        return -2;
    if (new_size == old_size)
        return 1;

    uint32_t diff = old_size - new_size;
    char *old_end = rp_frame_slots[i].end;

    int linked =
        (i + 1u < rp_frame_slot_count) &&
        (rp_frame_slots[i + 1u].chunk_index == rp_frame_slots[i].chunk_index) &&
        (rp_frame_slots[i + 1u].state == RP_FRAME_SLOT_FREE ||
         rp_frame_slots[i + 1u].state == RP_FRAME_SLOT_RESERVED) &&
        (rp_frame_slots[i + 1u].start == old_end);

    if (!linked)
    {
        rp_slot_shrink_deferred++;
        return 0;
    }

    rp_frame_slots[i].end = rp_frame_slots[i].start + new_size;
    rp_frame_slots[i + 1u].start -= diff;

    /* Normalize as many following regions as metadata allows.  A FREE entry
     * may temporarily be larger than max stride only if there is no following
     * FREE/RESERVED descriptor able to receive its excess; that is still a
     * valid capture region and will be normalized on its next shrink/free. */
    uint32_t j = i + 1u;
    while (j < rp_frame_slot_count &&
           rp_frame_slots[j].chunk_index == rp_frame_slots[i].chunk_index)
    {
        uint32_t sz = rp_slot_region_size_locked(j);
        if (sz < rp_frame_slot_stride)
        {
            rp_frame_slots[j].state = RP_FRAME_SLOT_RESERVED;
            break;
        }

        rp_frame_slots[j].state = RP_FRAME_SLOT_FREE;
        if (sz == rp_frame_slot_stride)
            break;

        uint32_t excess = sz - rp_frame_slot_stride;
        char *old_j_end = rp_frame_slots[j].end;
        int next_linked =
            (j + 1u < rp_frame_slot_count) &&
            (rp_frame_slots[j + 1u].chunk_index == rp_frame_slots[j].chunk_index) &&
            (rp_frame_slots[j + 1u].state == RP_FRAME_SLOT_FREE ||
             rp_frame_slots[j + 1u].state == RP_FRAME_SLOT_RESERVED) &&
            (rp_frame_slots[j + 1u].start == old_j_end);

        if (!next_linked)
            break;

        rp_frame_slots[j].end = rp_frame_slots[j].start + rp_frame_slot_stride;
        rp_frame_slots[j + 1u].start -= excess;
        j++;
    }

    rp_slot_shrink_ok++;
    uint32_t reserved = rp_count_reserved_frame_slots_locked();
    if (reserved > rp_slot_reserved_peak)
        rp_slot_reserved_peak = reserved;
    return 1;
}

static int rp_shrink_frame_slot(struct rp_frame_slot *slot, uint32_t new_size)
{
    int i = rp_slot_index(slot);
    if (i < 0 || rp_slot_lock())
        return -1;
    int rc = rp_shrink_frame_slot_locked((uint32_t)i, new_size);
    rp_slot_unlock();
    return rc;
}

/* mlv_lite-style free_slot().
 *
 * Merge the just-freed variable region with adjacent FREE/RESERVED regions
 * from the same Canon memSuite chunk, then repartition that address range into
 * full max-size FREE slots followed by zero/partial RESERVED descriptors. */
static void rp_free_frame_slot_locked(uint32_t slot_index)
{
    if (slot_index >= rp_frame_slot_count)
        return;

    struct rp_frame_slot *slot = &rp_frame_slots[slot_index];
    slot->used = 0;
    slot->state = RP_FRAME_SLOT_RESERVED;

    uint32_t start = slot_index;
    while (start > 0u)
    {
        struct rp_frame_slot *prev = &rp_frame_slots[start - 1u];
        struct rp_frame_slot *cur = &rp_frame_slots[start];
        if (prev->chunk_index != cur->chunk_index ||
            (prev->state != RP_FRAME_SLOT_FREE &&
             prev->state != RP_FRAME_SLOT_RESERVED) ||
            cur->start != prev->end)
        {
            break;
        }
        start--;
    }

    uint32_t end = slot_index;
    while (end + 1u < rp_frame_slot_count)
    {
        struct rp_frame_slot *cur = &rp_frame_slots[end];
        struct rp_frame_slot *next = &rp_frame_slots[end + 1u];
        if (next->chunk_index != cur->chunk_index ||
            (next->state != RP_FRAME_SLOT_FREE &&
             next->state != RP_FRAME_SLOT_RESERVED) ||
            next->start != cur->end)
        {
            break;
        }
        end++;
    }

    char *start_ptr = rp_frame_slots[start].start;
    char *end_ptr = rp_frame_slots[end].end;
    char *ptr = start_ptr;

    for (uint32_t i = start; i <= end; i++)
    {
        rp_frame_slots[i].start = ptr;
        rp_frame_slots[i].used = 0;

        uint32_t remaining = (uint32_t)(end_ptr - ptr);
        if (remaining >= rp_frame_slot_stride)
        {
            rp_frame_slots[i].end = ptr + rp_frame_slot_stride;
            rp_frame_slots[i].state = RP_FRAME_SLOT_FREE;
            ptr += rp_frame_slot_stride;
        }
        else
        {
            /* First remainder descriptor owns all residual bytes.  Any later
             * reserved descriptors are zero-sized at the physical end. */
            rp_frame_slots[i].end = end_ptr;
            rp_frame_slots[i].state = RP_FRAME_SLOT_RESERVED;
            ptr = end_ptr;
        }
    }

    rp_slot_free_rebuilds++;
    uint32_t reserved = rp_count_reserved_frame_slots_locked();
    if (reserved > rp_slot_reserved_peak)
        rp_slot_reserved_peak = reserved;
}

static void rp_abandon_frame_slot(struct rp_frame_slot *slot)
{
    int i = rp_slot_index(slot);
    if (i < 0 || rp_slot_lock())
        return;
    rp_free_frame_slot_locked((uint32_t)i);
    rp_slot_unlock();
}

static void rp_clear_frame_slot_metadata(void)
{
    for (uint32_t c = 0; c < RP_FRAME_CHUNK_MAX; c++)
    {
        rp_frame_chunks[c]._buf = NULL;
        rp_frame_chunks[c].start = NULL;
        rp_frame_chunks[c].bytes = 0;
        rp_frame_chunks[c].first_slot = 0;
        rp_frame_chunks[c].slot_count = 0;
    }

    for (uint32_t i = 0; i < RP_FRAME_SLOT_MAX; i++)
    {
        rp_frame_slots[i].start = NULL;
        rp_frame_slots[i].end = NULL;
        rp_frame_slots[i].used = 0;
        rp_frame_slots[i].frame_number = 0;
        rp_frame_slots[i].chunk_index = 0;
        rp_frame_slots[i].chunk_pos = 0;
        rp_frame_slots[i].state = RP_FRAME_SLOT_UNUSED;
    }

    rp_crx_ref_slot = NULL;
    rp_crx_ref_width = 0;
    rp_crx_ref_height = 0;
    rp_crx_ref_bits = 0;
    rp_crx_ref_row_bytes = 0;
    rp_crx_ref_payload_bytes = 0;
    rp_crx_ref_frame_number = 0;
    rp_crx_ref_copy_us = 0;

    rp_frame_slot_count = 0;
    rp_frame_chunk_count = 0;
    rp_frame_slot_cursor = 0;
    rp_v112_last_capture = RP_FRAME_SLOT_MAX;
    rp_v112_tail_hits = rp_v112_fallbacks = 0u;
    rp_frame_slot_stride = 0;
    rp_frame_suite_bytes = 0;
    rp_frame_suite_chunks = 0;
    rp_frame_output_bytes = 0;
    rp_frame_waste_bytes = 0;
    rp_slot_shrink_ok = 0;
    rp_slot_shrink_deferred = 0;
    rp_slot_free_rebuilds = 0;
    rp_slot_reserved_peak = 0;
}

static void rp_free_frame_slots(void)
{
    /* Do not return any provider while a writer may still dereference it.
     * On uncertainty preserve memory and reject all new sessions this boot. */
    if ((rp_frame_suite != NULL || rp_v110_mem3[0] != NULL ||
         rp_v110_mem3[1] != NULL || rp_v110_ssraw_addr != 0u) &&
        (rp_writer_jobs_queued != rp_writer_jobs_done ||
         rp_header_write_pending || rp_writer_storage_lockout ||
         rp_writer_io_fail || rp_writer_post_fail))
    {
        rp_v110_owner_quarantine = 1;
        rp_writer_storage_lockout = 1;
        RP_START_TRACE("SS110 QUARANTINE pending=%d header=%d io=%d post=%d",
                       (int)(rp_writer_jobs_queued-rp_writer_jobs_done),
                       (int)rp_header_write_pending,
                       (int)rp_writer_io_fail,(int)rp_writer_post_fail);
        return;
    }
#if RP_V110_SSRAM_OUTPUT
    if (rp_v110_ssraw_addr)
    {
        int rc = rp_ssraw_v110_release();
        RP_START_TRACE("SS110 SRM release rc=%d state=%d",rc,(int)rp_ssraw_v110_state());
        if (rc)
        {
            /* Pending native free is not cancellation; the CORE is resident.
             * Preserve other arenas too, and forbid later recording. */
            rp_v110_owner_quarantine = 1;
            rp_writer_storage_lockout = 1;
            return;
        }
        rp_v110_ssraw_addr = rp_v110_ssraw_bytes = 0;
    }
#endif
    for (int i = 1; i >= 0; --i)
    {
        if (rp_v110_mem3[i] != NULL)
        {
            shoot_free_suite(rp_v110_mem3[i]);
            rp_v110_mem3[i] = NULL;
        }
    }

    /* Slots are only views into one Canon shoot-memory suite.  Never free
     * individual dynamic regions; release the suite exactly once. */
    if (rp_frame_suite != NULL)
    {
        shoot_free_suite(rp_frame_suite);
        rp_frame_suite = NULL;
    }

    rp_clear_frame_slot_metadata();
}

static uint32_t rp_calc_frame_slot_stride(uint32_t payload_size)
{
    /* Maximum physical region reserved for one in-flight frame.  Actual CRX
     * VIDF blocks are shrunk to sector-aligned blockSize after encode. */
    uint32_t minimum = EDMAC_ALIGNMENT + payload_size;
    return ALIGN_UP(minimum, RP_FRAME_SLOT_ALIGN);
}

/* Add one Canon memSuite chunk using the same structural idea as mlv_lite:
 * max-size FREE entries first, followed by equally many RESERVED descriptors.
 * Reserved descriptors are metadata capacity that allows compressed frames to
 * hand their unused tails forward without allocating/copying another buffer. */
static int rp_add_frame_region(uintptr_t raw_ptr, uint32_t raw_size,
                               uint32_t source_index, int is_ssraw)
{
    if (raw_size == 0u ||
        rp_frame_chunk_count >= RP_FRAME_CHUNK_MAX ||
        rp_frame_slot_count >= RP_FRAME_SLOT_MAX)
    {
        return 0;
    }

    uintptr_t aligned = ALIGN_UP(raw_ptr, RP_FRAME_SLOT_ALIGN);
    uint32_t align_loss = (uint32_t)(aligned - raw_ptr);

    if (raw_size <= align_loss)
    {
        rp_frame_waste_bytes += raw_size;
        return 0;
    }

    /* Exclude intersecting virtual ranges. This is not a proof that two bus
     * addresses cannot alias downstream of MMU, but catches obvious overlap. */
    if (raw_ptr > 0xffffffffu || raw_size > 0xffffffffu-(uint32_t)raw_ptr)
        return -3;
    if (!is_ssraw && (uint32_t)raw_ptr < 0xA77B0000u &&
        (uint32_t)raw_ptr + raw_size > 0xA0000000u)
        return -4;
    uintptr_t mapped = is_ssraw ? aligned :
                       (uintptr_t)UNCACHEABLE((void *)aligned);
    for (uint32_t c=0; c<rp_frame_chunk_count; ++c)
    {
        uintptr_t old=(uintptr_t)rp_frame_chunks[c].start;
        if (mapped < old+rp_frame_chunks[c].bytes &&
            old < mapped+(raw_size-align_loss))
            return -5;
    }
    uint32_t raw_usable = raw_size - align_loss;
    /* Dynamic RESERVED descriptors may start life at chunk_end and later grow
     * backwards into FREE slots.  Therefore BOTH ends of the represented
     * chunk must be sector-aligned; otherwise a zero-size RESERVED entry at an
     * unaligned physical end could later become an unaligned DMA/FIO slot. */
    uint32_t usable = raw_usable & ~(RP_FRAME_SLOT_ALIGN - 1u);
    uint32_t tail_align_loss = raw_usable - usable;
    uint32_t possible_full = usable / rp_frame_slot_stride;
    uint32_t metadata_left = RP_FRAME_SLOT_MAX - rp_frame_slot_count;
    uint32_t reserve_per_full =
        (rp_record_codec == RAW_VID_CODEC_CRAW) ? RP_CRAW_RESERVED_PER_FULL :
        (rp_record_codec == RAW_VID_CODEC_CRX)  ? RP_CRX_RESERVED_PER_FULL :
                                                  RP_RAW_RESERVED_PER_FULL;
    uint32_t metadata_per_full = 1u + reserve_per_full;
    uint32_t use_full = MIN(possible_full, metadata_left / metadata_per_full);

    if (use_full == 0u)
    {
        RP_START_TRACE("S36 DYNSLOT chunk=%d skip bytes=%dKB stride=%d meta=%d",
                       (int)source_index, (int)(usable / 1024u),
                       (int)rp_frame_slot_stride, (int)metadata_left);
        rp_frame_waste_bytes += raw_size;
        return 0;
    }

    uint32_t wanted_reserve = use_full * reserve_per_full;
    uint32_t reserve_count = MIN(wanted_reserve, metadata_left - use_full);
    uint32_t chunk_index = rp_frame_chunk_count;
    uint32_t first_slot = rp_frame_slot_count;
    struct rp_frame_chunk *chunk = &rp_frame_chunks[chunk_index];
    chunk->_buf = NULL;
    /* RPSS v105: 0xA0... bank already maps Normal non-cacheable in L1.
     * UNCACHEABLE() may synthesize a non-equivalent VA; preserve native A. */
    chunk->start = is_ssraw ? (char *)aligned :
                   (char *)UNCACHEABLE((void *)aligned);
    chunk->bytes = usable;
    chunk->first_slot = first_slot;
    chunk->slot_count = use_full + reserve_count;

    char *ptr = chunk->start;
    for (uint32_t n = 0; n < use_full; n++)
    {
        struct rp_frame_slot *slot = &rp_frame_slots[rp_frame_slot_count++];
        slot->start = ptr;
        slot->end = ptr + rp_frame_slot_stride;
        slot->used = 0;
        slot->frame_number = 0;
        slot->chunk_index = (uint16_t)chunk_index;
        slot->chunk_pos = (uint16_t)n;
        slot->state = RP_FRAME_SLOT_FREE;
        ptr += rp_frame_slot_stride;
    }

    char *chunk_end = chunk->start + usable;
    for (uint32_t n = 0; n < reserve_count; n++)
    {
        struct rp_frame_slot *slot = &rp_frame_slots[rp_frame_slot_count++];
        slot->start = ptr;
        slot->end = (n == 0u) ? chunk_end : ptr;
        slot->used = 0;
        slot->frame_number = 0;
        slot->chunk_index = (uint16_t)chunk_index;
        slot->chunk_pos = (uint16_t)(use_full + n);
        slot->state =
            (n == 0u && (uint32_t)(slot->end - slot->start) >= rp_frame_slot_stride)
            ? RP_FRAME_SLOT_FREE
            : RP_FRAME_SLOT_RESERVED;
        ptr = slot->end;
    }

    rp_frame_chunk_count++;
    rp_frame_output_bytes += usable;
    rp_frame_waste_bytes += align_loss + tail_align_loss;

    RP_START_TRACE("S36 DYNSLOT chunk=%d run=%d base=%x usable=%dKB full=%d reserve=%d rpf=%d tail=%dKB alignWaste=%d",
                   (int)source_index, (int)chunk_index,
                   (uint32_t)chunk->start, (int)(usable / 1024u),
                   (int)use_full, (int)reserve_count, (int)reserve_per_full,
                   (int)((usable - use_full * rp_frame_slot_stride) / 1024u),
                   (int)(align_loss + tail_align_loss));
    return (int)use_full;
}

static int rp_add_shoot_suite_chunk(struct memChunk *mem_chunk,
                                    uint32_t suite_chunk_index)
{
    if (mem_chunk == NULL) return 0;
    return rp_add_frame_region((uintptr_t)GetMemoryAddressOfMemoryChunk(mem_chunk),
                               (uint32_t)GetSizeOfMemoryChunk(mem_chunk),
                               suite_chunk_index,0);
}

static int rp_alloc_frame_slots(uint32_t payload_size)
{
    if (rp_v110_owner_quarantine || rp_writer_storage_lockout) return -99;
    rp_free_frame_slots();
    if (rp_v110_owner_quarantine) return -99;
    rp_frame_slot_stride = rp_calc_frame_slot_stride(payload_size);

#if RP_V110_MEM3_2X32
    /* Explicit 2 x 32 MiB requests go through Canon RscMgr (Mem3).
     * Neither request implies one *physically* contiguous 32-MiB chunk.
     * The existing per-chunk slot metadata honors arbitrary returned pieces. */
    for (uint32_t p=0; p<2u; ++p)
    {
        rp_v110_mem3[p] = shoot_malloc_suite(RP_V110_MEM3_BLOCK);
        if (!rp_v110_mem3[p])
        {
            RP_START_TRACE("SS110 MEM3 allocation FAIL n=%d bytes=%d",(int)p,(int)RP_V110_MEM3_BLOCK);
            rp_free_frame_slots();
            return -20-(int)p;
        }
        uint32_t nbytes=(uint32_t)GetSizeOfMemorySuite(rp_v110_mem3[p]);
        if (nbytes<RP_V110_MEM3_BLOCK)
        {
            RP_START_TRACE("SS110 MEM3 short n=%d bytes=%d",(int)p,(int)nbytes);
            rp_free_frame_slots();
            return -23;
        }
        rp_frame_suite_bytes+=nbytes;
        uint32_t chunks=(uint32_t)GetNumberOfChunks(rp_v110_mem3[p]);
        rp_frame_suite_chunks+=chunks;
        struct memChunk *m=GetFirstChunkFromSuite(rp_v110_mem3[p]);
        uint32_t counted=0;
        while (m)
        {
            if (rp_frame_chunk_count>=RP_FRAME_CHUNK_MAX ||
                rp_frame_slot_count+2u>=RP_FRAME_SLOT_MAX) break;
            int added=rp_add_shoot_suite_chunk(m,(p<<16)|counted);
            if (added<0)
            {
                RP_START_TRACE("SS110 MEM3 invalid n=%d chunk=%d rc=%d",
                               (int)p,(int)counted,added);
                rp_free_frame_slots();
                return -27;
            }
            m=GetNextMemoryChunk(rp_v110_mem3[p],m);
            counted++;
        }
        RP_START_TRACE("SS110 MEM3 n=%d bytes=%d chunks=%d seen=%d",
                       (int)p,(int)nbytes,(int)chunks,(int)counted);
        if (counted!=chunks)
        {
            RP_START_TRACE("SS110 MEM3 truncated n=%d",(int)p);
            rp_free_frame_slots();
            return -24;
        }
    }
#if RP_V110_SSRAM_OUTPUT
    {
        uint32_t base=0,bytes=0;
        int rc=rp_ssraw_v110_acquire(&base,&bytes);
        RP_START_TRACE("SS110 SRM acquire rc=%d addr=%x bytes=%x",rc,base,bytes);
        if (rc || base!=0xA0000000u || bytes!=0x077B0000u)
        {
            /* Native job may remain pending in resident CORE.  Do not use
             * any unexpected bank or retry before cold boot. */
            rp_v110_owner_quarantine=1;
            rp_writer_storage_lockout=1;
            return -25;
        }
        rp_v110_ssraw_addr=base;
        rp_v110_ssraw_bytes=bytes;
        int added=rp_add_frame_region((uintptr_t)base,bytes,0x38u,1);
        RP_START_TRACE("SS110 SRM REGION added=%d total=%d bytes=%d",added,
                       (int)rp_frame_slot_count,(int)rp_frame_output_bytes);
        if (added<=0)
        {
            rp_free_frame_slots();
            return -26;
        }
    }
#endif
#else
    rp_frame_suite = shoot_malloc_suite(0);
    if (rp_frame_suite == NULL)
    {
        RP_START_TRACE("S36 DYNSLOT suite alloc FAIL payload=%d stride=%d source=%dKB",
                       (int)payload_size, (int)rp_frame_slot_stride,
                       (int)(raw_vid_rp_capture_buffer_bytes() / 1024u));
        rp_clear_frame_slot_metadata();
        return -1;
    }

    rp_frame_suite_bytes = (uint32_t)GetSizeOfMemorySuite(rp_frame_suite);
    rp_frame_suite_chunks = (uint32_t)GetNumberOfChunks(rp_frame_suite);

    struct memChunk *mem_chunk = GetFirstChunkFromSuite(rp_frame_suite);
    uint32_t suite_chunk_index = 0;
    while (mem_chunk != NULL)
    {
        if (rp_frame_chunk_count >= RP_FRAME_CHUNK_MAX ||
            rp_frame_slot_count + 2u >= RP_FRAME_SLOT_MAX)
        {
            RP_START_TRACE("S36 DYNSLOT truncate chunk=%d runs=%d entries=%d",
                           (int)suite_chunk_index, (int)rp_frame_chunk_count,
                           (int)rp_frame_slot_count);
            break;
        }

        rp_add_shoot_suite_chunk(mem_chunk, suite_chunk_index);
        mem_chunk = GetNextMemoryChunk(rp_frame_suite, mem_chunk);
        suite_chunk_index++;
    }
#endif

    uint32_t free_count = rp_count_free_frame_slots();
    if (free_count < RP_FRAME_SLOT_MIN)
    {
        RP_START_TRACE("S36 DYNSLOT alloc FAIL free=%d entries=%d suite=%dKB stride=%d",
                       (int)free_count, (int)rp_frame_slot_count,
                       (int)(rp_frame_suite_bytes / 1024u),
                       (int)rp_frame_slot_stride);
        rp_free_frame_slots();
        return -2;
    }

    rp_slot_min_free = free_count;
    uint32_t reserved = rp_count_reserved_frame_slots();
    rp_slot_reserved_peak = reserved;

    RP_START_TRACE("S36 DYNSLOT alloc PASS free=%d reserved=%d entries=%d runs=%d stride=%d output=%dKB suite=%dKB waste=%dKB",
                   (int)free_count, (int)reserved, (int)rp_frame_slot_count,
                   (int)rp_frame_chunk_count, (int)rp_frame_slot_stride,
                   (int)(rp_frame_output_bytes / 1024u),
                   (int)(rp_frame_suite_bytes / 1024u),
                   (int)(rp_frame_waste_bytes / 1024u));
    return 0;
}

static void alloc_crop_bufs(struct crop_buf crop_bufs[], int resolution_mode, int bpp)
{
    if (crop_bufs[0]._buf != NULL
        || crop_bufs[1]._buf != NULL)
    {
        SEND_LOG_DATA_STR("ERROR: non-null _buf in alloc_crop_bufs()\n");
        return;
    }
    
    /* The original RP MVP used two fixed 8 MiB accumulation buffers.  That is
     * enough through 2560x1440, but not for a complete 3728x2052 RAW frame
     * (13.4 MiB at 14-bit).  Size the floor from the frozen output preset so
     * every allocated buffer can hold at least one VIDF payload.
     *
     * Full uses the largest runtime-proven RP channel as the pre-RAW-LV hint;
     * after raw_lv_request() the exact current channel is validated separately. */
    uint32_t hint_w = 1280;
    uint32_t hint_h = 720;
    if (resolution_mode == RAW_VID_RES_1080P)
    {
        hint_w = 1920; hint_h = 1080;
    }
    else if (resolution_mode == RAW_VID_RES_1440P)
    {
        hint_w = 2560; hint_h = 1440;
    }
    else if (resolution_mode == RAW_VID_RES_FULL ||
             resolution_mode == RAW_VID_RES_W3_TRANSPORT ||
             resolution_mode == RAW_VID_RES_W3_WIDE_720)
    {
        hint_w = 3728; hint_h = 2052;
    }

    uint32_t min_usable_size = MIN_CROP_BUF_SIZE;
    uint32_t initial_usable_size = INITIAL_CROP_BUF_SIZE;
    if (is_camera("RP", "1.6.0"))
    {
        uint32_t payload_hint = hint_w * hint_h * (uint32_t)bpp / 8u;
        uint32_t required = ALIGN_UP(payload_hint + CROP_BUF_ALIGN, CROP_BUF_STEP_SIZE);
        min_usable_size = MAX((uint32_t)MIN_CROP_BUF_SIZE, required);
        initial_usable_size = MAX(8u * 1024u * 1024u, min_usable_size);
    }

    uint32_t usable_size = initial_usable_size;
    uint32_t buf_size = usable_size + CROP_BUF_ALIGN;
    char *buf0 = NULL;
    char *buf1 = NULL;
    while (usable_size >= min_usable_size)
    {
        buf_size = usable_size + CROP_BUF_ALIGN;
        buf0 = fio_malloc(buf_size);
        if (buf0 != NULL)
        {
            buf1 = fio_malloc(buf_size);
            if (buf1 != NULL)
            {
                break;
            }
            else
            {
                fio_free(buf0);
                buf0 = NULL;
            }
        }

        if (usable_size < min_usable_size + CROP_BUF_STEP_SIZE)
            break;
        usable_size -= CROP_BUF_STEP_SIZE;
    }

    if (buf0 == NULL || buf1 == NULL)
    {
        SEND_LOG_DATA_STR("Couldn't find good crop bufs\n");
        // These should both already be null, but it's a big memory leak
        // if we get it wrong!
        if (buf0 != NULL)
        {
            SEND_LOG_DATA_STR("ERROR: buf0 unexpectedly NULL\n");
            fio_free(buf0);
            buf0 = NULL;
        }
        if (buf1 != NULL)
        {
            SEND_LOG_DATA_STR("ERROR: buf1 unexpectedly NULL\n");
            fio_free(buf1);
            buf1 = NULL;
        }
        return;
    }

    // Here we have two large enough bufs.

    crop_bufs[0]._buf = buf0;
    crop_bufs[1]._buf = buf1;

    if ((uint32_t)buf0 % CROP_BUF_ALIGN != 0)
    {
        crop_bufs[0].start = buf0 + (CROP_BUF_ALIGN - (uint32_t)buf0 % CROP_BUF_ALIGN);
    }
    else
    {
        crop_bufs[0].start = buf0;
    }
    if ((uint32_t)buf1 % CROP_BUF_ALIGN != 0)
    {
        crop_bufs[1].start = buf1 + (CROP_BUF_ALIGN - (uint32_t)buf1 % CROP_BUF_ALIGN);
    }
    else
    {
        crop_bufs[1].start = buf1;
    }
    // Do we want to include the unaligned end portion in the above sizes?
    // I'm assuming not, it's a small amount, and would rarely contain data
    // before we write out.

    /*
     * Legacy raw_vidx accumulated into DMA/uncached aliases.  RP stage 17
     * deliberately uses cacheable CPU destinations: the source remains the
     * DMA/uncached full-frame ring, while FIO_WriteFile() already performs
     * cache clean and converts cacheable sources to UNCACHEABLE internally.
     */
    if (is_camera("RP", "1.6.0") && RP_START_PROBE_STAGE == 17)
    {
        crop_bufs[0].start = CACHEABLE(crop_bufs[0].start);
        crop_bufs[1].start = CACHEABLE(crop_bufs[1].start);
    }
    else
    {
        crop_bufs[0].start = UNCACHEABLE(crop_bufs[0].start);
        crop_bufs[1].start = UNCACHEABLE(crop_bufs[1].start);
    }

    /* buf_size includes CROP_BUF_ALIGN bytes of alignment padding.  Starting
     * from the aligned alias and adding the entire allocation size can run up
     * to CROP_BUF_ALIGN-1 bytes past the allocation.  Keep a guaranteed-safe
     * aligned payload size instead. */
    /* usable_size was tracked separately from the allocation padding above. */
    crop_bufs[0].cur = crop_bufs[0].start;
    crop_bufs[0].end = crop_bufs[0].start + usable_size;
    crop_bufs[1].cur = crop_bufs[1].start;
    crop_bufs[1].end = crop_bufs[1].start + usable_size;
    crop_bufs[0].writing = 0;
    crop_bufs[1].writing = 0;
}

static void free_crop_bufs(struct crop_buf crop_bufs[])
{
    /* Do not leave dirty cache lines referring to memory returned to Canon.
     * sync_caches() is a core-internal symbol and is not exported to modules on
     * RP.  dcache_clean() is an RP stub and is sufficient here because these
     * buffers contain data, not executable code. */
    if (is_camera("RP", "1.6.0") && RP_START_PROBE_STAGE == 17)
    {
        for (int i = 0; i < 2; i++)
        {
            if (crop_bufs[i].start != NULL && crop_bufs[i].end > crop_bufs[i].start)
            {
                dcache_clean((uint32_t)CACHEABLE(crop_bufs[i].start),
                             (uint32_t)(crop_bufs[i].end - crop_bufs[i].start));
            }
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (crop_bufs[i]._buf != NULL)
            fio_free(crop_bufs[i]._buf);

        crop_bufs[i]._buf = NULL;
        crop_bufs[i].start = NULL;
        crop_bufs[i].cur = NULL;
        crop_bufs[i].end = NULL;
        crop_bufs[i].writing = 0;
    }
}

static void write_buf_to_disk(struct crop_buf *crop_buf)
{
    #ifdef RAW_VID_LOG
    uint32_t buf_size = 100;
    char log_buf[buf_size];
    #endif

    if (crop_buf == NULL)
    {
        SEND_LOG_DATA_STR("ERROR: buf struct for disk write was NULL\n");
        return;
    }

    if (crop_buf->_buf == NULL
        || crop_buf->start == NULL
        || crop_buf->cur == NULL
        || crop_buf->end == NULL)
    {
        SEND_LOG_DATA_STR("ERROR: some buf member for disk write was NULL\n");
        crop_buf->writing = 0;
        return;
    }

    /* COMMAND_STOP waits for writing to clear before closing raw_vid_fp. */
    if (raw_vid_fp == NULL)
    {
        SEND_LOG_DATA_STR("ERROR: raw_vid_fp was NULL before write\n");
        crop_buf->writing = 0;
        return;
    }

    #ifdef RAW_VID_LOG
    uint32_t start_time = get_ms_clock();
    #endif

    /* S35: avoid launching the heavy storage transaction in the narrow
     * pre-VD window implicated by the 4K Err70 traces. */
    rp_wait_fio_safe_phase();

    uint32_t fio_bytes = (uint32_t)(crop_buf->cur - crop_buf->start);
    uint64_t fio_t0 = 0;
    if (is_camera("RP", "1.6.0") && RP_START_PROBE_STAGE == 36)
    {
        fio_t0 = (uint64_t)get_us_clock();
        RP_START_TRACE("S36 FIO start bytes=%d prio=%x",
                       (int)fio_bytes, WORKER_PRIO);
    }

    FIO_WriteFile(raw_vid_fp, crop_buf->start, fio_bytes);

    if (is_camera("RP", "1.6.0") && RP_START_PROBE_STAGE == 36)
    {
        uint32_t fio_us = (uint32_t)((uint64_t)get_us_clock() - fio_t0);
        uint32_t rate_x10 = fio_us
            ? (uint32_t)(((uint64_t)fio_bytes * 10u) / fio_us)
            : 0;
        if (fio_us > rp_writer_max_us)
            rp_writer_max_us = fio_us;
        RP_START_TRACE("S36 FIO done bytes=%d us=%d rate=%d.%dMBs",
                       (int)fio_bytes, (int)fio_us,
                       (int)(rate_x10 / 10u), (int)(rate_x10 % 10u));
    }

    #ifdef RAW_VID_LOG
    uint32_t end_time = get_ms_clock();
    uint32_t data_rate_x100 = (uint32_t)((float)fio_bytes / 1024 / 1024
                              * 1000 * 100 / (end_time - start_time));
    snprintf(log_buf, buf_size, "Buf flush data rate, MB/s: %d.%02d\n",
             data_rate_x100 / 100,
             data_rate_x100 % 100);
    send_log_data_str(log_buf);
    #endif

    /* Publish FREE last.  Producer uses cur==start as the reusable test and
     * writing as stop/drain ownership, so reset data position before ownership. */
    crop_buf->cur = crop_buf->start;
    crop_buf->writing = 0;
}

static int rp_queue_crx_header_write(char *ptr, uint32_t size)
{
    if (rp_write_q == NULL || ptr == NULL || size == 0u)
        return -1;
    if (rp_header_write_pending || rp_header_write_done)
        return -2;

    rp_header_write_ptr = ptr;
    rp_header_write_size = size;
    rp_header_write_failed = 0;
    rp_header_write_pending = 1;

    int rc = msg_queue_post(
        rp_write_q, (uint32_t)(uintptr_t)RP_WRITE_HEADER_TOKEN);
    if (rc)
    {
        rp_header_write_pending = 0;
        rp_header_write_ptr = NULL;
        rp_header_write_size = 0;
        return rc;
    }

    RP_START_TRACE("S36 REV2 HEADER queued bytes=%d", (int)size);
    return 0;
}

static int rp_queue_frame_slot_write(struct rp_frame_slot *slot)
{
    if (!is_camera("RP", "1.6.0") || RP_START_PROBE_STAGE != 36)
        return -100;
    if (rp_write_q == NULL || slot == NULL)
        return -1;

    if (rp_slot_lock())
        return -2;

    if (slot->state != RP_FRAME_SLOT_CAPTURING || slot->used == 0u)
    {
        rp_slot_unlock();
        return -3;
    }

    slot->state = RP_FRAME_SLOT_FULL;
    rp_writer_jobs_queued++;
    /* Rolling ingress in decimal MB/s x 100, measured on serialized VIDFs,
     * never on uncompressed source bytes. No disk work in producer context. */
    uint32_t now_us = get_us_clock();
    if (!rp_ingress_window_t0) rp_ingress_window_t0 = now_us;
    rp_ingress_window_bytes += slot->used;
    if (!rp_writer_recent_frame_bytes)
        rp_writer_recent_frame_bytes = slot->used;
    else
        rp_writer_recent_frame_bytes =
            (uint32_t)(((uint64_t)rp_writer_recent_frame_bytes * 7u +
                        slot->used) / 8u);
    uint32_t window_us = now_us - rp_ingress_window_t0;
    if (window_us >= 500000u)
    {
        rp_writer_ingress_rate_x100 =
            (uint32_t)((rp_ingress_window_bytes * 100u) / window_us);
        rp_ingress_window_bytes = 0;
        rp_ingress_window_t0 = now_us;
    }
    rp_slot_unlock();

    int rc = msg_queue_post(rp_write_q, (uint32_t)(uintptr_t)slot);
    if (rc)
    {
        if (!rp_slot_lock())
        {
            rp_writer_jobs_queued--;
            slot->state = RP_FRAME_SLOT_CAPTURING;
            rp_slot_unlock();
        }
        rp_writer_post_fail++;
        RP_START_TRACE("S36 SLOT enqueue FAIL rc=%d frame=%d bytes=%d",
                       rc, (int)slot->frame_number, (int)slot->used);
        return rc;
    }

    uint32_t qdepth = 0;
    msg_queue_count(rp_write_q, &qdepth);
    uint32_t free_count = rp_count_free_frame_slots();
    if (free_count < rp_slot_min_free)
        rp_slot_min_free = free_count;

    RP_START_TRACE("S36 SLOT enqueue job=%d q=%d frame=%d bytes=%d free=%d reserved=%d",
                   (int)rp_writer_jobs_queued, (int)qdepth,
                   (int)slot->frame_number, (int)slot->used,
                   (int)free_count, (int)rp_count_reserved_frame_slots());
    return 0;
}

static uint32_t rp_batch_bytes(struct rp_frame_slot **batch,
                               uint32_t count)
{
    uint32_t total = 0;
    for (uint32_t i = 0; i < count; i++)
    {
        if (batch[i] == NULL || total > 0xFFFFFFFFu - batch[i]->used)
            return 0;
        total += batch[i]->used;
    }
    return total;
}

static struct rp_v112_decision rp_v112_writer_pressure(uint32_t free_slots)
{
    struct rp_v112_pressure in;
    struct rp_v112_decision out;
    in.free_bytes = rp_v112_free_bytes();
    in.free_slots = free_slots;
    in.ingress_x100 = rp_writer_ingress_rate_x100;
    in.drain_x100 = rp_writer_drain_rate_x100;
    in.writer_x100 = rp_writer_rate_x100;
    in.fps_x1000 = session.fps > 0 ? (uint32_t)session.fps : 0u;
    in.draining = rp_writer_drain_requested;
    rp_v112_pressure_decide(&in, &out);
    return out;
}

static uint32_t rp_collect_contiguous_full_slots(struct rp_frame_slot *first,
                                                  struct rp_frame_slot **batch,
                                                  uint32_t max_slots)
{
    if (first == NULL || max_slots == 0u || rp_slot_lock())
        return 0;

    if (first->state != RP_FRAME_SLOT_FULL || first->used == 0u)
    {
        rp_slot_unlock();
        return 0;
    }

    if (first->used > RP_FIO_MAX_BATCH_BYTES)
    {
        rp_slot_unlock();
        return 0;
    }
    batch[0] = first;
    uint32_t count = 1u;
    uint32_t total_bytes = first->used;
    struct rp_frame_slot *prev = first;

    while (count < max_slots)
    {
        struct rp_frame_slot *next =
            rp_find_full_frame_slot_locked(first->frame_number + count);
        if (next == NULL || next->used == 0u)
            break;

        /* Variable-size compressed VIDFs may be grouped as long as the actual
         * used prefixes are physically adjacent. */
        if (next->chunk_index != prev->chunk_index ||
            next->start != prev->start + prev->used)
            break;
        if (next->used > RP_FIO_MAX_BATCH_BYTES ||
            total_bytes > RP_FIO_MAX_BATCH_BYTES - next->used)
            break;

        batch[count++] = next;
        total_bytes += next->used;
        prev = next;
    }

    rp_slot_unlock();
    return count;
}

/* True only when there is still a physical place for the next frame
 * directly after this contiguous run.  Never wait to merge across holes or
 * between memSuite chunks.  A FREE neighbor is only a possibility: producer
 * might choose a different chunk, so the wait remains strictly bounded. */
static int rp_batch_can_extend(struct rp_frame_slot **batch, uint32_t count)
{
    if (!count || rp_slot_lock()) return 0;
    struct rp_frame_slot *last = batch[count - 1u];
    uint32_t next_frame = last->frame_number + 1u;
    char *expected = last->start + last->used;
    int possible = 0;
    if (last->chunk_index < rp_frame_chunk_count)
    {
        struct rp_frame_chunk *chunk = &rp_frame_chunks[last->chunk_index];
        if (expected < chunk->start + chunk->bytes)
        {
            for (uint32_t n = chunk->first_slot;
                 n < chunk->first_slot + chunk->slot_count &&
                 n < rp_frame_slot_count; ++n)
            {
                struct rp_frame_slot *s = &rp_frame_slots[n];
                if (s->start != expected) continue;
                if ((s->state == RP_FRAME_SLOT_CAPTURING &&
                     s->frame_number == next_frame) ||
                    (s->state == RP_FRAME_SLOT_FREE &&
                     (uint32_t)(s->end - s->start) >= rp_frame_slot_stride))
                {
                    possible = 1;
                }
                break;
            }
        }
    }
    rp_slot_unlock();
    return possible;
}

/* Keep whole VIDF records.  No intermediate arena and no partial block cuts.
 * Prefer the closest prefix to the learned target without exceeding 128 MiB. */
static uint32_t rp_select_target_prefix(struct rp_frame_slot **batch,
                                        uint32_t count, uint32_t target_bytes,
                                        uint32_t allowed_bytes)
{
    if (!count || batch[0]->used > allowed_bytes) return 0;
    uint32_t bytes = batch[0]->used;
    uint32_t n = 1u;
    while (n < count)
    {
        uint32_t next_bytes = batch[n]->used;
        if (next_bytes > allowed_bytes ||
            bytes > allowed_bytes - next_bytes)
            break;
        uint32_t with_next = bytes + next_bytes;
        if (bytes >= target_bytes) break;
        if (with_next > target_bytes && n > 0u &&
            with_next - target_bytes >= target_bytes - bytes)
            break;
        bytes = with_next;
        ++n;
    }
    return n;
}

static uint32_t rp_wait_for_coalesce(struct rp_frame_slot *first,
                                     struct rp_frame_slot **batch,
                                     uint32_t candidate_frames,
                                     uint32_t free_slots,
                                     uint32_t target_bytes,
                                     const struct rp_v112_decision *pressure,
                                     uint32_t *waited_us)
{
    if (waited_us) *waited_us = 0;
    if (first == NULL || candidate_frames == 0u ||
        rp_writer_drain_requested || free_slots < RP_COALESCE_START_FREE)
        return candidate_frames;

    uint32_t candidate_bytes = rp_batch_bytes(batch, candidate_frames);
    if (!candidate_bytes || candidate_bytes >= target_bytes ||
        !rp_batch_can_extend(batch, candidate_frames))
        return candidate_frames;

    uint32_t avg = rp_writer_recent_frame_bytes;
    if (!avg) avg = candidate_bytes / candidate_frames;
    uint32_t wait_limit = rp_adapt_wait_budget(
        target_bytes, candidate_bytes, avg, free_slots,
        (uint32_t)(session.fps > 0 ? session.fps : 0),
        rp_writer_drain_requested != 0u);
    wait_limit = rp_v112_pressure_wait(pressure, wait_limit);
    if (!wait_limit) return candidate_frames;

    uint32_t t0 = get_us_clock();
    uint32_t candidate = candidate_frames;
    rp_writer_coalesce_waits++;

    while (candidate_bytes < target_bytes && !rp_writer_drain_requested)
    {
        if (rp_count_free_frame_slots() <= RP_COALESCE_ABORT_FREE)
            break;
        if ((uint32_t)(get_us_clock() - t0) >= wait_limit)
            break;
        if (!rp_batch_can_extend(batch, candidate))
            break;
        msleep(1);
        uint32_t newer = rp_collect_contiguous_full_slots(
            first, batch, RP_FRAME_SLOT_MAX);
        if (newer > candidate)
        {
            candidate = newer;
            candidate_bytes = rp_batch_bytes(batch, candidate);
            if (!candidate_bytes) break;
        }
    }

    uint32_t elapsed = (uint32_t)(get_us_clock() - t0);
    if (waited_us) *waited_us = elapsed;
    rp_writer_coalesce_total_us += elapsed;
    if (elapsed > rp_writer_coalesce_max_us)
        rp_writer_coalesce_max_us = elapsed;
    if (candidate_bytes >= target_bytes)
        rp_writer_coalesce_hits++;
    else
        rp_writer_coalesce_timeouts++;
    return candidate;
}

static int rp_consume_group_queue_tokens(struct rp_frame_slot **batch,
                                         uint32_t batch_count)
{
    for (uint32_t i = 1; i < batch_count; i++)
    {
        struct rp_frame_slot *queued = NULL;
        int rc = msg_queue_receive(rp_write_q, &queued, 100);
        if (rc || queued != batch[i])
        {
            RP_START_TRACE("S36 BATCH token FAIL i=%d rc=%d got=%x exp=%x",
                           (int)i, rc, (uint32_t)queued, (uint32_t)batch[i]);
            return -1;
        }
    }
    return 0;
}

static int rp_write_frame_batch_to_disk(struct rp_frame_slot **batch,
                                        uint32_t batch_count,
                                        uint32_t coalesce_wait_us)
{
    if (batch == NULL || batch_count == 0u || batch[0] == NULL)
        return -1;

    if (raw_vid_fp == NULL)
    {
        RP_START_TRACE("S36 BATCH write no-fp frame=%d count=%d",
                       (int)batch[0]->frame_number, (int)batch_count);
        return -2;
    }

    uint32_t fio_bytes = 0u;
    if (rp_slot_lock())
        return -3;

    for (uint32_t i = 0; i < batch_count; i++)
    {
        if (batch[i]->state != RP_FRAME_SLOT_FULL ||
            batch[i]->used == 0u ||
            batch[i]->used > RP_FIO_MAX_BATCH_BYTES ||
            (batch[i]->used & (RP_FRAME_SLOT_ALIGN - 1u)) ||
            ((uintptr_t)batch[i]->start & (RP_FRAME_SLOT_ALIGN - 1u)) ||
            (i > 0u &&
             (batch[i]->chunk_index != batch[i - 1u]->chunk_index ||
              batch[i]->frame_number != batch[i - 1u]->frame_number + 1u ||
              batch[i]->start != batch[i - 1u]->start + batch[i - 1u]->used)) ||
            fio_bytes > RP_FIO_MAX_BATCH_BYTES - batch[i]->used)
        {
            rp_slot_unlock();
            return -4;
        }
        batch[i]->state = RP_FRAME_SLOT_WRITING;
        fio_bytes += batch[i]->used;
    }
    rp_slot_unlock();

    uint32_t written = 0u;
    uint32_t fio_us = 0u;

    if (!rp_header_write_failed)
    {
        rp_wait_fio_safe_phase();

        uint64_t fio_t0 = (uint64_t)get_us_clock();
        RP_START_TRACE("S36 FIO start frame=%d n=%d bytes=%d free=%d prio=%x",
                       (int)batch[0]->frame_number, (int)batch_count,
                       (int)fio_bytes, (int)rp_count_free_frame_slots(),
                       WORKER_PRIO);

        written = FIO_WriteFile(raw_vid_fp, batch[0]->start, fio_bytes);
        fio_us = (uint32_t)((uint64_t)get_us_clock() - fio_t0);
    }

    uint32_t rate_x100 = fio_us
        ? (uint32_t)(((uint64_t)fio_bytes * 100u) / fio_us)
        : 0u;
    if (fio_us > rp_writer_max_us)
        rp_writer_max_us = fio_us;

    if (rate_x100)
    {
        if (rp_writer_rate_x100 == 0u)
            rp_writer_rate_x100 = rate_x100;
        else
            rp_writer_rate_x100 =
                (rp_writer_rate_x100 * 7u + rate_x100) / 8u;
    }

    rp_writer_batches++;
    if (batch_count > rp_writer_max_batch_frames)
        rp_writer_max_batch_frames = batch_count;
    if (fio_bytes > rp_writer_max_batch_bytes)
        rp_writer_max_batch_bytes = fio_bytes;

    RP_START_TRACE("S36 FIO done frame=%d n=%d bytes=%d wrote=%d us=%d rate=%d.%02dMBs avg=%d.%02d",
                   (int)batch[0]->frame_number, (int)batch_count,
                   (int)fio_bytes, (int)written, (int)fio_us,
                   (int)(rate_x100 / 100u), (int)(rate_x100 % 100u),
                   (int)(rp_writer_rate_x100 / 100u),
                   (int)(rp_writer_rate_x100 % 100u));

    int io_ok = (!rp_header_write_failed && written == fio_bytes);
    rp_adapt_observe(&rp_adapt, fio_bytes, fio_us, coalesce_wait_us, io_ok);
    rp_v113_history_observe(&rp_v113_io_history, fio_bytes, fio_us, io_ok);
    if (io_ok)
    {
        uint32_t now_us = get_us_clock();
        if (!rp_drain_window_t0) rp_drain_window_t0 = now_us;
        rp_drain_window_bytes += written;
        uint32_t elapsed_us = now_us - rp_drain_window_t0;
        if (elapsed_us >= 500000u)
        {
            rp_writer_drain_rate_x100 =
                (uint32_t)((rp_drain_window_bytes * 100u) / elapsed_us);
            rp_drain_window_bytes = 0;
            rp_drain_window_t0 = now_us;
        }
    }
    if (!io_ok)
    {
        rp_writer_io_fail++;
        RP_START_TRACE("S36 FIO FAIL headerFail=%d req=%d got=%d",
                       (int)rp_header_write_failed,
                       (int)fio_bytes, (int)written);
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
    }

    if (!rp_slot_lock())
    {
        for (uint32_t i = 0; i < batch_count; i++)
        {
            int idx = rp_slot_index(batch[i]);
            if (idx >= 0)
                rp_free_frame_slot_locked((uint32_t)idx);
        }
        rp_slot_unlock();
    }

    rp_writer_jobs_done += batch_count;
    return io_ok ? 0 : -5;
}

static struct rp_frame_slot *rp_v110_writer_batch[RP_FRAME_SLOT_MAX];
/* Writer-only scratch; never allocated on the small DryOS task stack. */
static uint32_t rp_v113_writer_sizes[RP_FRAME_SLOT_MAX];
static void rp_writer_task(void)
{
    struct rp_frame_slot **batch=rp_v110_writer_batch;

    TASK_LOOP
    {
        struct rp_frame_slot *first = NULL;
        int rc = msg_queue_receive(rp_write_q, &first, 0);
        if (rc)
        {
            RP_START_TRACE("S36 WR receive FAIL rc=%d", rc);
            continue;
        }

        if (first == RP_WRITE_HEADER_TOKEN)
        {
            uint32_t written = 0u;
            uint32_t fio_us = 0u;
            if (raw_vid_fp != NULL &&
                rp_header_write_ptr != NULL &&
                rp_header_write_size != 0u)
            {
                uint64_t t0 = (uint64_t)get_us_clock();
                written = FIO_WriteFile(
                    raw_vid_fp, rp_header_write_ptr, rp_header_write_size);
                fio_us = (uint32_t)((uint64_t)get_us_clock() - t0);
            }

            if (written != rp_header_write_size)
            {
                rp_header_write_failed = 1u;
                RP_START_TRACE("S36 REV2 HEADER FAIL req=%d got=%d",
                               (int)rp_header_write_size, (int)written);
                raw_vid_rp_request_stop();
                raw_vid_request_stop_async();
            }
            else
            {
                rp_header_write_done = 1u;
                RP_START_TRACE("S36 REV2 HEADER wrote=%d us=%d",
                               (int)written, (int)fio_us);
            }

            rp_header_write_pending = 0u;
            continue;
        }

        if (first == NULL)
        {
            RP_START_TRACE("S36 WR receive NULL");
            continue;
        }

        if (first->state != RP_FRAME_SLOT_FULL)
        {
            RP_START_TRACE("S36 WR state FAIL frame=%d state=%d",
                           (int)first->frame_number, (int)first->state);
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
            continue;
        }

        uint32_t candidate =
            rp_collect_contiguous_full_slots(first, batch, RP_FRAME_SLOT_MAX);
        if (candidate == 0u)
        {
            RP_START_TRACE("S36 BATCH collect FAIL frame=%d",
                           (int)first->frame_number);
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
            continue;
        }

        uint32_t free_slots = rp_count_free_frame_slots();
        struct rp_v112_decision pressure = rp_v112_writer_pressure(free_slots);
        uint32_t target_bytes = rp_adapt_target(
            &rp_adapt, free_slots, rp_writer_drain_requested != 0u);
        if (pressure.level >= RP_V112_PRESSURE_HIGH)
            target_bytes = rp_adapt_sizes_bytes[rp_adapt.allowed_index];
        uint32_t coalesce_wait_us = 0u;
        candidate = rp_wait_for_coalesce(
            first, batch, candidate, free_slots, target_bytes,
            &pressure, &coalesce_wait_us);
        free_slots = rp_count_free_frame_slots();

        uint32_t candidate_bytes = rp_batch_bytes(batch, candidate);
        uint32_t allowed_bytes = (rp_adapt.allowed_index ==
            RP_ADAPT_SAFE_LAST_BUCKET)
            ? (0xFFFEu * 512u)
            : rp_adapt_sizes_bytes[rp_adapt.allowed_index];
        uint32_t to_target = rp_select_target_prefix(
            batch, candidate, target_bytes, allowed_bytes);
        if (!to_target)
        {
            RP_START_TRACE("S36 ADAPT first frame exceeds cap used=%d cap=%d",
                           (int)first->used, (int)allowed_bytes);
            rp_writer_drain_requested = 1u;
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
            continue;
        }
        pressure = rp_v112_writer_pressure(free_slots);
        /* v113: the first READY prefix that completes FIO soon enough to
         * return slots before a no-progress overflow. Do not change the
         * adaptive size cap or merge across different Canon memory owners. */
        struct rp_v113_input vin;
        struct rp_v113_choice choice;
        vin.free_bytes = rp_v112_free_bytes();
        vin.free_slots = free_slots;
        vin.ingress_x100 = rp_writer_ingress_rate_x100;
        vin.fps_x1000 = session.fps > 0 ? (uint32_t)session.fps : 0u;
        vin.fallback_rate_x100 = rp_writer_rate_x100;
        vin.draining = rp_writer_drain_requested;
        vin.pressure = &pressure;
        vin.history = &rp_v113_io_history;
        for (uint32_t i = 0; i < to_target; ++i)
            rp_v113_writer_sizes[i] = batch[i]->used;
        rp_v113_select_prefix(&vin, rp_v113_writer_sizes, to_target, &choice);
        uint32_t selected = choice.frames;
        if (!selected)
        {
            RP_START_TRACE("S36 V113 invalid-prefix n=%d", (int)to_target);
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
            continue;
        }
        RP_START_TRACE("S36 V113 p=%d freeKB=%d slots=%d n=%d/%d",
                       (int)pressure.level, (int)(vin.free_bytes / 1024u),
                       (int)free_slots, (int)selected, (int)to_target);
        RP_START_TRACE("S36 V113 time budget=%d pred=%d noRel=%d",
                       choice.budget_us == UINT32_MAX ? -1 : (int)(choice.budget_us / 1000u),
                       (int)(choice.predicted_us / 1000u),
                       choice.no_release_us == UINT32_MAX ? -1 : (int)(choice.no_release_us / 1000u));
        RP_START_TRACE("S36 V113 rate=%d.%02d warn=%d tail=%d fb=%d",
                       (int)(choice.conservative_rate_x100 / 100u),
                       (int)(choice.conservative_rate_x100 % 100u),
                       (int)choice.budget_exceeded,
                       (int)rp_v112_tail_hits, (int)rp_v112_fallbacks);

        if (rp_consume_group_queue_tokens(batch, selected))
        {
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
            continue;
        }

        uint32_t selected_bytes = rp_batch_bytes(batch, selected);
        uint32_t qdepth = 0;
        msg_queue_count(rp_write_q, &qdepth);
        RP_START_TRACE("S36 BATCH select first=%d candidate=%d/%dKB n=%d/%dKB q=%d free=%d reserved=%d wait=%d",
                       (int)first->frame_number,
                       (int)candidate, (int)(candidate_bytes / 1024u),
                       (int)selected, (int)(selected_bytes / 1024u),
                       (int)qdepth, (int)free_slots,
                       (int)rp_count_reserved_frame_slots(),
                       (int)coalesce_wait_us);

        RP_START_TRACE("S36 ADAPT target=%dKB cap=%dKB ingress=%d.%02d drain=%d.%02d fio=%d.%02d eff=%d.%02d free=%d probe=%d stop=%d",
                       (int)(target_bytes / 1024u),
                       (int)(rp_adapt_sizes_bytes[rp_adapt.allowed_index] / 1024u),
                       (int)(rp_writer_ingress_rate_x100 / 100u),
                       (int)(rp_writer_ingress_rate_x100 % 100u),
                       (int)(rp_writer_drain_rate_x100 / 100u),
                       (int)(rp_writer_drain_rate_x100 % 100u),
                       (int)(rp_adapt.last_write_rate_x100 / 100u),
                       (int)(rp_adapt.last_write_rate_x100 % 100u),
                       (int)(rp_adapt.last_effective_rate_x100 / 100u),
                       (int)(rp_adapt.last_effective_rate_x100 % 100u),
                       (int)free_slots, (int)rp_adapt.probe_attempts,
                       (int)rp_writer_drain_requested);
        int write_rc = rp_write_frame_batch_to_disk(
            batch, selected, coalesce_wait_us);
        if (write_rc)
        {
            RP_START_TRACE("S36 ADAPT write abort rc=%d", write_rc);
            rp_writer_drain_requested = 1u;
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
        }
        RP_START_TRACE("S36 WR complete done=%d queued=%d free=%d reserved=%d",
                       (int)rp_writer_jobs_done,
                       (int)rp_writer_jobs_queued,
                       (int)rp_count_free_frame_slots(),
                       (int)rp_count_reserved_frame_slots());
    }
}

/* No disk operations, no W3/PU MMIO. Safe to call after the recorder
 * reaches INACTIVE or during the between-case cooldown. Zero counters in an
 * active new session do not themselves count as idle: also check state/file. */
void raw_vid_rp_writer_get_status(struct raw_vid_rp_writer_status *status)
{
    if (!status) return;
    memset(status,0,sizeof(*status));
    status->queued = rp_writer_jobs_queued;
    status->done = rp_writer_jobs_done;
    status->header_pending = rp_header_write_pending;
    status->io_fail = rp_writer_io_fail;
    status->post_fail = rp_writer_post_fail;
    status->batches = rp_writer_batches;
    status->max_batch_bytes = rp_writer_max_batch_bytes;
    status->max_write_us = rp_writer_max_us;
    status->rate_mb_s_x100 = rp_writer_rate_x100;
    status->ingress_mb_s_x100 = rp_writer_ingress_rate_x100;
    status->drain_mb_s_x100 = rp_writer_drain_rate_x100;
    status->adaptive_target_bytes = rp_adapt.last_target_bytes;
    status->adaptive_allowed_bytes =
        rp_adapt_sizes_bytes[rp_adapt.allowed_index];
    status->file_open = raw_vid_fp != NULL;
    /* Failure to inspect the queue is NOT an idle state. */
    if (!rp_write_q || msg_queue_count(rp_write_q,&status->queue_depth))
        status->queue_depth = 0xffffffffu;
    status->idle = (get_recording_state() == INACTIVE &&
                    status->queued == status->done &&
                    !status->queue_depth && !status->header_pending &&
                    !rp_header_write_failed && !status->io_fail &&
                    !status->post_fail && !status->file_open);
}

static int rp_drain_frame_slots_on_stop(void)
{
    uint64_t start = (uint64_t)get_us_clock();
    uint64_t timeout_us = 7000000u;
    uint32_t pending = rp_writer_jobs_queued - rp_writer_jobs_done;
    uint32_t speed = rp_writer_rate_x100;
    if (speed && pending && rp_writer_recent_frame_bytes)
    {
        uint64_t expected_us =
            ((uint64_t)pending * rp_writer_recent_frame_bytes * 100ull) / speed;
        uint64_t predicted_us = expected_us * 2ull + 3000000ull;
        if (predicted_us > timeout_us) timeout_us = predicted_us;
        if (timeout_us > 30000000ull) timeout_us = 30000000ull;
    }
    RP_START_TRACE("S36 ADAPT drain pending=%d limit_us=%d",
                   (int)pending, (int)timeout_us);

    while ((rp_writer_jobs_done != rp_writer_jobs_queued ||
            rp_header_write_pending) &&
           ((uint64_t)get_us_clock() - start) < timeout_us)
    {
        msleep(10);
    }

    if (rp_writer_jobs_done != rp_writer_jobs_queued ||
        rp_header_write_pending || rp_header_write_failed)
    {
        return -1;
    }

    if (rp_slot_lock())
        return -2;

    int bad = 0;
    for (uint32_t i = 0; i < rp_frame_slot_count; i++)
    {
        if (rp_frame_slots[i].state == RP_FRAME_SLOT_FREE ||
            rp_frame_slots[i].state == RP_FRAME_SLOT_RESERVED)
        {
            continue;
        }

        if (RP_CRX_ROUNDTRIP_VERIFY &&
            &rp_frame_slots[i] == rp_crx_ref_slot &&
            rp_frame_slots[i].state == RP_FRAME_SLOT_REFERENCE)
        {
            continue;
        }

        bad = 1;
        break;
    }
    rp_slot_unlock();
    return bad ? -3 : 0;
}

static const char *rp_crx_ref_path_for_bits(uint32_t bits)
{
    if (bits == 10u) return "ML/LOGS/RPCRX10.BIN";
    if (bits == 12u) return "ML/LOGS/RPCRX12.BIN";
    if (bits == 14u) return "ML/LOGS/RPCRX14.BIN";
    return NULL;
}

static int rp_hold_crx_reference(struct rp_frame_slot *source,
                                 uint32_t width, uint32_t height,
                                 uint32_t bits, uint32_t frame_number,
                                 uint32_t payload_bytes)
{
    if (!RP_CRX_ROUNDTRIP_VERIFY)
        return 0;
    if (source == NULL || rp_crx_ref_slot != NULL)
        return -1100;
    if (bits != 10u && bits != 12u && bits != 14u)
        return -1101;

    uint32_t row_bits = width * bits;
    if ((row_bits & 7u) != 0u)
        return -1102;
    uint32_t row_bytes = row_bits / 8u;
    if (row_bytes == 0u || height == 0u ||
        (uint64_t)row_bytes * height != payload_bytes)
        return -1103;

    if (source->state != RP_FRAME_SLOT_CAPTURING)
        return -1104;

    uint32_t capacity = (uint32_t)(source->end - source->start);
    if (payload_bytes > capacity)
        return -1105;

    /*
     * v7 zero-copy round-trip reference:
     * this is the exact tight input buffer that StartEncode just consumed.
     * Keep the slot intact and build the revision-2 VIDF in a second output slot.
     */
    source->used = payload_bytes;
    source->frame_number = frame_number;
    source->state = RP_FRAME_SLOT_REFERENCE;

    rp_crx_ref_slot = source;
    rp_crx_ref_width = width;
    rp_crx_ref_height = height;
    rp_crx_ref_bits = bits;
    rp_crx_ref_row_bytes = row_bytes;
    rp_crx_ref_payload_bytes = payload_bytes;
    rp_crx_ref_frame_number = frame_number;
    rp_crx_ref_copy_us = 0;

    RP_START_TRACE("CRX REF ZERO-COPY PASS f=%d bpp=%d wh=%d,%d row=%d bytes=%d slot=%x",
                   (int)frame_number, (int)bits, (int)width, (int)height,
                   (int)row_bytes, (int)payload_bytes,
                   (uint32_t)(uintptr_t)source->start);
    return 0;
}

/* RAW-LV and the Canon/ML writer have already been stopped when this runs.
 * Therefore the extra reference FIO cannot perturb the encoder experiment. */
static int rp_write_crx_reference_sidecar(void)
{
    if (!RP_CRX_ROUNDTRIP_VERIFY)
        return 0;
    if (rp_record_codec != RAW_VID_CODEC_CRX)
        return 0;
    if (rp_crx_ref_slot == NULL ||
        rp_crx_ref_slot->state != RP_FRAME_SLOT_REFERENCE)
    {
        RP_START_TRACE("CRX REF write missing reference");
        return -1110;
    }

    const char *path = rp_crx_ref_path_for_bits(rp_crx_ref_bits);
    if (path == NULL)
        return -1111;

    struct rp_crx_ref_header hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, "RPCRXR1", 8u);
    hdr.version = RP_CRX_REF_VERSION;
    hdr.width = rp_crx_ref_width;
    hdr.height = rp_crx_ref_height;
    hdr.bits_per_pixel = rp_crx_ref_bits;
    hdr.row_bytes = rp_crx_ref_row_bytes;
    hdr.payload_bytes = rp_crx_ref_payload_bytes;
    hdr.frame_number = rp_crx_ref_frame_number;
    hdr.copy_us = rp_crx_ref_copy_us;

    FIO_RemoveFile(path);
    FILE *f = FIO_CreateFile(path);
    if (f == NULL)
        return -1112;

    uint32_t h_written = FIO_WriteFile(f, &hdr, sizeof(hdr));
    uint32_t p_written = 0;
    if (h_written == sizeof(hdr))
    {
        p_written = FIO_WriteFile(f, rp_crx_ref_slot->start,
                                  rp_crx_ref_payload_bytes);
    }
    FIO_CloseFile(f);

    int rc = 0;
    if (h_written != sizeof(hdr) || p_written != rp_crx_ref_payload_bytes)
        rc = -1113;

    RP_START_TRACE("CRX REF write rc=%d path=%s hdr=%d/%d payload=%d/%d",
                   rc, path, (int)h_written, (int)sizeof(hdr),
                   (int)p_written, (int)rp_crx_ref_payload_bytes);

    rp_abandon_frame_slot(rp_crx_ref_slot);
    rp_crx_ref_slot = NULL;
    return rc;
}

/* RP MVP stop path: wait for any outstanding asynchronous full-buffer write,
 * then synchronously flush the remaining partial buffer(s).  The legacy
 * raw_vidx path historically dropped this final partial data. */
static int rp_flush_crop_bufs_on_stop(struct crop_buf crop_bufs[])
{
    uint64_t start = (uint64_t)get_us_clock();
    const uint64_t timeout_us = 3000000u;

    while ((crop_bufs[0].writing || crop_bufs[1].writing) &&
           ((uint64_t)get_us_clock() - start) < timeout_us)
    {
        msleep(10);
    }

    if (crop_bufs[0].writing || crop_bufs[1].writing)
        return -1;

    if (raw_vid_fp == NULL)
        return -2;

    for (int i = 0; i < 2; i++)
    {
        if (crop_bufs[i].start && crop_bufs[i].cur > crop_bufs[i].start)
        {
            uint32_t size = (uint32_t)(crop_bufs[i].cur - crop_bufs[i].start);
            FIO_WriteFile(raw_vid_fp, crop_bufs[i].start, size);
            crop_bufs[i].cur = crop_bufs[i].start;
        }
    }
    return 0;
}

static int rp_process_frame_slot(struct raw_vid_event *event,
                                 uint32_t *video_frame_count)
{
    struct rp_frame_slot *slot = rp_get_free_frame_slot();
    if (slot == NULL)
    {
        rp_slot_starves++;
        uint32_t qdepth = 0;
        msg_queue_count(rp_write_q, &qdepth);
        RP_START_TRACE("S36 SLOT STARVE count=%d q=%d queued=%d done=%d",
                       (int)rp_frame_slot_count, (int)qdepth,
                       (int)rp_writer_jobs_queued, (int)rp_writer_jobs_done);
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
        return -1;
    }

    uint32_t frame_no = *video_frame_count;
    char *dst = write_video_frame_header_padded((uint32_t)event->size,
                                                rp_frame_slot_stride,
                                                slot->start, slot->end);
    if (dst == NULL)
    {
        RP_START_TRACE("S36 SLOT header FAIL frame=%d cap=%d payload=%d",
                       (int)frame_no, (int)(slot->end - slot->start),
                       (int)event->size);
        rp_abandon_frame_slot(slot);
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
        return -2;
    }

    /* blockSize deliberately includes sector padding after the RAW payload.
     * This is the mlv_lite layout: each slot is a complete MLV block and its
     * size is exactly the physical slot stride, so adjacent slots can be
     * written with one FIO call without gaps or copies. */
    slot->used = rp_frame_slot_stride;
    slot->frame_number = frame_no;
    (*video_frame_count)++;

    int err = take_semaphore_now(edmac_sem);
    if (err)
    {
        RP_START_TRACE("S36 SLOT edmac busy frame=%d rc=%d", (int)frame_no, err);
        rp_abandon_frame_slot(slot);
        if (*video_frame_count > 0)
            (*video_frame_count)--;
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
        return -3;
    }

    uint32_t m2m_us = 0;
    int m2m_rc;
    m2m_rc = raw_vid_rp_m2m_crop_once(
        event->src, raw_info.pitch,
        event->x, event->y, event->w, event->h,
        dst, (uint32_t)event->size, &m2m_us);

    give_semaphore(edmac_sem);

    if (m2m_rc)
    {
        rp_abandon_frame_slot(slot);
        if (*video_frame_count > 0)
            (*video_frame_count)--;
        RP_START_TRACE("S36 SLOT crop FAIL rc=%d us=%d frame=%d",
                       m2m_rc, (int)m2m_us, (int)frame_no);
        rp_worker_m2m_failed = 1;
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
        return -4;
    }

    int write_rc = rp_queue_frame_slot_write(slot);
    if (write_rc)
    {
        RP_START_TRACE("S36 SLOT queue FAIL rc=%d frame=%d",
                       write_rc, (int)frame_no);
        rp_abandon_frame_slot(slot);
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
        return -5;
    }

    return 0;
}

// This is the heart of worker.c.
//
// This runs as a task whenever the module is loaded,
// regardless of whether a recording is in progress.
//
// It is the only consumer of the message queue. 

static int rp_crx_release_handles(const struct rp_crx_result *result,
                                  uint32_t *first_rc)
{
    if (first_rc)
        *first_rc = 0;
    if (!result)
        return -1;

    uint32_t n = MIN(result->count, RP_CRX_RESULT_MAX);
    int bad = 0;
    for (uint32_t i = 0; i < n; i++)
    {
        if (!result->handles[i])
            continue;
        int rc = raw_vid_rp_crx_free(result->handles[i]);
        if (i == 0 && first_rc)
            *first_rc = (uint32_t)rc;
        if (rc & 1)
        {
            rp_crx_release_failures++;
            bad = rc ? rc : 1;
        }
    }
    return bad;
}

static uint16_t rp_crx_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static uint32_t rp_crx_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

/* Validate the opaque Canon CMP1 enough to prove that the stream-level
 * declaration we write is compatible with this recorder session.
 *
 * Return:
 *   1  first valid CMP1 latched
 *   0  later CMP1 is byte-identical to the latched configuration
 *  <0  malformed/incompatible/changed configuration
 */
static int rp_crx_validate_and_latch_cmp1(const void *cmp1_ptr,
                                          uint32_t cmp1_size,
                                          uint32_t logical_w,
                                          uint32_t logical_h,
                                          uint32_t logical_bits,
                                          uint32_t sample_header_size,
                                          int lossy)
{
    if (!cmp1_ptr || cmp1_size < 32u || cmp1_size > RP_CRX_META_MAX_BYTES)
        return -1;

    const uint8_t *c = (const uint8_t *)cmp1_ptr;
    const uint16_t version = rp_crx_be16(c + 4u);
    const uint32_t coded_w = rp_crx_be32(c + 8u);
    const uint32_t coded_h = rp_crx_be32(c + 12u);
    const uint8_t nbits = c[24];
    const uint8_t nplanes = c[25] >> 4;
    const uint8_t cfa_layout = c[25] & 0x0Fu;
    const uint8_t enc_type = c[26] >> 4;
    const uint8_t image_levels = c[26] & 0x0Fu;
    const uint32_t mdat_hdr_size = rp_crx_be32(c + 28u);

    int codec_ok = lossy
        ? (enc_type == 3u || image_levels > 0u)
        : (enc_type == 0u && image_levels == 0u);

    if ((version != 0x0100u && version != 0x0200u) ||
        coded_w != logical_w || coded_h != logical_h ||
        nbits != logical_bits ||
        nplanes != 4u || cfa_layout > 3u ||
        !codec_ok ||
        mdat_hdr_size != sample_header_size)
    {
        RP_START_TRACE("CRX CMP1 reject lossy=%d ver=%x wh=%d,%d/%d,%d bits=%d/%d planes=%d cfa=%d enc=%d lev=%d hdr=%d/%d",
                       lossy ? 1 : 0, (int)version,
                       (int)coded_w, (int)coded_h,
                       (int)logical_w, (int)logical_h,
                       (int)nbits, (int)logical_bits,
                       (int)nplanes, (int)cfa_layout,
                       (int)enc_type, (int)image_levels,
                       (int)mdat_hdr_size, (int)sample_header_size);
        return -2;
    }

    /* Current pipeline is explicitly same-domain end-to-end.  Do not infer a
     * shift from nBits: fail above if that contract changes. */
    rp_crx_value_shift = RP_CRX_VALUE_SHIFT_CURRENT;

    if (rp_crx_cmp1_size == 0u)
    {
        memcpy(rp_crx_cmp1_cache, c, cmp1_size);
        rp_crx_cmp1_size = cmp1_size;
        RP_START_TRACE("CRX CMP1 latch lossy=%d size=%d shift=%d wh=%d,%d bits=%d enc=%d lev=%d hdr=%d",
                       lossy ? 1 : 0, (int)cmp1_size, (int)rp_crx_value_shift,
                       (int)coded_w, (int)coded_h, (int)nbits,
                       (int)enc_type, (int)image_levels, (int)mdat_hdr_size);
        return 1;
    }

    if (cmp1_size != rp_crx_cmp1_size ||
        memcmp(rp_crx_cmp1_cache, c, cmp1_size) != 0)
    {
        RP_START_TRACE("CRX CMP1 CHANGED old=%d new=%d; stop logical clip",
                       (int)rp_crx_cmp1_size, (int)cmp1_size);
        return -3;
    }

    return 0;
}

static int rp_process_crx_frame_slot(struct raw_vid_event *event,
                                     uint32_t *video_frame_count,
                                     char *header_buf,
                                     uint32_t header_buf_size,
                                     uint32_t *mlv_header_size)
{
    struct rp_frame_slot *input_slot = rp_get_free_frame_slot();
    struct rp_frame_slot *out_slot = input_slot;
    if (input_slot == NULL)
    {
        rp_slot_starves++;
        RP_START_TRACE("CRX SLOT STARVE count=%d", (int)rp_frame_slot_count);
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
        return -1000;
    }

    const uint32_t frame_no = *video_frame_count;
    const uint32_t bits = (uint32_t)raw_vid_get_bpp();
    const int lossy = raw_vid_codec_is_lossy_crx(rp_record_codec);
    const uint32_t input_bits = (uint32_t)event->w * (uint32_t)event->h * bits;
    const uint32_t input_bytes = input_bits / 8u;
    const uint32_t input_slot_capacity =
        (uint32_t)(input_slot->end - input_slot->start);
    int rc = 0;
    int sem_rc = 0;
    int release_bad = 0;
    uint32_t first_release_rc = 0;
    struct rp_crx_result result;
    memset(&result, 0, sizeof(result));

    if ((bits != 10u && bits != 12u && bits != 14u) ||
        (input_bits & 7u) != 0u ||
        input_bytes == 0u || input_bytes > input_slot_capacity)
    {
        rc = -1001;
        goto fail_no_handle;
    }

    uint32_t input_us = 0;
    uint32_t ref_copy_us = 0;
    int use_direct_w3 = rp_crx_standard_direct_w3_allowed();
    if (!use_direct_w3)
    {
        RP_START_TRACE("V23.2 DIRECT required");
        rc = -1032;
        goto fail_no_handle;
    }
    if (lossy && !use_direct_w3)
    {
        RP_START_TRACE("CRAW reject: standard direct-W3 path not available");
        rc = -1030;
        goto fail_no_handle;
    }

    /*
     * Phase A / input acquisition.
     *
     * STANDARD capture:
     *   hold the completed W3 capture slot and submit physical W3 + ROI
     *   directly to E0651986 / Canon AlgsMgr.
     *
     * EXPERIMENT capture (latch/retry/session or cursor/tail audit):
     *   preserve v20.1 exactly: W3 -> tight R46/W13 slot, release W3,
     *   then submit the tight source.
     */
    if (use_direct_w3)
    {
        uint32_t source_width = (uint32_t)raw_info.width;
        uint32_t source_pitch = (uint32_t)raw_info.pitch;
        uint32_t source_height = (uint32_t)raw_info.height;
        uint64_t expected_pitch = (uint64_t)source_width * bits / 8u;

        if (event->capture_slot < 0 ||
            !event->src ||
            (((uintptr_t)event->src) & 0x3Fu) != 0u ||
            source_width == 0u || source_height == 0u ||
            expected_pitch != source_pitch ||
            event->x + event->w > source_width ||
            event->y + event->h > source_height ||
            (((uint32_t)event->x * bits) & 7u) != 0u)
        {
            RP_START_TRACE("CRX DIRECT W3 reject f=%d src=%x slot=%d phys=%d,%d pitch=%d roi=%d,%d %dx%d bpp=%d",
                           (int)frame_no, (uint32_t)event->src,
                           (int)event->capture_slot,
                           (int)source_width, (int)source_height,
                           (int)source_pitch,
                           (int)event->x, (int)event->y,
                           (int)event->w, (int)event->h, (int)bits);
            rc = -1020;
            goto fail_no_handle;
        }

        RP_START_TRACE("CRX DIRECT W3 submit f=%d lossy=%d src=%x slot=%d phys=%d,%d pitch=%d roi=%d,%d %dx%d bpp=%d",
                       (int)frame_no, lossy ? 1 : 0, (uint32_t)event->src,
                       (int)event->capture_slot,
                       (int)source_width, (int)source_height,
                       (int)source_pitch,
                       (int)event->x, (int)event->y,
                       (int)event->w, (int)event->h, (int)bits);

        rc = raw_vid_rp_crx_encode_source_mode(
            event->src, source_width,
            event->x, event->y, event->w, event->h,
            bits, lossy, &result);
        rp_crx_direct_w3_frames++;
    }
    else
    {
        if (lossy)
        {
            rc = -1031;
            goto fail_no_handle;
        }
        sem_rc = take_semaphore_now(edmac_sem);
        if (sem_rc)
        {
            rc = -1002;
            goto fail_no_handle;
        }

        rc = raw_vid_rp_m2m_crop_once(
            event->src, raw_info.pitch,
            event->x, event->y, event->w, event->h,
            input_slot->start, input_bytes, &input_us);
        give_semaphore(edmac_sem);

        if (rc)
        {
            RP_START_TRACE("CRX input crop FAIL frame=%d rc=%d us=%d",
                           (int)frame_no, rc, (int)input_us);
            rc = -1003;
            goto fail_no_handle;
        }
        rp_crx_total_input_us += input_us;
        rp_crx_staged_input_frames++;

        RP_START_TRACE("CRX STAGED INPUT f=%d us=%d",
                       (int)frame_no, (int)input_us);

        if (event->capture_slot >= 0)
        {
            raw_vid_rp_release_buffer(event->capture_slot);
            event->capture_slot = -1;
        }

        rc = raw_vid_rp_crx_encode(
            input_slot->start, event->w, event->h, bits, &result);
    }

    rp_crx_total_encode_us += result.encode_us;
    if (rc)
    {
        RP_START_TRACE("CRX encode FAIL frame=%d rc=%d submit=%x count=%d us=%d",
                       (int)frame_no, rc, result.submit_rc,
                       (int)result.count, (int)result.encode_us);
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1004;
        goto fail_no_handle;
    }

    if (result.count != 1u || result.handles[0] == 0u)
    {
        RP_START_TRACE("CRX result count FAIL frame=%d count=%d h0=%x h1=%x h2=%x",
                       (int)frame_no, (int)result.count,
                       result.handles[0], result.handles[1], result.handles[2]);
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1005;
        goto fail_no_handle;
    }

    /*
     * Direct-W3 lifetime boundary: raw_vid_rp_crx_encode_source() returned
     * only after the completion callback. In the round-trip build, generate
     * the independent tight reference NOW, while the same W3 slot is still
     * held. This copy cannot affect encoder input.
     */
    if (use_direct_w3)
    {
        if (RP_CRX_ROUNDTRIP_VERIFY && frame_no == 0u)
        {
            sem_rc = take_semaphore_now(edmac_sem);
            if (sem_rc)
            {
                rp_crx_release_handles(&result, &first_release_rc);
                rc = -1021;
                goto fail_no_handle;
            }

            int ref_rc = raw_vid_rp_m2m_crop_once(
                event->src, raw_info.pitch,
                event->x, event->y, event->w, event->h,
                input_slot->start, input_bytes, &ref_copy_us);
            give_semaphore(edmac_sem);

            if (ref_rc)
            {
                RP_START_TRACE("CRX DIRECT W3 ref crop FAIL f=%d rc=%d us=%d",
                               (int)frame_no, ref_rc, (int)ref_copy_us);
                rp_crx_release_handles(&result, &first_release_rc);
                rc = -1022;
                goto fail_no_handle;
            }

            RP_START_TRACE("CRX DIRECT W3 ref crop PASS f=%d us=%d bytes=%d",
                           (int)frame_no, (int)ref_copy_us, (int)input_bytes);
        }

        if (event->capture_slot >= 0)
        {
            RP_START_TRACE("CRX DIRECT W3 release slot=%d after callback enc_us=%d",
                           (int)event->capture_slot, (int)result.encode_us);
            raw_vid_rp_release_buffer(event->capture_slot);
            event->capture_slot = -1;
        }
    }

    uint32_t handle = result.handles[0];
    volatile uint32_t *hw = (volatile uint32_t *)(uintptr_t)handle;
    uint32_t profile = hw[1];
    uint32_t expected_profile = result.descriptor_type;
    uint32_t route_id = result.encoder_id;
    uint32_t main_ptr = hw[2];
    uint32_t main_size = hw[3];
    uint32_t ring_slots = hw[6];
    uint32_t ring_addr_ptr = hw[7];
    uint32_t ring_size_ptr = hw[8];

    /* Canon copies the submitted descriptor.type into result_handle[1].
     * The old validator incorrectly derived this value from BPP, which is
     * only true for the historical fixed routes (14->0, 12->4, 10->5).
     * Dynamic D-slot mode deliberately decouples BPP from route type. */
    int route_type_ok = lossy
        ? (expected_profile == 2u || expected_profile == 3u ||
           expected_profile == 6u || expected_profile == 7u)
        : (expected_profile == 0u || expected_profile == 1u ||
           expected_profile == 4u || expected_profile == 5u);

    if (!route_type_ok || route_id >= 22u ||
        profile != expected_profile ||
        !main_ptr || main_size == 0u || main_size > RP_CRX_META_MAX_BYTES ||
        ring_slots == 0u || ring_slots > RP_CRX_RING_MAX ||
        !ring_addr_ptr || !ring_size_ptr)
    {
        RP_START_TRACE("CRX handle FAIL f=%d h=%x prof=%d exp=%d id=%d main=%x/%d",
                       (int)frame_no, handle, (int)profile, (int)expected_profile,
                       (int)route_id, main_ptr, (int)main_size);
        RP_START_TRACE("CRX handle ring slots=%d a=%x s=%x",
                       (int)ring_slots, ring_addr_ptr, ring_size_ptr);
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1006;
        goto fail_no_handle;
    }

    volatile uint32_t *ring_addr =
        (volatile uint32_t *)(uintptr_t)ring_addr_ptr;
    volatile uint32_t *ring_size =
        (volatile uint32_t *)(uintptr_t)ring_size_ptr;

    uint32_t sub_ptr = ring_addr[0];
    uint32_t sub_size = ring_size[0];
    if (!sub_ptr || sub_size == 0u || sub_size > RP_CRX_META_MAX_BYTES)
    {
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1007;
        goto fail_no_handle;
    }

    uint32_t segment_count = 0;
    uint32_t payload_bytes = 0;

    /* v12 runtime shows type/profile 0, 4 and 5 use the same Canon
     * CreateHandle family.  Keep the original strict topology checks: ring[0]
     * is the CPU-readable CRX subheader and payload must occupy odd indices.
     * Any profile-specific deviation still fails closed below. */
    for (uint32_t i = 2u; i < ring_slots; i += 2u)
    {
        if (ring_addr[i] || ring_size[i])
        {
            RP_START_TRACE("CRX even-ring populated f=%d i=%d a=%x z=%d",
                           (int)frame_no, (int)i, ring_addr[i],
                           (int)ring_size[i]);
            rp_crx_release_handles(&result, &first_release_rc);
            rc = -1008;
            goto fail_no_handle;
        }
    }

    for (uint32_t i = 1u; i < ring_slots; i += 2u)
    {
        uint32_t a = ring_addr[i];
        uint32_t z = ring_size[i];
        if (!a && !z)
            continue;
        if (!a || !z || (z & 1u) || segment_count >= RP_CRX_MAX_SEGMENTS ||
            payload_bytes > 0xFFFFFFFFu - z)
        {
            RP_START_TRACE("CRX ring FAIL f=%d i=%d a=%x z=%d seg=%d",
                           (int)frame_no, (int)i, a, (int)z,
                           (int)segment_count);
            rp_crx_release_handles(&result, &first_release_rc);
            rc = -1008;
            goto fail_no_handle;
        }
        rp_crx_src_addr[segment_count] = a;
        rp_crx_src_size[segment_count] = z;
        payload_bytes += z;
        segment_count++;
    }

    if (segment_count == 0u)
    {
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1009;
        goto fail_no_handle;
    }

    if (lossy)
    {
        /* Native full C-RAW on RP was observed with a much larger mdat
         * subheader (~0x438) and 40 real payload segments.  Do not hard-code
         * those values for the synthetic 1920x1080 test; log the topology and
         * let CMP1 + host decode decide validity. */
        RP_START_TRACE("CRAW topology f=%d profile=%d hdr=%d ring=%d seg=%d payload=%d",
                       (int)frame_no, (int)profile, (int)sub_size,
                       (int)ring_slots, (int)segment_count,
                       (int)payload_bytes);
    }

    /*
     * MLV CRX container revision 2 stream configuration.
     *
     * main_ptr/main_size is the exact Canon CMP1 returned by this encode.
     * Latch it from frame 0, validate it against the actual recorder pipeline,
     * then require byte-identical CMP1 for the rest of the logical clip.
     */
    int cmp1_state = rp_crx_validate_and_latch_cmp1(
        (const void *)(uintptr_t)main_ptr, main_size,
        (uint32_t)event->w, (uint32_t)event->h, bits, sub_size, lossy);
    if (cmp1_state < 0)
    {
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1016;
        goto fail_no_handle;
    }

    if (header_buf == NULL || mlv_header_size == NULL || header_buf_size == 0u)
    {
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1017;
        goto fail_no_handle;
    }

    int first_stream_frame = (*mlv_header_size == 0u);
    int header_needs_queue = 0;
    if (first_stream_frame)
    {
        if (cmp1_state != 1)
        {
            rp_crx_release_handles(&result, &first_release_rc);
            rc = -1018;
            goto fail_no_handle;
        }

        uint32_t h = (uint32_t)write_file_headers_crx_v2(
            header_buf, header_buf + header_buf_size,
            rp_crx_cmp1_cache, rp_crx_cmp1_size, rp_crx_value_shift);
        if (h == 0u || h > header_buf_size)
        {
            RP_START_TRACE("CRX REV2 header FAIL cmp1=%d shift=%d",
                           (int)rp_crx_cmp1_size, (int)rp_crx_value_shift);
            rp_crx_release_handles(&result, &first_release_rc);
            rc = -1019;
            goto fail_no_handle;
        }
        *mlv_header_size = h;
        header_needs_queue = 1;
        RP_START_TRACE("CRX REV2 header READY bytes=%d crxi=40+%d shift=%d",
                       (int)h, (int)rp_crx_cmp1_size,
                       (int)rp_crx_value_shift);
    }

    /*
     * Phase C — revision-2 on-disk frame.
     *
     * The stream header is a one-time writer job. Every shoot-memory slot now
     * contains only a normal sector-sized VIDF. This is crucial for dynamic
     * shrink/coalescing: consecutive compressed slots can become physically
     * adjacent regardless of the fixed stream-header byte count.
     */
    if (RP_CRX_ROUNDTRIP_VERIFY && frame_no == 0u)
    {
        out_slot = rp_get_free_frame_slot();
        if (out_slot == NULL)
        {
            RP_START_TRACE("CRX out-slot STARVE f=%d slots=%d free=%d",
                           (int)frame_no, (int)rp_frame_slot_count,
                           (int)rp_count_free_frame_slots());
            rp_crx_release_handles(&result, &first_release_rc);
            rc = -1010;
            goto fail_no_handle;
        }
    }

    uint32_t slot_capacity =
        (uint32_t)(out_slot->end - out_slot->start);

    if (sub_size > 0xFFFFFFFFu - payload_bytes)
    {
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1024;
        goto fail_no_handle;
    }
    uint32_t sample_size = sub_size + payload_bytes;

    char *vidf_start = out_slot->start;
    uint32_t vidf_block_size = 0u;
    char *sample_dst = write_video_frame_header_crx_v2(
        sample_size, vidf_start, out_slot->end, &vidf_block_size);
    if (sample_dst == NULL || vidf_block_size == 0u)
    {
        RP_START_TRACE("CRX REV2 VIDF header FAIL f=%d sample=%d cap=%d",
                       (int)frame_no, (int)sample_size, (int)slot_capacity);
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1025;
        goto fail_no_handle;
    }

    uint32_t slot_used = vidf_block_size;

    /* CRX sample begins with Canon's mdat sample header (ring[0]). */
    memcpy(sample_dst, (const void *)(uintptr_t)sub_ptr, sub_size);

    /*
     * v21.4 exact-pack: keep the REV2 sample byte-contiguous while using the
     * SG capability as a true scatter/gather transaction.
     *
     * R46/W13 requires each destination base to be 64-byte aligned.  For each
     * Canon segment we therefore plan at most: 
     *
     *   head 0..62 B -> its own 64-B aligned scratch cell
     *   aligned remainder -> final sample destination
     *
     * All heads are transferred in ONE raw_vid_rp_m2m_copy_sg() call, copied
     * to their final addresses with tiny CPU memcpy, then all aligned bulk
     * pieces are transferred in ONE second SG call.  Large data never moves
     * through the CPU and the expensive native resource lifecycle is paid at
     * most twice per frame instead of up to twice per segment (40 for C-RAW).
     */
    uint8_t *sample_end = (uint8_t *)sample_dst + sample_size;
    uint8_t *payload_dst = (uint8_t *)sample_dst + sub_size;
    uintptr_t scratch_base =
        ((uintptr_t)sample_end + 63u) & ~(uintptr_t)63u;
    uint32_t head_count = 0u;
    uint32_t bulk_count = 0u;
    uint32_t planned_bytes = 0u;
    int detach_rc = 0;

    for (uint32_t i = 0; i < segment_count; i++)
    {
        uint32_t src = rp_crx_src_addr[i];
        uint32_t remaining = rp_crx_src_size[i];
        uint8_t *dst = payload_dst;

        if (((uintptr_t)dst & 1u) != 0u || (remaining & 1u) != 0u)
        {
            detach_rc = -320;
            break;
        }

        uint32_t mis = (uint32_t)((uintptr_t)dst & 63u);
        if (mis)
        {
            uint32_t head = MIN(64u - mis, remaining);
            if ((head & 1u) || head_count >= RP_CRX_MAX_SEGMENTS)
            {
                detach_rc = -321;
                break;
            }

            rp_crx_head_src[head_count] = src;
            rp_crx_head_size[head_count] = head;
            rp_crx_head_final[head_count] = dst;
            head_count++;
            src += head;
            dst += head;
            remaining -= head;
            planned_bytes += head;
        }

        if (remaining)
        {
            if (((uintptr_t)dst & 63u) != 0u ||
                bulk_count >= RP_CRX_MAX_SEGMENTS)
            {
                detach_rc = -322;
                break;
            }
            rp_crx_bulk_src[bulk_count] = src;
            rp_crx_bulk_size[bulk_count] = remaining;
            rp_crx_bulk_dst[bulk_count] = dst;
            bulk_count++;
            planned_bytes += remaining;
        }

        payload_dst += rp_crx_src_size[i];
    }

    uint32_t scratch_need = head_count * 64u;
    if (!detach_rc &&
        (planned_bytes != payload_bytes || payload_dst != sample_end))
        detach_rc = -323;
    if (!detach_rc && head_count &&
        (scratch_base < (uintptr_t)sample_end ||
         scratch_base + scratch_need > (uintptr_t)out_slot->end))
    {
        RP_START_TRACE("CRX REV2 SG scratch FAIL f=%d heads=%d need=%d sampleEnd=%d cap=%d",
                       (int)frame_no, (int)head_count, (int)scratch_need,
                       (int)((uintptr_t)sample_end - (uintptr_t)out_slot->start),
                       (int)slot_capacity);
        detach_rc = -324;
    }

    for (uint32_t i = 0; !detach_rc && i < head_count; i++)
        rp_crx_head_scratch[i] = (void *)(scratch_base + i * 64u);

    int sem_taken = 0;
    sem_rc = detach_rc ? 0 : take_semaphore_now(edmac_sem);
    if (!detach_rc && sem_rc)
    {
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1012;
        goto fail_no_handle;
    }
    if (!detach_rc)
        sem_taken = 1;

    uint32_t detach_bytes = 0;
    uint32_t detach_blocks = 0;
    uint32_t detach_us = 0;
    uint32_t compact_us = 0;

    if (!detach_rc && head_count)
    {
        uint32_t b = 0, bl = 0, us = 0;
        detach_rc = raw_vid_rp_m2m_copy_sg(
            rp_crx_head_src, rp_crx_head_size, rp_crx_head_scratch,
            head_count, &b, &bl, &us);
        detach_bytes += b;
        detach_blocks += bl;
        detach_us += us;

        if (!detach_rc)
        {
            uint32_t cpu_t0 = get_us_clock();
            for (uint32_t i = 0; i < head_count; i++)
                memcpy(rp_crx_head_final[i], rp_crx_head_scratch[i],
                       rp_crx_head_size[i]);
            compact_us += (uint32_t)(get_us_clock() - cpu_t0);
        }
    }

    if (!detach_rc && bulk_count)
    {
        uint32_t b = 0, bl = 0, us = 0;
        detach_rc = raw_vid_rp_m2m_copy_sg(
            rp_crx_bulk_src, rp_crx_bulk_size, rp_crx_bulk_dst,
            bulk_count, &b, &bl, &us);
        detach_bytes += b;
        detach_blocks += bl;
        detach_us += us;
    }

    if (sem_taken)
        give_semaphore(edmac_sem);

    if (detach_rc || detach_bytes != payload_bytes ||
        payload_dst != sample_end)
    {
        RP_START_TRACE("CRX exact-pack FAIL f=%d rc=%d seg=%d bytes=%d/%d blocks=%d us=%d dst=%d/%d",
                       (int)frame_no, detach_rc, (int)segment_count,
                       (int)detach_bytes, (int)payload_bytes,
                       (int)detach_blocks, (int)detach_us,
                       (int)((uintptr_t)payload_dst - (uintptr_t)sample_dst),
                       (int)sample_size);
        rp_crx_release_handles(&result, &first_release_rc);
        rc = -1013;
        goto fail_no_handle;
    }

    /* Canon output is now present contiguously in the exact REV2 sample. */
    release_bad = rp_crx_release_handles(&result, &first_release_rc);

    /*
     * Give unused max-slot memory forward before publishing FULL.
     * If the following region is currently busy, defer the physical shrink:
     * the on-disk block still uses only slot_used bytes and the writer simply
     * starts a new batch at the next frame.
     */
    int shrink_rc = rp_shrink_frame_slot(out_slot, slot_used);
    if (shrink_rc < 0)
    {
        RP_START_TRACE("CRX DYNSLOT shrink FAIL f=%d used=%d cap=%d rc=%d",
                       (int)frame_no, (int)slot_used,
                       (int)slot_capacity, shrink_rc);
        rp_abandon_frame_slot(out_slot);
        if (out_slot != input_slot)
            rp_abandon_frame_slot(input_slot);
        rc = -1028;
        goto fail_already_released;
    }

    out_slot->used = slot_used;
    out_slot->frame_number = frame_no;

    uint32_t frame_space =
        (uint32_t)(sample_dst - vidf_start - 0x20);
    RP_START_TRACE("CRX REV2 VIDF f=%d lossy=%d vidf=%d frameSpace=%d sample=%d shrink=%d region=%d exactPack=1 headcpy=%d",
                   (int)frame_no, lossy ? 1 : 0, (int)vidf_block_size,
                   (int)frame_space, (int)sample_size,
                   shrink_rc,
                   (int)(out_slot->end - out_slot->start),
                   (int)compact_us);

    /* Queue the stream metadata before frame 0. Both messages are FIFO and all
     * file writes occur in raw_writer. */
    if (header_needs_queue)
    {
        int header_rc = rp_queue_crx_header_write(
            header_buf, *mlv_header_size);
        if (header_rc)
        {
            RP_START_TRACE("CRX REV2 header queue FAIL rc=%d", header_rc);
            rp_abandon_frame_slot(out_slot);
            if (out_slot != input_slot)
                rp_abandon_frame_slot(input_slot);
            rc = -1029;
            goto fail_already_released;
        }
    }

    int write_rc = rp_queue_frame_slot_write(out_slot);
    if (write_rc)
    {
        rp_abandon_frame_slot(out_slot);
        if (out_slot != input_slot)
            rp_abandon_frame_slot(input_slot);
        rc = -1014;
        goto fail_already_released;
    }

    if (RP_CRX_ROUNDTRIP_VERIFY && frame_no == 0u)
    {
        int ref_rc = rp_hold_crx_reference(
            input_slot, (uint32_t)event->w, (uint32_t)event->h, bits,
            frame_no, input_bytes);
        if (!ref_rc && use_direct_w3)
            rp_crx_ref_copy_us = ref_copy_us;
        if (ref_rc)
        {
            RP_START_TRACE("CRX REF hold FAIL f=%d rc=%d",
                           (int)frame_no, ref_rc);
            /* The revision-2 VIDF is already queued. Stop cleanly after preserving
             * the file, but do not attempt another frame in this verification build. */
            rc = -1015;
            raw_vid_rp_request_stop();
            raw_vid_request_stop_async();
            return rc;
        }
    }

    (*video_frame_count)++;
    rp_crx_frames_ok++;
    rp_crx_total_detach_us += detach_us;
    rp_crx_total_compact_us += compact_us;
    rp_crx_total_payload_bytes += sample_size;
    rp_crx_total_m2m_blocks += detach_blocks;

    /* Diagnostic only. Scale to KiB before multiplying so this stays in
     * cheap 32-bit arithmetic even for the largest offered RP frames. */
    uint32_t raw_kib = input_bytes >> 10;
    uint32_t sample_kib = sample_size >> 10;
    uint32_t ratio_permille = raw_kib
                            ? (sample_kib * 1000u) / raw_kib
                            : 0u;
    RP_START_TRACE("CRX frame PASS f=%d path=%s fmt=REV2/VIDF bpp=%d raw=%d sample=%d ratio=%d/1000 in=%d ref=%d enc=%d det=%d compact=%d seg=%d vidf=%d used=%d rel=%x",
                   (int)frame_no, use_direct_w3 ? "W3" : "M2M",
                   (int)bits, (int)input_bytes,
                   (int)sample_size, (int)ratio_permille,
                   (int)input_us, (int)ref_copy_us,
                   (int)result.encode_us,
                   (int)detach_us, (int)compact_us,
                   (int)segment_count, (int)vidf_block_size,
                   (int)slot_used, first_release_rc);

    if (release_bad)
    {
        RP_START_TRACE("CRX release FAIL f=%d rc=%x; stopping after saved frame",
                       (int)frame_no, first_release_rc);
        raw_vid_rp_request_stop();
        raw_vid_request_stop_async();
    }
    return 0;

fail_already_released:
    rp_crx_frames_failed++;
    raw_vid_rp_request_stop();
    raw_vid_request_stop_async();
    return rc;

fail_no_handle:
    if (out_slot && out_slot != input_slot)
        rp_abandon_frame_slot(out_slot);
    if (input_slot)
        rp_abandon_frame_slot(input_slot);
    rp_crx_frames_failed++;
    RP_START_TRACE("CRX frame FAIL f=%d rc=%d", (int)frame_no, rc);
    raw_vid_rp_request_stop();
    raw_vid_request_stop_async();
    return rc;
}
static void worker(void)
{
    #ifdef RAW_VID_LOG
    uint32_t buf_size = 100;
    char log_buf[buf_size];
    uint32_t start_time = 0;
    uint32_t session_data_size = 0;
    #endif

    char mlv_name[20] = {0};

    struct crop_buf crop_bufs[2] = {NULL};
    struct crop_buf *crop_buf = NULL;
    uint32_t video_frame_count = 0;

    char *header_buf = NULL; // space for MLV file header
    const uint32_t header_buf_size = 1024; // wants to be big enough for combined headers,
                                           // this file shouldn't know their size, it's okay to just guess.
    uint32_t header_size = 0; // MLV lib returns real size here
    header_buf = malloc(header_buf_size);
    if (header_buf == NULL)
    {
        SEND_LOG_DATA_STR("ERROR: couldn't malloc for header_buf\n");
        return; // worker task dies; raw video recording becomes impossible
    }

    TASK_LOOP
    {
        int err = -1;
        struct raw_vid_event *event = NULL;
        msg_queue_receive(event_q, &event, 0);

        if (event == NULL)
        { // shouldn't happen, we only post valid events
            SEND_LOG_DATA_STR("ERROR: worker, event was NULL\n");
            continue;
        }

        switch (event->type)
        {
        case IMAGE_DATA:
            if (recording_state != ACTIVE)
            {
                SEND_LOG_DATA_STR("ERROR: trying to process frame while recording not ACTIVE\n");
                break;
            }

            if (event->src == NULL)
            {
                SEND_LOG_DATA_STR("ERROR: event->src was NULL\n");
                break;
            }

            if (raw_vid_fp == NULL)
            {
                SEND_LOG_DATA_STR("ERROR: fp was NULL at attempt to write\n");
                break;
            }

            #ifdef RAW_VID_LOG
            snprintf(log_buf, buf_size, "Image prod time, cons time, size: %d, %d, %d\n",
                     event->creation_time, get_ms_clock(), event->size);
            send_log_data_str(log_buf);
            #endif

            if (crop_buf == NULL)
            {
                // FIXME we should stop here
                SEND_LOG_DATA_STR("ERROR: crop_buf was NULL at dst check\n");
                break;
            }

            if (event->size > crop_buf->end - crop_buf->start)
            {
                // For simplicity, alloc_crop_bufs() ensures the bufs are equal size.
                // Thus if we hit this, we can't ever save the data, there is no point
                // swapping bufs.  We must give up.
                // FIXME we don't yet handle stopping except via user pressing Rec 2nd time
                // (when added we need to audit for other places).
                SEND_LOG_DATA_STR("ERROR: event size too large for empty crop buf\n");
                break;
            }

            // Here we have src data, and an output file.
            // We want to do a (potentially cropped) copy from
            // src to dst.  We accumulate frames into a larger buffer
            // before triggering flush to disk.

            char *dst = NULL;
            struct crop_buf *flush_src = NULL;
            // write the header of a vidf block, which returns us an aligned
            // location to write the image data.
            dst = write_video_frame_header(event->size, crop_buf->cur, crop_buf->end);
            if (dst == NULL)
            {
                // The prior IMAGE_DATA was the last that fit in this buffer,
                // we must swap.  And we can trigger flush to disk of prior in use buffer.
                #ifdef RAW_VID_LOG
                snprintf(log_buf, buf_size, "Swapping bufs, time: %d\n", get_ms_clock());
                send_log_data_str(log_buf);
                #endif
                
                flush_src = crop_buf; // this triggers later flush to disk

                if (crop_buf == &crop_bufs[0])
                    crop_buf = &crop_bufs[1];
                else
                    crop_buf = &crop_bufs[0];

                if (crop_buf->cur != crop_buf->start)
                {
                    // raw_write() can reset these, presumably we're writing too fast
                    // so it's not finished.
                    SEND_LOG_DATA_STR("After buf swap, buf was in use; not written out\n");
                    crop_buf = NULL; // Hack, prevents next IMAGE_DATA doing anything,
                                     // really we should flush the remaining?
                    break; // FIXME we should stop as well
                }

                dst = write_video_frame_header(event->size, crop_buf->cur, crop_buf->end);
                if (dst == NULL)
                {
                    SEND_LOG_DATA_STR("ERROR: couldn't fit image in either buffer\n");
                    break;
                }
            }
            #ifdef RAW_VID_LOG
            session_data_size += (dst - crop_buf->cur + event->size);
            #endif
            crop_buf->cur = dst + event->size; // cur after image data, which is not yet written
            video_frame_count++;

            err = take_semaphore_now(edmac_sem);
            if (err)
            {
                // If this happens, it means we're not managing to save a frame
                // before the next vsync event in event_pusher.c tries to capture one.

                SEND_LOG_DATA_STR("ERROR: worker couldn't take edmac_sem\n");
                // FIXME this break means we're choosing to drop frames
                // rather than stop recording, I think just stopping is better.
                // Maybe re-imp the option from mlv_lite.
                break;
            }

            // here we know the copy rect from the prior IMAGE_DATA has completed,
            // as we have edmac_sem, so we can safely flush if one of the buffers was full.
            if (flush_src != NULL)
            {
                task_create("raw_write", WORKER_WRITE_PRIO, 0x800,
                            write_buf_to_disk, flush_src);
            }

            int line_size = event->w * BPP/8; // width in bytes
            #ifdef RAW_VID_LOG
            snprintf(log_buf, buf_size, "w, h, sz: %d, %d, %d\n",
                     event->w, event->h, line_size * event->h);
            send_log_data_str(log_buf);
            #endif

            // FIXME:
            // We should stop using edmac_copy() directly.
            // - Not all cams have it.
            // - It has alignment requirements we can hide by having a small
            //   unaligned copy with normal memcpy at the start,
            //   simplifying MLV file format (and removing need to align buffers!).
            // - This high-level code shouldn't care *how* copies occur,
            //   only that they do.
            //
            // The extra cost for doing a memcpy() to align should be tiny,
            // alignment requirement is 0x10 on Digic 5.

            edmac_copy_rectangle_cbr_start(
                (void*)dst, event->src,
                raw_info.pitch, (event->x + 7)/8*BPP, event->y/2*2,
                line_size, 0, 0,
                line_size, event->h,
                &edmac_cbr_r, &edmac_cbr_w, NULL
            );
            // edmac_cbr_w() will release edmac sem, when copy complete

            break;

        case COMMAND_START:
            SEND_LOG_DATA_STR("Worker got start request.\n");
            if (event->w == 0 || event->h == 0)
            {
                SEND_LOG_DATA_STR("ERROR: start request with 0 area\n");
                break;
            }

            if (recording_state == INACTIVE)
            {
                // Get mem to buffer our cropped frames.  We do this because 
                // card write speeds are improved if we pass only large, size aligned
                // buffers to FIO_WriteFile().  Optimal size and alignment probably
                // varies by cam, card (manufacturer, controller, etc), card format etc.
                //
                // We double buffer so we can record into one as we write out the other.
                // At this point, we don't know the required size as no image data has
                // been sent.
                if (crop_buf != NULL)
                {
                    SEND_LOG_DATA_STR("ERROR: crop_buf not null on COMMAND_START\n");
                    break;
                }
                alloc_crop_bufs(crop_bufs);
                if (crop_bufs[0].start == NULL
                    || crop_bufs[1].start == NULL)
                {
                    SEND_LOG_DATA_STR("ERROR: could not obtain crop bufs, cannot start\n");
                    break;
                }
                #ifdef RAW_VID_LOG
                snprintf(log_buf, buf_size, "buf[0] and buf[1] size: %d, %d\n",
                         crop_bufs[0].end - crop_bufs[0].cur,
                         crop_bufs[1].end - crop_bufs[1].cur);
                send_log_data_str(log_buf);
                #endif
                crop_buf = &crop_bufs[0];

                // Set LV to raw
                raw_lv_request();

                // Get new save file
                if (raw_vid_fp != NULL)
                {
                    SEND_LOG_DATA_STR("ERROR: Worker tried to init save file but already in use\n");
                    break;
                }

                if (get_numbered_file_name("tst%02d.mlv", 99, mlv_name, sizeof(mlv_name)) > -1)
                {
                    raw_vid_fp = FIO_CreateFile(mlv_name);
                    if (raw_vid_fp == NULL)
                    {
                        SEND_LOG_DATA_STR("Worker couldn't create mlv file\n");
                        break;
                    }
                }
                else
                {
                    SEND_LOG_DATA_STR("ERROR: Worker couldn't get mlv file name\n");
                    break;
                }

                // FIXME from here onwards we want to cleanup the file if we error.
                // Really, we need to revisit error conditions in all states,
                // and ensure we handle correctly (e.g. flushing pending data).

                session.fps = fps_get_current_x1000();
                session.res_x = event->w;
                session.res_y = event->h;

                err = start_mlv_session(&session);
                if (err)
                {
                    SEND_LOG_DATA_STR("ERROR: failed to start mlv session\n");
                    break;
                }

                // write standard header blocks
                header_size = write_file_headers(header_buf, header_buf + header_buf_size);
                if (header_size == 0)
                {
                    SEND_LOG_DATA_STR("ERROR: failed to write mlv headers\n");
                    break;
                }
                FIO_WriteFile(raw_vid_fp, header_buf, header_size);

                #ifdef RAW_VID_LOG
                send_log_data_str("Recording is active\n");
                start_time = get_ms_clock();
                #endif
                recording_state = ACTIVE;
            }
            break;

        case COMMAND_STOP:
            SEND_LOG_DATA_STR("Worker got stop request.\n");
            // FIXME how to stop is more complicated than this,
            // need to flush, reset some pointers, block new start
            // until finished.
            if (recording_state == ACTIVE)
            {
                // restore LV away from raw
                // This ML asserts if you haven't paired it with raw_lv_request(),
                // but is safe.
                raw_lv_release();
                // TODO mlv_lite does restore_bit_depth(), we
                // don't mess with this yet.

                #ifdef RAW_VID_LOG
                snprintf(log_buf, buf_size, "session time, data written: %d, %d\n",
                         get_ms_clock() - start_time, session_data_size);
                send_log_data_str(log_buf);
                session_data_size = 0;
                #endif

                // FIXME we want to handle any potential flushing before freeing stuff,
                // we may need buffers for dealing with queued frames.
                crop_buf = NULL;
                if (crop_bufs[0]._buf != NULL)
                {
                    fio_free(crop_bufs[0]._buf);
                    crop_bufs[0]._buf = NULL;
                    crop_bufs[0].start = NULL;
                    crop_bufs[0].cur = NULL;
                    crop_bufs[0].end = NULL;
                }
                if (crop_bufs[1]._buf != NULL)
                {
                    fio_free(crop_bufs[1]._buf);
                    crop_bufs[1]._buf = NULL;
                    crop_bufs[1].start = NULL;
                    crop_bufs[1].cur = NULL;
                    crop_bufs[1].end = NULL;
                }

                if (raw_vid_fp != NULL)
                {
                    // TODO flush pending data

                    // update with the final video frame count
                    err = update_video_frame_count(video_frame_count, header_buf, header_buf + header_size);
                    if (err)
                    {
                        SEND_LOG_DATA_STR("ERROR: couldn't update video frame count\n");
                        break;
                    }

                    // write back; file is now complete
                    FIO_SeekSkipFile(raw_vid_fp, 0, SEEK_SET);
                    FIO_WriteFile(raw_vid_fp, header_buf, header_size);

                    FIO_CloseFile(raw_vid_fp);
                    raw_vid_fp = NULL;
                }
                else
                {
                    SEND_LOG_DATA_STR("ERROR: unexpected fp state in worker\n");
                }

                stop_mlv_session();
                recording_state = INACTIVE;
                SEND_LOG_DATA_STR("Recording is inactive\n");
            }
            break;

        case EMPTY:
        case PREPARING:
            // These types should never be sent.  We will need to fix the cause.
            // But we still want to release the event storage.
            SEND_LOG_DATA_STR("ERROR: unexpected event type\n");
            break;
        }

        // Note that we must reach here from every case above,
        // or we will leak events and starve the queue.
        release_event(event);

        // no sleep needed, queue receive with timeout 0 is blocking.
    }

    free(header_buf);
    SEND_LOG_DATA_STR("Raw video worker stopping.\n");
}

void init_worker(struct msg_queue *q)
{
    event_q = q;
    edmac_sem = create_named_semaphore("edmac_raw_worker", SEM_CREATE_UNLOCKED);
    if (edmac_sem == NULL)
    {
        SEND_LOG_DATA_STR("ERROR: worker could not create edmac sem\n");
        return;
    }
    task_create("raw_v_worker", WORKER_PRIO, 0x1000, worker, NULL);
    allow_recording();
}