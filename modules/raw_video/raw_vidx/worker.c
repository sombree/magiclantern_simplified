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