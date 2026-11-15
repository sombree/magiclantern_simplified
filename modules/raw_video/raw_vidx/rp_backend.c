/*
 * EOS RP 1.6.0 RAW-video capture backend for raw_vidx.
 *
 * v23 production capture model (runtime-proven by rpbppextent v178/v182/v183.4):
 *   - 10/12-bit pitch is produced without SetSize patching by changing the
 *     writable Mem1 source width plus cfg.b2.  Canon's own SetSize mask
 *     0x7ffffffe turns 4K LE12 raw pitch 5593 into physical pitch 5592.
 *   - destinations are selected through the class-2 MemoryNavigator
 *     descriptor offset (count remains 1; stride/indices remain Canon-owned).
 *   - the next desc.offset is written only after provider->Evf->Mem1->W3 has
 *     latched the current destination, i.e. the timing proven by v183.4.
 *   - ch3 Complete remains Canon-owned; ML replaces only Canon's BX-LR table
 *     callback and classifies an ML completion by exact W3+EC == dma+size.
 *
 * Runtime ownership model:
 *   - RAW write EDMAC is channel 3, pBlock 0xD0420200.
 *   - class-2 MemoryNavigator low-id 7 owns the destination descriptor.
 *   - a destination is considered latched only when provider slot 0xF168,
 *     EvfDevVram+0xEC, Mem1 current 0x183F8 and W3+A0 all agree.
 *   - only after that four-way latch may desc.offset be changed for the next
 *     frame.  This is the continuous timing proven by v183.4 in 4K.
 *   - Canon already owns channel-3 Complete IRQ 0x91.  ML changes only the
 *     existing callback-table word at 0x00076E1C and never registers the IRQ.
 *   - Complete does not steer ownership.  It marks READY only when W3+EC
 *     exactly equals an already-latched ML slot end (dma + physical size).
 *
 * Normal REC never patches executable firmware code.
 */

#include "module.h"
#include "dryos.h"
#include "raw.h"
#include "edmac.h"
#include "mem_defs.h"
#include "rp_cache_ops.h"

#include "raw_vid.h"
#include "raw_vid_event_pusher.h"
#include "raw_vid_worker.h"
#include "rp_backend.h"
#include "rp_hot_shared.h"

#include <stdint.h>

#define RP_RAW_CHANNEL                  3u
#define RP_RAW_PBLOCK                   0xD0420200u
#define RP_COMPLETE_TABLE               0x00076E04u
#define RP_COMPLETE_SLOT                (RP_COMPLETE_TABLE + RP_RAW_CHANNEL * 8u)
#define RP_COMPLETE_ARG_SLOT            (RP_COMPLETE_SLOT + 4u)
#define RP_CANON_COMPLETE_NOP_THUMB     0xE037951Du

/* RP 1.6.0 Mem1Path PackUnpack provenance:
 * E0379A16 calls E057D290(3, *(u32*)E0379C3C + 0x64).
 * E057D290 consumes five low-byte fields at +0,+4,+8,+C,+10; field #2 is
 * native RAW bit depth (2=14b, 1=12b, 0=10b), matching the D8/M50 method. */
#define RP_MEM1_STATE_PTR_LITERAL       0xE0379C3Cu
#define RP_MEM1_PACK_CFG_OFF            0x64u
#define RP_MEM1_PACK_BPP_OFF            0x08u
#define RP_MEM1_PACK_B3_OFF             0x0Cu
#define RP_MEM1_PACK_BPP_FIELD_OFF      (RP_MEM1_PACK_CFG_OFF + RP_MEM1_PACK_BPP_OFF)
#define RP_MEM1_PACK_B3_FIELD_OFF       (RP_MEM1_PACK_CFG_OFF + RP_MEM1_PACK_B3_OFF)

/* v178/v182/v183.4 RAM-only source-width + MNAV steering ABI. */
#define RP_MEM1_MODE_OFF                0x00u
#define RP_MEM1_SELECTED_W_OFF          0x10u
#define RP_MEM1_SELECTED_H_OFF          0x14u
#define RP_MEM1_A_W_OFF                 0x18u
#define RP_MEM1_A_H_OFF                 0x1Cu
#define RP_MEM1_B_W_OFF                 0x20u
#define RP_MEM1_B_H_OFF                 0x24u
#define RP_MEM1_B_GATE_OFF              0x28u
#define RP_MEM1_C_W_OFF                 0x2Cu
#define RP_MEM1_C_H_OFF                 0x30u
#define RP_MEM1_D_W_OFF                 0x34u
#define RP_MEM1_D_H_OFF                 0x38u
#define RP_MEM1_D_GATE_OFF              0x3Cu

#define RP_EVFDEV_VRAM_BASE_LITERAL     0xE00DE01Cu
#define RP_MEM1_CURRENT_BUFFER_SLOT     0x000183F8u
#define RP_MNAV_CONTROL                 0x00008A20u
#define RP_MNAV_PROVIDER_SLOT           0x0000F168u
#define RP_MNAV_LOW_ID                  7u
#define RP_SETSIZE_XB_MASK              0x7FFFFFFEu
#define RP_STEER_POLL_MS                1u
#define RP_STEER_LATCH_TIMEOUT_US       500000u

/* v148/v149 causal proof on the same Movie RAW-LV Mem1Path/W3/PU2:
 *
 *   b3 low byte 0 / PU2 bit0 0 -> Canon non-linear packed family
 *   b3 low byte 1 / PU2 bit0 1 -> contiguous little-endian packed samples
 *
 * Proven at 14, 12 and 10 bit.
 */
#define RP_PU2_PBLOCK                   0xD0422200u
#define RP_PU_MODE_OFF                  0x24u
#define RP_PU_BIT0                      0x00000001u

/* Canon Mem1 read-only accessors recovered for RP 1.6.0. */
#define RP_RAW_GET_BUFFER_THUMB         0xE0379B91u
#define RP_RAW_GET_SIZE_THUMB           0xE0379B7Fu

/* RP 1.6.0 Canon Engine::MemoryToMemoryEsub1 path, recovered statically from
 * EFsVcopy / E0738BAC and output545.  These are RP-native values; notably the
 * Boomer selector differs from the M50/R value copied by the older D8 patch. */
#define RP_M2M_RD_CH                     46u
#define RP_M2M_WR_CH                     13u
#define RP_M2M_RD_PBLOCK                 0xD0421600u
#define RP_M2M_WR_PBLOCK                 0xD0420C00u
#define RP_M2M_BOOMER_SELECTOR           0x3F7F0126u
#define RP_M2M_BYTES                     4096u
#define RP_M2M_ALIGN                     0x40u
#define RP_M2M_TIMEOUT_US                100000u

/* Synthetic 2D/stride probe matching the current 4K RAW crop geometry.
 * x=192 px at packed 14-bit -> 336 byte horizontal offset.
 * y=100 is the current top margin used by raw_vidx.
 * One copied row is 1280 px * 14/8 = 2240 bytes. */
#define RP_M2M_RECT_SRC_PITCH            6524u
#define RP_M2M_RECT_XOFF                 336u
#define RP_M2M_RECT_YOFF                 100u
#define RP_M2M_RECT_ROW_BYTES            2240u
#define RP_M2M_RECT_ROWS                 720u
#define RP_M2M_RECT_DST_BYTES            (RP_M2M_RECT_ROW_BYTES * RP_M2M_RECT_ROWS)
#define RP_M2M_RECT_SRC_SPAN             ((RP_M2M_RECT_YOFF + RP_M2M_RECT_ROWS) * RP_M2M_RECT_SRC_PITCH + RP_M2M_RECT_XOFF)
#define RP_M2M_RECT_GUARD                256u
#define RP_M2M_RECT_TIMEOUT_US           250000u

/* v11k2 diagnostic matrix.  Keep row width fixed and vary only row count and
 * whether a source stride gap exists.  This directly separates:
 *   - yb > 0 / multi-row operation,
 *   - total transfer length,
 *   - off1b != 0 / source-stride skipping.
 *
 * Stages 26/28/30: tightly packed source, pitch == row bytes, off1b == 0.
 * Stages 27/29/31: RAW-like source pitch 6524, off1b == 4284.
 * x/y offsets are deliberately zero for every matrix case. */
#define RP_M2M_MATRIX_ROW_BYTES          2240u
#define RP_M2M_MATRIX_GAP_PITCH          6524u
#define RP_M2M_MATRIX_GUARD              256u
#define RP_M2M_MATRIX_TIMEOUT_US         250000u
#define RP_M2M_VERIFY_YIELD_ROWS         8u

#define RP_FN_RES_CREATE                 0xE0726704u
#define RP_FN_RES_LOCK                   0xE07267ACu
#define RP_FN_RES_UNLOCK                 0xE07267B2u
#define RP_FN_RES_DESTROY                0xE072672Eu
#define RP_FN_EDMAC_RESET                0xE057B978u
#define RP_FN_EDMAC_FINAL_START_RD       0xE057B93Eu
#define RP_FN_EDMAC_REGISTER_COMPLETE    0xE057BA62u
#define RP_FN_EDMAC_UNREGISTER_COMPLETE  0xE057BDAAu
#define RP_FN_BOOMER_RESET               0xE057BDF8u
#define RP_FN_BOOMER_SELECT              0xE057BE70u
#define RP_FN_EDMAC_STOP_BOOMER          0xE057BF04u
#define RP_FN_EDMAC_RESET_PACK           0xE057BF48u
#define RP_FN_EDMAC_ARM                  0xE057C17Cu
#define RP_FN_EDMAC_SET_ADDRESS          0xE057C358u
#define RP_FN_EDMAC_SET_SIZE             0xE057C364u
#define RP_FN_EDMAC_SET_MODE             0xE057C7CAu

#define RP_THUMB_FN(addr, type) ((type)(uintptr_t)((addr) | 1u))

/* Three full 4K RAW frames are ~40.2 MB.  Three are enough for the first
 * synchronous-CPU-crop recorder and materially reduce pressure on shoot RAM. */
#define RP_CAPTURE_ALIGN                0x40u
#define RP_CAPTURE_ALLOC_PAD            0x100u
#define RP_STOP_TIMEOUT_US              500000u

#define RP_REPORT_PATH                  "RPVIDX.TXT"
#define RP_BACKEND_VERSION              "v23.1-prod-clean-ram-mnav"

#define RP_BACKEND_TRACE(fmt, ...) \
    DryosDebugMsg(0, 15, "RPVIDX B " fmt, ##__VA_ARGS__)

static struct msg_queue *rp_event_q = NULL;
static struct semaphore *rp_post_sem = NULL;
static volatile uint32_t rp_backend_initialized = 0;
static volatile uint32_t rp_ready_task_started = 0;
static volatile uint32_t rp_queued_count = 0;
static volatile uint32_t rp_released_count = 0;
static volatile uint32_t rp_post_fail = 0;
static volatile uint32_t rp_stop_task_scheduled = 0;


/* One-shot mem2mem probe completion flags.  Canon invokes these from EDMAC
 * Complete context, so callbacks are deliberately RAM-store-only. */
static volatile uint32_t rp_m2m_rd_done = 0;
static volatile uint32_t rp_m2m_wr_done = 0;

/* Stage-33 persistent mem2mem session state.  Match the M50 lifecycle:
 * resource lock + subchip wake stay live for the recording session, while
 * Boomer/CBRs/channel programming remain per-copy. */
static void *rp_m2m_session_lock = NULL;
static volatile uint32_t rp_m2m_session_active = 0;
static uint32_t rp_m2m_session_frames = 0;
static uint32_t rp_m2m_session_errors = 0;
static uint32_t rp_m2m_session_sum_us = 0;
static uint32_t rp_m2m_session_min_us = 0;
static uint32_t rp_m2m_session_max_us = 0;

struct rp_mem1_geometry
{
    uint32_t state, mode, b_gate, d_gate;
    uint32_t a_w, a_h, b_w, b_h, c_w, c_h, d_w, d_h;
    uint32_t selected_w, selected_h;
    uint32_t active_pair, active_w_off, active_h_off, active_w, active_h;
};

struct rp_mnav_snapshot
{
    uint32_t state, worktype, flag2, desc, pool;
    uint32_t offset, stride, count;
    int32_t read_idx, write_idx;
    uint32_t pool_base, predicted_write;
};

static struct rp_mem1_geometry rp_mem1_saved_geom;
static uint32_t rp_mem1_saved_geom_valid = 0;
static uint32_t rp_fake_width = 0;
static uint32_t rp_expected_getsize = 0;
static uint32_t rp_expected_hw_geom = 0;
static uint32_t rp_expected_pu2_mode = 0;

static struct rp_mnav_snapshot rp_mnav;
static uint32_t rp_mnav_valid = 0;
static uint32_t rp_mnav_desc_original[4];
static int32_t rp_mnav_read_original = 0;
static int32_t rp_mnav_write_original = 0;
static volatile uint32_t rp_steer_task_started = 0;
static volatile uint32_t rp_steer_active = 0;
static volatile uint32_t rp_steer_expected_dma = 0;
static volatile int rp_steer_expected_slot = -2; /* -1 Canon, >=0 ML, -2 none */
static volatile int rp_steer_current_slot = -1;
static volatile uint32_t rp_steer_latch_count = 0;
static volatile uint32_t rp_steer_canon_frames = 0;
static volatile uint32_t rp_steer_no_free = 0;
static volatile uint32_t rp_steer_desc_writes = 0;
static volatile uint32_t rp_steer_last_latch_us = 0;

static uint32_t rp_crop_x = RP_RAW_VID_CROP_X;
static uint32_t rp_crop_y = RP_RAW_VID_CROP_Y;
static uint32_t rp_crop_w = 0;
static uint32_t rp_crop_h = 0;

static uint32_t rp_saved_complete_fn = 0;
static uint32_t rp_saved_complete_arg = 0;
static uint32_t rp_tap_ptr = 0;
static volatile uint32_t rp_tap_installed = 0;

static volatile uint32_t *rp_pack_bpp_field = NULL;
static uint32_t rp_pack_bpp_saved = 0;
static uint32_t rp_pack_bpp_saved_valid = 0;
static int rp_saved_raw_bpp = 14;
static int rp_saved_raw_pitch = 0;
static int rp_saved_raw_frame_size = 0;
static int rp_bit_depth_active = 0;
static uint32_t rp_saved_pu2_mode = 0;
static int rp_matrix_applied = 0; /* tracks restored b0/b1/b4 state */

/* v148-proven Mem1 linear-14 selector state.  CRX uses this only at 14 bpp. */
static volatile uint32_t *rp_pack_b3_field = NULL;
static uint32_t rp_pack_b3_saved = 0;
static uint32_t rp_linear_baseline_pu2 = 0;
static uint32_t rp_linear_packing_active = 0;

/* Frozen recording geometry for the post-stop report.  raw_info is restored
 * to Canon's 14-bit state before RPVIDX.TXT is written. */
static int rp_report_bpp = 14;
static int rp_report_pitch = 0;
static int rp_report_frame_size = 0;
static uint32_t rp_report_fake_width = 0;
static uint32_t rp_report_getsize = 0;
static uint32_t rp_report_hw_geom = 0;
static uint32_t rp_report_pu2_mode = 0;
static uint32_t rp_capture_alloc_bytes = 0;
/* Bytes actually described by the W3 DMA descriptor for the CURRENT frame,
 * independent of the sensor's sample precision. Populated at prepare and
 * zeroed at release; not the speculative full LE16 target in WIREPLAN. */
static uint32_t rp_capture_wire_payload_bytes = 0;
static uint32_t rp_capture_wire_pitch = 0;
uint32_t raw_vid_rp_wire_payload_bytes(void) { return rp_capture_wire_payload_bytes; }
uint32_t raw_vid_rp_wire_pitch_bytes(void) { return rp_capture_wire_pitch; }
/* Guard helpers below use rp_dmb() before its implementation.
 * Keep the signature visible to strict GNU99 builds (-Werror). */
static inline void rp_dmb(void);

/* v5: guarded b0=1 source-ring experiment. This explicitly overallocates
 * for hypothetical 16-bit/pixel W3 output; the normal ML event payload and
 * MLV pitch remain unchanged. Never assumes that 16-bit write is proven. */
#define RP_W3_GUARD_EXTRA_BYTES 4096u
#define RP_W3_GUARD_PATTERN 0xA7u
#define RP_W3_GUARD_CHECK_BYTES 64u
#define RP_W3_GUARD_POST16_BYTES 1024u
static struct rp_w3_guard_status rp_guard;
static int rp_guard_requested(void)
{
    return rp_w3_matrix_case_active() &&
         ((rp_w3_matrix_field_index() == 0 &&
           rp_w3_matrix_field_value() == 1) ||
          /* Conservative over-allocation for uncharacterized b4 output.
           * DOES NOT imply that b4 selects LE16 or that 16-bit is a hard
           * upper bound for undocumented firmware behavior. */
          (rp_w3_matrix_field_index() == 4 &&
           (rp_w3_matrix_field_value() == 3 ||
            (rp_w3_matrix_field_value() == 2 &&
             (raw_info.bits_per_pixel == 12 || raw_info.bits_per_pixel == 10)) ||
            (rp_w3_matrix_field_value() == 1 &&
             raw_info.bits_per_pixel == 10))));
}

static int rp_capture_allocation_size(uint32_t *alloc_size)
{
    uint32_t nominal = (uint32_t)raw_info.frame_size;
    if (!nominal || nominal > 0x7FFFFFFFu) return -1;
    uint64_t bytes = (uint64_t)nominal + RP_CAPTURE_ALLOC_PAD;
    if (rp_guard_requested())
    {
        struct rp_w3_transport_plan plan;
        if (rp_w3_transport_make_plan(&plan, (uint32_t)raw_info.width,
              (uint32_t)raw_info.height, (uint32_t)raw_info.bits_per_pixel,
              1, (uint32_t)raw_info.pitch, 0u)) return -2;
        if (plan.nominal_bytes != nominal ||
            plan.target_bytes < plan.nominal_bytes) return -2;
        bytes = (uint64_t)plan.target_bytes + RP_CAPTURE_ALLOC_PAD +
                RP_W3_GUARD_EXTRA_BYTES;
        rp_guard.enabled = 1;
        rp_guard.nominal = nominal;
        rp_guard.nominal16 = plan.target_bytes;
    }
    if (bytes > 0x7FFFFFFFu) return -3;
    *alloc_size = (uint32_t)bytes;
    rp_guard.allocated_per_slot = (uint32_t)bytes;
    return 0;
}

/* Only during capture preparation, before redirecting ch3. All accesses
 * use the uncached DMA alias, so the guard is not hidden in the CPU cache. */
static int rp_guard_seed_slot(int i)
{
    if (!rp_guard.enabled) return 0;
    if (i < 0 || i >= rp_buf_count) return -1;
    uint8_t *dma = (uint8_t *)(uintptr_t)rp_bufs[i].dma;
    if (!dma || rp_guard.nominal16 < rp_guard.nominal) return -2;
    uint64_t end = (uint64_t)rp_guard.nominal16 +
                   RP_W3_GUARD_POST16_BYTES + RP_W3_GUARD_CHECK_BYTES;
    /* The +256 pad also accommodates up to 63 bytes of alignment shift. */
    if (end + RP_CAPTURE_ALIGN > rp_guard.allocated_per_slot) return -3;
    memset(dma + rp_guard.nominal, RP_W3_GUARD_PATTERN,
           (uint32_t)end - rp_guard.nominal);
    rp_dmb();
    return 0;
}

static int rp_guard_probe_slot(int slot)
{
    if (!rp_guard.enabled) return 0;
    if (slot < 0 || slot >= rp_buf_count) return -1;
    const volatile uint8_t *dma =
        (const volatile uint8_t *)(uintptr_t)rp_bufs[slot].dma;
    int nominal_changed=0, full16_changed=0, after16_changed=0;
    for (uint32_t k=0;k<RP_W3_GUARD_CHECK_BYTES;k++)
    {
        if (dma[rp_guard.nominal+k] != RP_W3_GUARD_PATTERN) nominal_changed=1;
        if (dma[rp_guard.nominal16+k] != RP_W3_GUARD_PATTERN) full16_changed=1;
        if (dma[rp_guard.nominal16+RP_W3_GUARD_POST16_BYTES+k] !=
            RP_W3_GUARD_PATTERN) after16_changed=1;
    }
    rp_guard.checked++;
    if (nominal_changed)
    {
        if (!rp_guard.nominal_changed)
            rp_guard.first_nominal_slot=(uint32_t)slot;
        rp_guard.nominal_changed++;
    }
    if (full16_changed) rp_guard.full16_changed++;
    if (after16_changed)
    {
        if (!rp_guard.after16_changed)
            rp_guard.first_after16_slot=(uint32_t)slot;
        rp_guard.after16_changed++;
        /* Writes even after hypothetical full16 payload are a hard abort.
         * Note: cannot recover an already-overrun memory transaction. */
        rp_fatal=1;
        rp_dmb();
        return -2;
    }
    return 0;
}

int raw_vid_rp_transport_get_plan(struct rp_w3_transport_plan *out)
{
    if (!out) return -1;
    if (!is_camera("RP", "1.6.0") || raw_info.width <= 0 ||
        raw_info.height <= 0 || raw_info.pitch <= 0)
        return -2;
    /* For the current matrix the only confirmed LE16 mode is b0=1.
     * Actual W3 pitch remains packed: THIS FUNCTION NEVER WRITES MMIO. */
    const int le16 = rp_w3_matrix_case_active() &&
        rp_w3_matrix_field_index() == 0 &&
        rp_w3_matrix_field_value() == 1;
    uint32_t usable = 0u;
    /* v6.1: report the actual minimum usable capacity of all redirected
     * allocations, measured from each 64-byte-aligned DMA start. The old
     * formula subtracted BOTH the 256-byte allocation pad and 64-byte
     * alignment, which incorrectly reported nominal_size - 64 despite
     * successful allocations of nominal_size + 256.
     *
     * Do not count unallocated slots, never infer capacity from raw_info,
     * and do not confuse this capacity with the hardware DMA byte count. */
    if (rp_buf_count == RP_CAPTURE_BUFS && rp_guard.allocated_per_slot > 0u)
    {
        uint32_t min_usable = 0xffffffffu;
        for (int i = 0; i < RP_CAPTURE_BUFS; i++)
        {
            if (!rp_bufs[i].alloc || !rp_bufs[i].dma)
            {
                min_usable = 0u;
                break;
            }
            const uintptr_t base = (uintptr_t)rp_bufs[i].alloc;
            const uint32_t alignment_offset =
                (RP_CAPTURE_ALIGN - (uint32_t)(base & (RP_CAPTURE_ALIGN - 1u)))
                 & (RP_CAPTURE_ALIGN - 1u);
            if (alignment_offset >= rp_guard.allocated_per_slot)
            {
                min_usable = 0u;
                break;
            }
            const uint32_t slot_usable =
                rp_guard.allocated_per_slot - alignment_offset;
            if (slot_usable < min_usable) min_usable = slot_usable;
        }
        usable = min_usable == 0xffffffffu ? 0u : min_usable;
    }
    return rp_w3_transport_make_plan(out, (uint32_t)raw_info.width,
        (uint32_t)raw_info.height, (uint32_t)raw_info.bits_per_pixel,
        le16, (uint32_t)raw_info.pitch, usable);
}

void raw_vid_rp_guard_get_status(struct rp_w3_guard_status *out)
{
    if (out) *out=rp_guard;
}


static void raw_vid_rp_ready_task(void);
static void raw_vid_rp_steer_task(void);

static inline void rp_dmb(void)
{
    /* RP-only control object is ARMv7-A/ARM state.  Canon's existing Complete
     * callback-table slot points to rp_hot_thumb.c (RAM callback, no ROM hook). */
    __asm__ volatile("dmb ish" ::: "memory");
}

static inline uint64_t rp_wall_us(void)
{
    return (uint64_t)get_us_clock();
}

typedef uint32_t (*rp_get_u32_fn)(void);

static inline uint32_t rp_raw_get_buffer(void)
{
    return ((rp_get_u32_fn)RP_RAW_GET_BUFFER_THUMB)();
}

static inline uint32_t rp_raw_get_size(void)
{
    return ((rp_get_u32_fn)RP_RAW_GET_SIZE_THUMB)();
}

static inline uint32_t rp_rd32(uint32_t a)
{
    return *(volatile uint32_t *)(uintptr_t)a;
}

static inline void rp_wr32(uint32_t a, uint32_t v)
{
    *(volatile uint32_t *)(uintptr_t)a = v;
}

static inline int8_t rp_rd8s(uint32_t a)
{
    return *(volatile int8_t *)(uintptr_t)a;
}

static inline void rp_wr8(uint32_t a, uint8_t v)
{
    *(volatile uint8_t *)(uintptr_t)a = v;
}

static int rp_valid_ram_ptr(uint32_t p)
{
    return ((p & 3u) == 0u && p >= 0x1000u && p < 0xC0000000u);
}

static uint32_t rp_evf_raw(void)
{
    uint32_t base = rp_rd32(RP_EVFDEV_VRAM_BASE_LITERAL);
    if (!rp_valid_ram_ptr(base))
        return 0u;
    return rp_rd32(base + 0xECu);
}

static int rp_read_mem1_geometry(struct rp_mem1_geometry *g)
{
    if (!g) return 0;
    memset(g, 0, sizeof(*g));

    uint32_t s = rp_rd32(RP_MEM1_STATE_PTR_LITERAL);
    if (!rp_valid_ram_ptr(s))
        return 0;

    g->state = s;
    g->mode = rp_rd32(s + RP_MEM1_MODE_OFF);
    g->b_gate = rp_rd32(s + RP_MEM1_B_GATE_OFF);
    g->d_gate = rp_rd32(s + RP_MEM1_D_GATE_OFF);
    g->a_w = rp_rd32(s + RP_MEM1_A_W_OFF); g->a_h = rp_rd32(s + RP_MEM1_A_H_OFF);
    g->b_w = rp_rd32(s + RP_MEM1_B_W_OFF); g->b_h = rp_rd32(s + RP_MEM1_B_H_OFF);
    g->c_w = rp_rd32(s + RP_MEM1_C_W_OFF); g->c_h = rp_rd32(s + RP_MEM1_C_H_OFF);
    g->d_w = rp_rd32(s + RP_MEM1_D_W_OFF); g->d_h = rp_rd32(s + RP_MEM1_D_H_OFF);
    g->selected_w = rp_rd32(s + RP_MEM1_SELECTED_W_OFF);
    g->selected_h = rp_rd32(s + RP_MEM1_SELECTED_H_OFF);

    if (g->mode < 0x14u)
    {
        if (g->b_gate != 0u &&
            (g->mode == 0x0Bu || g->mode == 0x0Fu || g->mode == 0x09u))
        {
            g->active_pair = 1u;
            g->active_w_off = RP_MEM1_B_W_OFF; g->active_h_off = RP_MEM1_B_H_OFF;
            g->active_w = g->b_w; g->active_h = g->b_h;
        }
        else
        {
            g->active_pair = 0u;
            g->active_w_off = RP_MEM1_A_W_OFF; g->active_h_off = RP_MEM1_A_H_OFF;
            g->active_w = g->a_w; g->active_h = g->a_h;
        }
    }
    else if (g->mode == 0x1Bu && g->d_gate != 0u)
    {
        g->active_pair = 3u;
        g->active_w_off = RP_MEM1_D_W_OFF; g->active_h_off = RP_MEM1_D_H_OFF;
        g->active_w = g->d_w; g->active_h = g->d_h;
    }
    else
    {
        g->active_pair = 2u;
        g->active_w_off = RP_MEM1_C_W_OFF; g->active_h_off = RP_MEM1_C_H_OFF;
        g->active_w = g->c_w; g->active_h = g->c_h;
    }
    return 1;
}

static uint32_t rp_mnav_desc_base(uint32_t worktype, uint32_t flag2)
{
    if (worktype == 1u) return 0x334u;
    if (worktype == 2u) return 0x4A4u;
    if (worktype == 3u) return 0x784u;
    if (worktype == 4u && flag2 != 0u) return 0x1C4u;
    if (worktype == 5u) return 0x8F4u;
    return 0x54u;
}

static int rp_read_mnav(struct rp_mnav_snapshot *m)
{
    if (!m) return 0;
    memset(m, 0, sizeof(*m));

    uint32_t state = rp_rd32(RP_MNAV_CONTROL);
    uint32_t flag2 = rp_rd32(RP_MNAV_CONTROL + 0x08u);
    uint32_t work = rp_rd32(RP_MNAV_CONTROL + 0x0Cu);
    if (!rp_valid_ram_ptr(state)) return 0;

    uint32_t desc = state + rp_mnav_desc_base(work, flag2) + RP_MNAV_LOW_ID * 0x10u;
    if (!rp_valid_ram_ptr(desc)) return 0;

    uint32_t pool = rp_rd32(desc + 0x00u);
    if (pool >= 0x100u) return 0;
    uint32_t pool_base_addr = state + pool * 4u + 4u;
    if (!rp_valid_ram_ptr(pool_base_addr)) return 0;

    m->state = state; m->worktype = work; m->flag2 = flag2; m->desc = desc;
    m->pool = pool; m->offset = rp_rd32(desc + 0x04u);
    m->stride = rp_rd32(desc + 0x08u); m->count = rp_rd32(desc + 0x0Cu);
    m->read_idx = (int32_t)rp_rd8s(state + 0x24u + RP_MNAV_LOW_ID * 2u);
    m->write_idx = (int32_t)rp_rd8s(state + 0x25u + RP_MNAV_LOW_ID * 2u);
    m->pool_base = rp_rd32(pool_base_addr);
    m->predicted_write = m->pool_base + m->offset +
                         m->stride * (uint32_t)m->write_idx;
    return 1;
}

static int rp_chain_is(uint32_t dma)
{
    return dma != 0u &&
           rp_rd32(RP_MNAV_PROVIDER_SLOT) == dma &&
           rp_evf_raw() == dma &&
           rp_rd32(RP_MEM1_CURRENT_BUFFER_SLOT) == dma &&
           rp_rd32(RP_RAW_PBLOCK + 0xA0u) == dma;
}

static int rp_find_free_capture_buffer(void)
{
    for (int i = 0; i < rp_buf_count; i++)
        if (rp_bufs[i].state == RPBUF_FREE)
            return i;
    return -1;
}

static void rp_mnav_write_offset(uint32_t dma)
{
    if (!rp_mnav_valid || !rp_mnav.desc || !rp_mnav.pool_base)
        return;
    rp_wr32(rp_mnav.desc + 0x04u, dma - rp_mnav.pool_base);
    rp_dmb();
    rp_steer_desc_writes++;
}

static void rp_mnav_restore_original(void)
{
    if (!rp_mnav_valid) return;
    rp_wr32(rp_mnav.desc + 0x00u, rp_mnav_desc_original[0]);
    rp_wr32(rp_mnav.desc + 0x04u, rp_mnav_desc_original[1]);
    rp_wr32(rp_mnav.desc + 0x08u, rp_mnav_desc_original[2]);
    rp_wr32(rp_mnav.desc + 0x0Cu, rp_mnav_desc_original[3]);
    rp_wr8(rp_mnav.state + 0x24u + RP_MNAV_LOW_ID * 2u,
           (uint8_t)rp_mnav_read_original);
    rp_wr8(rp_mnav.state + 0x25u + RP_MNAV_LOW_ID * 2u,
           (uint8_t)rp_mnav_write_original);
    rp_dmb();
}

static void rp_m2m_complete_cbr(void *ctx)
{
    uintptr_t which = (uintptr_t)ctx;

    /* IRQ/Complete context: no logging, no task/semaphore/FIO calls. */
    if (which == 1u)
        rp_m2m_rd_done = 1;
    else if (which == 2u)
        rp_m2m_wr_done = 1;

    __asm__ volatile("dmb ish" ::: "memory");
}

static uint8_t *rp_m2m_align_ptr(void *p)
{
    uintptr_t q = (uintptr_t)p;
    q = (q + RP_M2M_ALIGN - 1u) & ~(uintptr_t)(RP_M2M_ALIGN - 1u);
    return (uint8_t *)UNCACHEABLE((void *)q);
}

static inline uint32_t rp_complete_slot_read(void)
{
    return *(volatile uint32_t *)RP_COMPLETE_SLOT;
}

static inline uint32_t rp_complete_arg_read(void)
{
    return *(volatile uint32_t *)RP_COMPLETE_ARG_SLOT;
}

static void rp_reset_runtime_state(void)
{
    rp_capture_active = 0;
    rp_stop_requested = 0;
    rp_handback_done = 0;
    rp_final_complete_seen = 0;
    rp_ownership_uncertain = 0;
    rp_current_slot = -1;

    rp_owner_seq = 0;
    rp_complete_seq = 0;
    rp_ready_count = 0;
    rp_queued_count = 0;
    rp_released_count = 0;
    rp_drop_no_free = 0;
    rp_bad_complete = 0;
    rp_post_fail = 0;
    rp_fatal = 0;
    rp_stop_task_scheduled = 0;
    rp_overlap_count = 0;

    rp_steer_active = 0;
    rp_steer_expected_dma = 0;
    rp_steer_expected_slot = -2;
    rp_steer_current_slot = -1;
    rp_steer_latch_count = 0;
    rp_steer_canon_frames = 0;
    rp_steer_no_free = 0;
    rp_steer_desc_writes = 0;
    rp_steer_last_latch_us = 0;
    rp_complete_frame_size = 0;
    rp_complete_last_cur = 0;
    rp_complete_matched = 0;
    rp_complete_unmatched = 0;
}

static void rp_free_capture_buffers(void)
{
    for (int i = 0; i < RP_CAPTURE_BUFS; i++)
    {
        if (rp_bufs[i].alloc)
            fio_free(rp_bufs[i].alloc);

        rp_bufs[i].alloc = NULL;
        rp_bufs[i].dma = 0;
        rp_bufs[i].state = RPBUF_FREE;
        rp_bufs[i].owner_seq = 0;
        rp_bufs[i].complete_seq = 0;
    }
    rp_buf_count = 0;
    rp_capture_alloc_bytes = 0;
    rp_capture_wire_payload_bytes = 0;
    rp_capture_wire_pitch = 0;
}

static uint32_t rp_align_dma_buffer(void *alloc)
{
    uintptr_t p = (uintptr_t)alloc;
    p = (p + RP_CAPTURE_ALIGN - 1u) & ~(uintptr_t)(RP_CAPTURE_ALIGN - 1u);
    return (uint32_t)UNCACHEABLE((void *)p);
}

static int rp_alloc_capture_buffers(void)
{
    rp_free_capture_buffers();
    memset(&rp_guard,0,sizeof(rp_guard));
    uint32_t alloc_size=0;
    if (rp_capture_allocation_size(&alloc_size)) return -1;
    for (int i = 0; i < RP_CAPTURE_BUFS; i++)
    {
        void *p = fio_malloc(alloc_size);
        if (!p)
        {
            rp_free_capture_buffers();
            return -1;
        }

        rp_bufs[i].alloc = p;
        rp_bufs[i].dma = rp_align_dma_buffer(p);
        rp_bufs[i].state = RPBUF_FREE;
        rp_buf_count = i + 1;
        if (rp_guard_seed_slot(i))
        {
            RP_BACKEND_TRACE("W3GUARD seed failed slot=%d", i);
            rp_free_capture_buffers();
            return -2;
        }
    }
    rp_capture_alloc_bytes = alloc_size * (uint32_t)rp_buf_count;
    if (rp_guard.enabled)
        RP_BACKEND_TRACE("W3GUARD nominal=%d full16=%d allocated_each=%d slots=%d",
          (int)rp_guard.nominal,(int)rp_guard.nominal16,
          (int)rp_guard.allocated_per_slot,rp_buf_count);
    return 0;
}

static int rp_find_oldest_ready_buffer(void)
{
    int best = -1;
    uint32_t best_seq = 0;

    for (int i = 0; i < rp_buf_count; i++)
    {
        if (rp_bufs[i].state != RPBUF_READY)
            continue;

        uint32_t seq = rp_bufs[i].complete_seq;
        if (best < 0 || (int32_t)(seq - best_seq) < 0)
        {
            best = i;
            best_seq = seq;
        }
    }
    return best;
}

static int rp_install_complete_tap(void)
{
    if (rp_tap_installed)
        return 0;

    rp_saved_complete_fn = rp_complete_slot_read();
    rp_saved_complete_arg = rp_complete_arg_read();

    if (rp_saved_complete_fn != RP_CANON_COMPLETE_NOP_THUMB ||
        rp_saved_complete_arg != 0)
        return -10;

    rp_tap_ptr = (uint32_t)(uintptr_t)&raw_vid_rp_complete_tap;
    if ((rp_tap_ptr & 1u) == 0)
        return -12;

    /* One aligned pointer store only.  Canon retains IRQ ownership and arg. */
    *(volatile uint32_t *)RP_COMPLETE_SLOT = rp_tap_ptr;
    rp_dmb();

    if (rp_complete_slot_read() != rp_tap_ptr ||
        rp_complete_arg_read() != rp_saved_complete_arg)
    {
        if (rp_complete_slot_read() == rp_tap_ptr)
        {
            *(volatile uint32_t *)RP_COMPLETE_SLOT = rp_saved_complete_fn;
            rp_dmb();
        }
        return -11;
    }

    rp_tap_installed = 1;
    return 0;
}

static int rp_restore_complete_tap(void)
{
    if (!rp_tap_installed)
        return 0;

    /* Never overwrite a slot Canon changed behind our back. */
    if (rp_complete_slot_read() != rp_tap_ptr ||
        rp_complete_arg_read() != rp_saved_complete_arg)
        return -20;

    *(volatile uint32_t *)RP_COMPLETE_SLOT = rp_saved_complete_fn;
    rp_dmb();

    if (rp_complete_slot_read() != rp_saved_complete_fn ||
        rp_complete_arg_read() != rp_saved_complete_arg)
        return -21;

    rp_tap_installed = 0;
    return 0;
}

static volatile uint32_t *rp_get_pack_cfg_field(uint32_t field_off)
{
    uint32_t state = *(volatile uint32_t *)(uintptr_t)RP_MEM1_STATE_PTR_LITERAL;

    if ((state & 3u) || state < 0x1000u || state >= 0x40000000u)
        return NULL;

    return (volatile uint32_t *)(uintptr_t)
        (state + RP_MEM1_PACK_CFG_OFF + field_off);
}

/*
 * EOS RP fallback for generic raw_info discovery.
 *
 * v147/v148 runtime proof showed that generic raw_info may remain zero even
 * while Mem1/W3 is fully active.  The firmware ABI itself is self-consistent:
 *
 *   W3+0x50 low16       = pitch bytes
 *   W3+0x50 high16 + 1 = rows
 *   width               = pitch * 8 / 14
 *   W3+0xA0             = Canon Mem1 buffer
 *   E0379B7E            = exact frame byte count
 *
 * Do not guess.  Publish raw_info only when every independent view agrees.
 */
int raw_vid_rp_sync_raw_info(void)
{
    if (!is_camera("RP", "1.6.0"))
        return -70;

    uint32_t geom = *(volatile uint32_t *)(RP_RAW_PBLOCK + 0x50u);
    uint32_t pitch = geom & 0xffffu;
    uint32_t height = (geom >> 16) + 1u;
    uint32_t pblock_buffer = *(volatile uint32_t *)(RP_RAW_PBLOCK + 0xA0u);
    uint32_t canon_buffer = rp_raw_get_buffer();
    uint32_t canon_size = rp_raw_get_size();

    if (!pitch || !height || !pblock_buffer || !canon_buffer || !canon_size)
        return -71;

    uint64_t width_num = (uint64_t)pitch * 8u;
    if (width_num % 14u)
        return -72;

    uint32_t width = (uint32_t)(width_num / 14u);
    uint64_t frame64 = (uint64_t)pitch * (uint64_t)height;

    if (!width || width > 8192u ||
        !height || height > 8192u ||
        frame64 == 0u || frame64 > 0xffffffffu)
        return -73;

    uint32_t frame_size = (uint32_t)frame64;

    if (canon_buffer != pblock_buffer || canon_size != frame_size)
        return -74;

    raw_info.buffer = (void *)(uintptr_t)canon_buffer;
    raw_info.width = (int)width;
    raw_info.height = (int)height;
    raw_info.pitch = (int)pitch;
    raw_info.frame_size = (int)frame_size;
    raw_info.bits_per_pixel = 14;

    rp_dmb();

    RP_BACKEND_TRACE("RAWINFO sync buf=%x wh=%d,%d pitch=%d size=%d geom=%x",
                     canon_buffer, (int)width, (int)height,
                     (int)pitch, (int)frame_size, geom);
    return 0;
}

static volatile uint32_t *rp_get_pack_b3_field(void)
{
    return rp_get_pack_cfg_field(RP_MEM1_PACK_B3_OFF);
}

int raw_vid_rp_set_linear_packing(void)
{
    if (!is_camera("RP", "1.6.0"))
        return -80;
    if (rp_linear_packing_active)
        return 0;

    if (raw_info.bits_per_pixel != 10 &&
        raw_info.bits_per_pixel != 12 &&
        raw_info.bits_per_pixel != 14)
        return -81;

    volatile uint32_t *field = rp_get_pack_b3_field();
    if (!field)
        return -82;

    uint32_t old = *field;
    if ((old & 0xffu) != 0u)
        return -83;

    uint32_t pu_before =
        *(volatile uint32_t *)(RP_PU2_PBLOCK + RP_PU_MODE_OFF);

    if (pu_before & RP_PU_BIT0)
        return -84;

    rp_pack_b3_field = field;
    rp_pack_b3_saved = old;
    rp_linear_baseline_pu2 = pu_before;

    *field = (old & ~0xffu) | 1u;
    rp_dmb();

    uint32_t expected = pu_before | RP_PU_BIT0;
    uint32_t seen = 0;
    uint32_t t0 = get_us_clock();

    while ((uint32_t)(get_us_clock() - t0) < 250000u)
    {
        seen = *(volatile uint32_t *)(RP_PU2_PBLOCK + RP_PU_MODE_OFF);
        if (seen == expected && ((*field & 0xffu) == 1u))
            break;
        msleep(5);
    }

    if (seen != expected || ((*field & 0xffu) != 1u))
    {
        *field = old;
        rp_dmb();
        rp_pack_b3_field = NULL;
        rp_pack_b3_saved = 0;
        rp_linear_baseline_pu2 = 0;
        RP_BACKEND_TRACE("LINEAR apply FAIL bpp=%d old=%x exp=%x seen=%x field=%x",
                         raw_info.bits_per_pixel,
                         old, expected, seen, (uint32_t)*field);
        return -85;
    }

    rp_linear_packing_active = 1;
    RP_BACKEND_TRACE("LINEAR apply PASS bpp=%d field=%x PU2=%x->%x",
                     raw_info.bits_per_pixel,
                     (uint32_t)(uintptr_t)field, pu_before, seen);
    return 0;
}

int raw_vid_rp_restore_linear_packing(void)
{
    if (!is_camera("RP", "1.6.0"))
        return -90;
    if (!rp_linear_packing_active)
        return 0;
    if (!rp_pack_b3_field)
        return -91;

    *rp_pack_b3_field = rp_pack_b3_saved;
    rp_dmb();

    uint32_t seen = 0;
    uint32_t t0 = get_us_clock();

    while ((uint32_t)(get_us_clock() - t0) < 250000u)
    {
        seen = *(volatile uint32_t *)(RP_PU2_PBLOCK + RP_PU_MODE_OFF);
        if (seen == rp_linear_baseline_pu2 &&
            *rp_pack_b3_field == rp_pack_b3_saved)
            break;
        msleep(5);
    }

    int rc = 0;
    if (seen != rp_linear_baseline_pu2 ||
        *rp_pack_b3_field != rp_pack_b3_saved)
        rc = -92;

    RP_BACKEND_TRACE("LINEAR restore rc=%d PU2=%x baseline=%x field=%x",
                     rc, seen, rp_linear_baseline_pu2,
                     (uint32_t)*rp_pack_b3_field);

    rp_pack_b3_field = NULL;
    rp_pack_b3_saved = 0;
    rp_linear_baseline_pu2 = 0;
    rp_linear_packing_active = 0;
    return rc;
}

static volatile uint32_t *rp_get_pack_bpp_field(void)
{
    return rp_get_pack_cfg_field(RP_MEM1_PACK_BPP_OFF);
}

static int rp_make_lower_bpp_plan(int bpp, uint32_t *fake_width,
                                  uint32_t *physical_pitch,
                                  uint32_t *physical_size,
                                  uint32_t *expected_getsize,
                                  uint32_t *expected_geom,
                                  uint32_t *expected_mode)
{
    if (bpp != 10 && bpp != 12) return -1;

    uint32_t width = (uint32_t)raw_info.width;
    uint32_t height = (uint32_t)raw_info.height;
    uint32_t fake = 0u;

    if (width == 2128u && height == 1208u)
        fake = (bpp == 12) ? 1824u : 1520u;
    else if (width == 3728u && height == 2052u)
        fake = (bpp == 12) ? 3196u : 2663u;
    else
        return -2; /* lower-BPP RAM geometry not runtime-proven here */

    uint32_t raw_pitch = (uint32_t)(((uint64_t)fake * 7u) >> 2);
    uint32_t pitch = (uint32_t)(((uint64_t)width * (uint32_t)bpp) >> 3);
    uint32_t size = pitch * height;
    uint32_t getter = (uint32_t)(((uint64_t)fake * height * 7u) >> 2);
    uint32_t geom = rp_rd32(RP_RAW_PBLOCK + 0x50u);
    uint32_t pu = rp_rd32(RP_PU2_PBLOCK + RP_PU_MODE_OFF);

    if ((raw_pitch & RP_SETSIZE_XB_MASK) != pitch)
        return -3;

    *fake_width = fake;
    *physical_pitch = pitch;
    *physical_size = size;
    *expected_getsize = getter;
    *expected_geom = (geom & 0xffff0000u) | pitch;
    *expected_mode = (pu & ~0x30u) | ((bpp == 12) ? 0x10u : 0u);
    return 0;
}

int raw_vid_rp_set_bit_depth(int bpp)
{
    if (!is_camera("RP", "1.6.0"))
        return -50;
    if (bpp != 10 && bpp != 12 && bpp != 14)
        return -51;
    if (rp_bit_depth_active)
        return (raw_info.bits_per_pixel == bpp) ? 0 : -52;

    rp_saved_raw_bpp = raw_info.bits_per_pixel;
    rp_saved_raw_pitch = raw_info.pitch;
    rp_saved_raw_frame_size = raw_info.frame_size;
    rp_saved_pu2_mode = rp_rd32(RP_PU2_PBLOCK + RP_PU_MODE_OFF);

    if (bpp == 14)
    {
        rp_fake_width = (uint32_t)raw_info.width;
        rp_expected_getsize = (uint32_t)raw_info.frame_size;
        rp_expected_hw_geom = rp_rd32(RP_RAW_PBLOCK + 0x50u);
        rp_expected_pu2_mode = rp_saved_pu2_mode;
        rp_report_fake_width = rp_fake_width;
        rp_report_getsize = rp_expected_getsize;
        rp_report_hw_geom = rp_expected_hw_geom;
        rp_report_pu2_mode = rp_expected_pu2_mode;
        rp_mem1_saved_geom_valid = 0u;
        rp_bit_depth_active = 1;
        RP_BACKEND_TRACE("BPP RAM set 14 native pitch=%d", raw_info.pitch);
        return 0;
    }

    volatile uint32_t *field = rp_get_pack_bpp_field();
    if (!field)
        return -54;

    uint32_t old = *field;
    uint32_t old_mode = old & 0xffu;
    if (old_mode != 2u && old_mode != 3u)
        return -55;

    struct rp_mem1_geometry g;
    if (!rp_read_mem1_geometry(&g) ||
        g.active_w != (uint32_t)raw_info.width ||
        g.active_h != (uint32_t)raw_info.height ||
        g.selected_w != (uint32_t)raw_info.width ||
        g.selected_h != (uint32_t)raw_info.height)
        return -57;

    uint32_t fake=0, pitch=0, frame=0, getter=0, exp_geom=0, exp_mode=0;
    int plan_rc = rp_make_lower_bpp_plan(bpp, &fake, &pitch, &frame,
                                         &getter, &exp_geom, &exp_mode);
    if (plan_rc)
        return -58;

    rp_pack_bpp_field = field;
    rp_pack_bpp_saved = old;
    rp_pack_bpp_saved_valid = 1;
    rp_mem1_saved_geom = g;
    rp_mem1_saved_geom_valid = 1u;

    const uint32_t mode = (bpp == 12) ? 1u : 0u;

    /* Fail-safe ordering: first reduce PackUnpack precision while Canon still
     * uses the larger native14 row pitch, then shrink the writable Mem1 source
     * width.  Any inter-frame race can create padding, never a 14-bit row in a
     * lower-BPP-sized destination. */
    *field = (old & ~0xffu) | mode;
    rp_dmb();

    uint32_t t0 = get_us_clock();
    while ((uint32_t)(get_us_clock() - t0) < 250000u)
    {
        uint32_t pu = rp_rd32(RP_PU2_PBLOCK + RP_PU_MODE_OFF);
        if ((pu & ~RP_PU_BIT0) == (exp_mode & ~RP_PU_BIT0))
            break;
        msleep(1);
    }

    rp_wr32(g.state + g.active_w_off, fake);
    rp_dmb();

    uint32_t seen_geom=0, seen_getter=0, seen_pu=0;
    t0 = get_us_clock();
    while ((uint32_t)(get_us_clock() - t0) < 350000u)
    {
        struct rp_mem1_geometry now;
        if (rp_read_mem1_geometry(&now))
        {
            seen_geom = rp_rd32(RP_RAW_PBLOCK + 0x50u);
            seen_getter = rp_raw_get_size();
            seen_pu = rp_rd32(RP_PU2_PBLOCK + RP_PU_MODE_OFF);
            if (now.active_pair == g.active_pair &&
                now.active_w == fake && now.active_h == g.active_h &&
                now.selected_w == fake && now.selected_h == g.active_h &&
                seen_geom == exp_geom && seen_getter == getter &&
                (seen_pu & ~RP_PU_BIT0) == (exp_mode & ~RP_PU_BIT0))
                break;
        }
        msleep(1);
    }

    if (seen_geom != exp_geom || seen_getter != getter ||
        (seen_pu & ~RP_PU_BIT0) != (exp_mode & ~RP_PU_BIT0))
    {
        /* Safe rollback: restore native width before restoring 14-bit mode. */
        rp_wr32(g.state + g.active_w_off, g.active_w);
        rp_dmb();
        *field = old;
        rp_dmb();
        rp_pack_bpp_saved_valid = 0;
        rp_pack_bpp_field = NULL;
        rp_mem1_saved_geom_valid = 0u;
        RP_BACKEND_TRACE("BPP RAM apply FAIL req=%d fake=%d geom=%x/%x get=%d/%d pu=%x/%x",
                         bpp,(int)fake,seen_geom,exp_geom,(int)seen_getter,
                         (int)getter,seen_pu,exp_mode);
        return -56;
    }

    rp_fake_width = fake;
    rp_expected_getsize = getter;
    rp_expected_hw_geom = exp_geom;
    rp_expected_pu2_mode = exp_mode;
    rp_report_fake_width = fake;
    rp_report_getsize = getter;
    rp_report_hw_geom = exp_geom;
    rp_report_pu2_mode = exp_mode;
    raw_info.bits_per_pixel = bpp;
    raw_info.pitch = (int)pitch;
    raw_info.frame_size = (int)frame;
    rp_bit_depth_active = 1;

    RP_BACKEND_TRACE("BPP RAM set %d fakeW=%d pitch=%d frame=%d GetSize=%d geom=%x",
                     bpp,(int)fake,(int)pitch,(int)frame,(int)getter,exp_geom);
    return 0;
}

int raw_vid_rp_restore_bit_depth(void)
{
    if (!is_camera("RP", "1.6.0"))
        return -60;

    int linear_rc = raw_vid_rp_restore_linear_packing();

    if (!rp_bit_depth_active)
    {
        raw_vid_set_active_bpp(14);
        return linear_rc;
    }

    int old_bpp = raw_info.bits_per_pixel;
    int rc = 0;

    if (old_bpp != 14 && rp_pack_bpp_saved_valid && rp_pack_bpp_field &&
        rp_mem1_saved_geom_valid)
    {
        /* Reverse of entry ordering: enlarge physical row first while samples
         * are still 10/12-bit, then restore Canon's native14 PackUnpack. */
        rp_wr32(rp_mem1_saved_geom.state + rp_mem1_saved_geom.active_w_off,
                rp_mem1_saved_geom.active_w);
        rp_dmb();

        uint32_t native_geom =
            ((rp_mem1_saved_geom.active_h - 1u) << 16) |
            ((uint32_t)rp_saved_raw_pitch & 0xffffu);
        uint32_t t0 = get_us_clock();
        while ((uint32_t)(get_us_clock() - t0) < 250000u)
        {
            struct rp_mem1_geometry now;
            if (rp_read_mem1_geometry(&now) &&
                now.active_w == rp_mem1_saved_geom.active_w &&
                rp_rd32(RP_RAW_PBLOCK + 0x50u) == native_geom)
                break;
            msleep(1);
        }

        *rp_pack_bpp_field = rp_pack_bpp_saved;
        rp_dmb();

        t0 = get_us_clock();
        while ((uint32_t)(get_us_clock() - t0) < 250000u)
        {
            uint32_t pu = rp_rd32(RP_PU2_PBLOCK + RP_PU_MODE_OFF);
            if (rp_rd32(RP_RAW_PBLOCK + 0x50u) == native_geom &&
                *rp_pack_bpp_field == rp_pack_bpp_saved &&
                (pu & ~RP_PU_BIT0) == (rp_saved_pu2_mode & ~RP_PU_BIT0))
                break;
            msleep(1);
        }

        if (rp_rd32(RP_RAW_PBLOCK + 0x50u) != native_geom ||
            *rp_pack_bpp_field != rp_pack_bpp_saved)
            rc = -61;
    }

    raw_info.bits_per_pixel = rp_saved_raw_bpp;
    raw_info.pitch = rp_saved_raw_pitch;
    raw_info.frame_size = rp_saved_raw_frame_size;
    rp_pack_bpp_saved_valid = 0;
    rp_pack_bpp_field = NULL;
    rp_mem1_saved_geom_valid = 0u;
    rp_fake_width = 0u;
    rp_expected_getsize = 0u;
    rp_expected_hw_geom = 0u;
    rp_expected_pu2_mode = 0u;
    rp_bit_depth_active = 0;
    raw_vid_set_active_bpp(14);

    RP_BACKEND_TRACE("BPP RAM restore from=%d to=%d pitch=%d linear_rc=%d",
                     old_bpp, raw_info.bits_per_pixel, raw_info.pitch, linear_rc);

    if (rc == 0 && linear_rc != 0) rc = linear_rc;
    return rc;
}


static int rp_geometry_supported(void)
{
    int bpp = raw_info.bits_per_pixel;
    if (bpp != 10 && bpp != 12 && bpp != 14)
        return 0;

    uint64_t expected = (uint64_t)raw_info.width *
                        (uint64_t)raw_info.height * (uint64_t)bpp / 8u;
    if (expected != (uint32_t)raw_info.frame_size)
        return 0;
    if ((uint32_t)raw_info.pitch !=
        (uint32_t)raw_info.width * (uint32_t)bpp / 8u)
        return 0;

    if (raw_info.width == 3728 && raw_info.height == 2052)
        return 1;
    if (raw_info.width == 2128 && raw_info.height == 1208)
        return 1;
    if (raw_info.width == 2128 && raw_info.height == 864)
        return 1;
    return 0;
}

static int rp_crop_supported_at(uint32_t x, uint32_t y,
                                uint32_t w, uint32_t h)
{
    if (!w || !h || (w & 7u) || (h & 1u))
        return 0;
    if (x + w > (uint32_t)raw_info.width)
        return 0;
    if (y + h > (uint32_t)raw_info.height)
        return 0;

    /* Packed-pixel crop starts must remain byte-exact for all supported BPP.
     * The fixed x=192 origin and Full x=0 both satisfy this. */
    if (((x * (uint32_t)raw_info.bits_per_pixel) & 7u) != 0)
        return 0;

    return 1;
}

static int rp_crop_supported(uint32_t w, uint32_t h)
{
    return rp_crop_supported_at(RP_RAW_VID_CROP_X, RP_RAW_VID_CROP_Y, w, h);
}


/*
 * RP native MemoryToMemoryEsub1 bring-up probe.
 *
 * Stage 19: resource lock only.
 * Stage 20: + RD46/WR13 reset, Boomer select, Complete CBR registration.
 * Stage 21: + source/destination addresses, 4 KiB geometry, mode 0.
 * Stage 22: + actual one-shot 4 KiB transfer and byte-for-byte verification.
 * Stage 23: synthetic RAW-like 2D geometry/stride configuration, no start;
 *           NO large CPU prefill, and channels are explicitly reset before
 *           releasing resources/buffers.
 * Stage 24: large synthetic-buffer initialization only (cacheable writes +
 *           selective dcache clean), NO EDMAC setup.
 * Stage 25: actual 2240-byte x 720-row transfer and full verification.
 *
 * This deliberately does NOT use generic ML SetEDmac/StartEDmac overrides:
 * those remain no-op placeholders on the current RP port.  It also avoids the
 * high-level EFsVcopy wrapper because its production geometry descriptor lives
 * in Canon runtime RAM (E1236544).  Instead this follows the exact low-level
 * Esub1 sequence recovered from RP firmware, with our own tiny descriptors.
 */
int raw_vid_rp_probe_m2m(int stage)
{
    typedef void *(*res_create_fn)(uint32_t *, uint32_t);
    typedef void (*res_lock_fn)(void *);
    typedef void (*res_unlock_fn)(void);
    typedef void (*res_destroy_fn)(void *);
    typedef void (*ch_fn)(uint32_t);
    typedef void (*boomer_select_fn)(uint32_t, uint32_t);
    typedef void (*register_cbr_fn)(int, void (*)(void *), void *);
    /* IMPORTANT: ML modules are built with -DMODULE but without the per-camera
     * CONFIG_DIGIC_VIII/CONFIG_DIGIC_8X defines.  Therefore src/edmac.h exposes
     * the legacy 11-word edmac_info layout here, while Canon D8 E057C364 reads
     * a 15-word descriptor.  Keep an RP-local ABI-exact D8 descriptor instead
     * of relying on struct edmac_info in module code. */
    struct rp_d8_edmac_info
    {
        uint32_t off1s;
        uint32_t off1a;
        uint32_t off1b;
        uint32_t off2s;
        uint32_t off2a;
        uint32_t off2b;
        uint32_t off3;
        uint32_t xs;
        uint32_t xa;
        uint32_t xb;
        uint32_t ys;
        uint32_t ya;
        uint32_t yb;
        uint32_t xn;
        uint32_t yn;
    };

    typedef void (*set_addr_fn)(uint32_t, void *);
    typedef void (*set_size_fn)(uint32_t, struct rp_d8_edmac_info *);
    typedef void (*set_mode_fn)(uint32_t, uint32_t);

    static uint32_t resources[2] = { 0x000100ADu, 0x000100BBu };

    res_create_fn res_create = RP_THUMB_FN(RP_FN_RES_CREATE, res_create_fn);
    res_lock_fn res_lock = RP_THUMB_FN(RP_FN_RES_LOCK, res_lock_fn);
    res_unlock_fn res_unlock = RP_THUMB_FN(RP_FN_RES_UNLOCK, res_unlock_fn);
    res_destroy_fn res_destroy = RP_THUMB_FN(RP_FN_RES_DESTROY, res_destroy_fn);
    ch_fn edmac_reset = RP_THUMB_FN(RP_FN_EDMAC_RESET, ch_fn);
    ch_fn edmac_final_start_rd = RP_THUMB_FN(RP_FN_EDMAC_FINAL_START_RD, ch_fn);
    register_cbr_fn register_complete =
        RP_THUMB_FN(RP_FN_EDMAC_REGISTER_COMPLETE, register_cbr_fn);
    ch_fn unregister_complete =
        RP_THUMB_FN(RP_FN_EDMAC_UNREGISTER_COMPLETE, ch_fn);
    ch_fn boomer_reset = RP_THUMB_FN(RP_FN_BOOMER_RESET, ch_fn);
    boomer_select_fn boomer_select =
        RP_THUMB_FN(RP_FN_BOOMER_SELECT, boomer_select_fn);
    ch_fn stop_boomer = RP_THUMB_FN(RP_FN_EDMAC_STOP_BOOMER, ch_fn);
    ch_fn reset_pack = RP_THUMB_FN(RP_FN_EDMAC_RESET_PACK, ch_fn);
    ch_fn edmac_arm = RP_THUMB_FN(RP_FN_EDMAC_ARM, ch_fn);
    set_addr_fn set_address = RP_THUMB_FN(RP_FN_EDMAC_SET_ADDRESS, set_addr_fn);
    set_size_fn set_size = RP_THUMB_FN(RP_FN_EDMAC_SET_SIZE, set_size_fn);
    set_mode_fn set_mode = RP_THUMB_FN(RP_FN_EDMAC_SET_MODE, set_mode_fn);

    void *src_alloc = NULL;
    void *dst_alloc = NULL;
    uint8_t *src = NULL;
    uint8_t *dst = NULL;
    void *lock = NULL;
    int locked = 0;
    int cbrs_registered = 0;
    int boomer_configured = 0;
    int channels_configured = 0;
    int transfer_started = 0;
    int rc = -1;

    if (stage < 19 || stage > 31)
        return -100;

    const int rect_probe = (stage >= 23);
    const int matrix_probe = (stage >= 26);

    uint32_t rect_pitch = RP_M2M_RECT_SRC_PITCH;
    uint32_t rect_row_bytes = RP_M2M_RECT_ROW_BYTES;
    uint32_t rect_rows = RP_M2M_RECT_ROWS;
    uint32_t rect_xoff = RP_M2M_RECT_XOFF;
    uint32_t rect_yoff = RP_M2M_RECT_YOFF;
    uint32_t rect_guard = RP_M2M_RECT_GUARD;

    if (matrix_probe)
    {
        rect_row_bytes = RP_M2M_MATRIX_ROW_BYTES;
        rect_xoff = 0;
        rect_yoff = 0;
        rect_guard = RP_M2M_MATRIX_GUARD;

        switch (stage)
        {
            case 26: rect_rows = 2;   rect_pitch = rect_row_bytes; break;
            case 27: rect_rows = 2;   rect_pitch = RP_M2M_MATRIX_GAP_PITCH; break;
            case 28: rect_rows = 64;  rect_pitch = rect_row_bytes; break;
            case 29: rect_rows = 64;  rect_pitch = RP_M2M_MATRIX_GAP_PITCH; break;
            case 30: rect_rows = 720; rect_pitch = rect_row_bytes; break;
            case 31: rect_rows = 720; rect_pitch = RP_M2M_MATRIX_GAP_PITCH; break;
            default: return -100;
        }
    }

    const uint32_t rect_dst_bytes = rect_row_bytes * rect_rows;
    const uint32_t rect_src_span =
        (rect_yoff + rect_rows) * rect_pitch + rect_xoff;
    const uint32_t src_span = rect_probe ? rect_src_span : RP_M2M_BYTES;
    const uint32_t dst_span = rect_probe ? (rect_dst_bytes + rect_guard) : RP_M2M_BYTES;

    src_alloc = fio_malloc(src_span + RP_M2M_ALIGN);
    dst_alloc = fio_malloc(dst_span + RP_M2M_ALIGN);
    if (!src_alloc || !dst_alloc)
    {
        rc = -101;
        goto out;
    }

    uint8_t *src_base = rp_m2m_align_ptr(src_alloc);
    dst = rp_m2m_align_ptr(dst_alloc);

    if (rect_probe)
    {
        src = src_base + rect_yoff * rect_pitch + rect_xoff;
        RP_BACKEND_TRACE("M2M S%d RECT alloc base=%x src=%x dst=%x", stage,
                         (uint32_t)src_base, (uint32_t)src, (uint32_t)dst);
        RP_BACKEND_TRACE("M2M S%d RECT pitch=%d row=%d rows=%d xoff=%d yoff=%d", stage,
                         (int)rect_pitch, (int)rect_row_bytes,
                         (int)rect_rows, (int)rect_xoff, (int)rect_yoff);
        if (matrix_probe)
        {
            RP_BACKEND_TRACE("M2M S%d MATRIX gap=%d p16=%d p64=%d bytes=%d", stage,
                             (int)(rect_pitch - rect_row_bytes),
                             (int)(rect_pitch & 15u), (int)(rect_pitch & 63u),
                             (int)rect_dst_bytes);
        }

        /* IMPORTANT DIAGNOSTIC DIFFERENCE FROM v11i:
         * Do not byte-fill ~7 MB through fio_malloc's UNCACHEABLE alias on the
         * high-priority raw_v_worker task.  That test itself can starve Canon.
         * Stage 23 needs no payload at all.  Stages 24/25 prepare data through
         * the cacheable alias, then clean exactly those ranges for DMA. */
        if (stage >= 24)
        {
            uint8_t *src_cpu = (uint8_t *)CACHEABLE(src_base);
            uint8_t *dst_cpu = (uint8_t *)CACHEABLE(dst);

            RP_BACKEND_TRACE("M2M S%d RECT init begin bytes=%d/%d", stage,
                             (int)src_span, (int)dst_span);

            memset(dst_cpu, 0xA5, dst_span);

            /* Distinct value per logical crop row catches a wrong stride while
             * touching only the 1.61 MB payload, not every skipped source byte. */
            for (uint32_t y = 0; y < rect_rows; y++)
            {
                uint8_t v = (uint8_t)((0x31u + y * 29u) & 0xFFu);
                uint8_t *row = src_cpu +
                    (rect_yoff + y) * rect_pitch + rect_xoff;
                memset(row, v, rect_row_bytes);
                if ((y & 63u) == 63u)
                    msleep(1);
            }

            dcache_clean((uint32_t)src_cpu, src_span);
            dcache_clean((uint32_t)dst_cpu, dst_span);
            rp_dmb();
            RP_BACKEND_TRACE("M2M S%d RECT init PASS", stage);
        }

        if (stage == 24)
        {
            rc = 0;
            goto out;
        }
    }
    else
    {
        src = src_base;
        for (uint32_t i = 0; i < src_span; i++)
            src_base[i] = (uint8_t)((i * 37u + (i >> 3) * 11u + 0x5Au) & 0xFFu);
        for (uint32_t i = 0; i < dst_span; i++)
            dst[i] = 0xCCu;
        RP_BACKEND_TRACE("M2M S%d alloc src=%x dst=%x n=%d",
                         stage, (uint32_t)src, (uint32_t)dst, (int)RP_M2M_BYTES);
    }

    /* Exact EFsVcopy resource pair / create+lock path. */
    lock = res_create(resources, 2u);
    if (!lock)
    {
        rc = -102;
        goto out;
    }
    res_lock(lock);
    locked = 1;
    RP_BACKEND_TRACE("M2M S%d resource lock PASS lock=%x", stage, (uint32_t)lock);

    if (stage == 19)
    {
        rc = 0;
        goto out;
    }

    /* Exact E0738BAC channel/Boomer setup for RD46 / WR13. */
    edmac_reset(RP_M2M_WR_CH);
    edmac_reset(RP_M2M_RD_CH);
    boomer_reset(RP_M2M_WR_CH);
    boomer_select(RP_M2M_WR_CH, RP_M2M_BOOMER_SELECTOR);
    boomer_configured = 1;

    rp_m2m_rd_done = 0;
    rp_m2m_wr_done = 0;
    rp_dmb();

    register_complete((int)RP_M2M_WR_CH, rp_m2m_complete_cbr, (void *)2u);
    register_complete((int)RP_M2M_RD_CH, rp_m2m_complete_cbr, (void *)1u);
    cbrs_registered = 1;

    reset_pack(RP_M2M_WR_CH);
    reset_pack(RP_M2M_RD_CH);

    RP_BACKEND_TRACE("M2M S%d setup PASS rd=%d wr=%d sel=%x",
                     stage, (int)RP_M2M_RD_CH, (int)RP_M2M_WR_CH,
                     RP_M2M_BOOMER_SELECTOR);

    if (stage == 20)
    {
        rc = 0;
        goto out;
    }

    struct rp_d8_edmac_info src_info = {0};
    struct rp_d8_edmac_info dst_info = {0};
    if (rect_probe)
    {
        src_info.off1b = rect_pitch - rect_row_bytes;
        src_info.xb = rect_row_bytes;
        src_info.yb = rect_rows - 1u;
        dst_info.off1b = 0;
        dst_info.xb = rect_row_bytes;
        dst_info.yb = rect_rows - 1u;
    }
    else
    {
        src_info.xb = RP_M2M_BYTES;
        src_info.yb = 0;
        dst_info.xb = RP_M2M_BYTES;
        dst_info.yb = 0;
    }

    RP_BACKEND_TRACE("M2M S%d desc legacy=%d d8=%d xn=%d/%d yn=%d/%d",
                     stage, (int)sizeof(struct edmac_info),
                     (int)sizeof(struct rp_d8_edmac_info),
                     (int)src_info.xn, (int)dst_info.xn,
                     (int)src_info.yn, (int)dst_info.yn);

    /* Exact E0738D86 + E0738DA0 ordering: WR first, then RD. */
    RP_BACKEND_TRACE("M2M S%d cfg address begin", stage);
    set_address(RP_M2M_WR_CH, dst);
    set_address(RP_M2M_RD_CH, src);
    RP_BACKEND_TRACE("M2M S%d cfg size begin", stage);
    set_size(RP_M2M_WR_CH, &dst_info);
    RP_BACKEND_TRACE("M2M S%d cfg size WR PASS", stage);
    set_size(RP_M2M_RD_CH, &src_info);
    RP_BACKEND_TRACE("M2M S%d cfg size RD PASS", stage);
    set_mode(RP_M2M_WR_CH, 0u);
    set_mode(RP_M2M_RD_CH, 0u);
    channels_configured = 1;
    RP_BACKEND_TRACE("M2M S%d cfg mode PASS", stage);
    rp_dmb();

    uint32_t wr_addr = *(volatile uint32_t *)(RP_M2M_WR_PBLOCK + 0xA0u);
    uint32_t rd_addr = *(volatile uint32_t *)(RP_M2M_RD_PBLOCK + 0xA0u);
    RP_BACKEND_TRACE("M2M S%d config PASS rd_a=%x wr_a=%x rd_xb=%x wr_xb=%x",
                     stage, rd_addr, wr_addr,
                     *(volatile uint32_t *)(RP_M2M_RD_PBLOCK + 0x50u),
                     *(volatile uint32_t *)(RP_M2M_WR_PBLOCK + 0x50u));
    if (rect_probe)
    {
        RP_BACKEND_TRACE("M2M S%d RECT geom soff=%d sxb=%d syb=%d doff=%d dxb=%d dyb=%d",
                         stage, (int)src_info.off1b, (int)src_info.xb,
                         (int)src_info.yb, (int)dst_info.off1b,
                         (int)dst_info.xb, (int)dst_info.yb);
        RP_BACKEND_TRACE("M2M S%d RECT regs rd50=%x rd68=%x wr50=%x wr68=%x", stage,
                         *(volatile uint32_t *)(RP_M2M_RD_PBLOCK + 0x50u),
                         *(volatile uint32_t *)(RP_M2M_RD_PBLOCK + 0x68u),
                         *(volatile uint32_t *)(RP_M2M_WR_PBLOCK + 0x50u),
                         *(volatile uint32_t *)(RP_M2M_WR_PBLOCK + 0x68u));
    }

    if (rd_addr != (uint32_t)src || wr_addr != (uint32_t)dst)
    {
        rc = -103;
        goto out;
    }

    if (stage == 21 || stage == 23)
    {
        rc = 0;
        goto out;
    }

    /* E0738CB6: arm WR, arm RD, then final RD kick/connect. */
    uint32_t t0 = get_us_clock();
    transfer_started = 1;
    edmac_arm(RP_M2M_WR_CH);
    edmac_arm(RP_M2M_RD_CH);
    edmac_final_start_rd(RP_M2M_RD_CH);

    const uint32_t timeout_us = matrix_probe ? RP_M2M_MATRIX_TIMEOUT_US :
                                (rect_probe ? RP_M2M_RECT_TIMEOUT_US : RP_M2M_TIMEOUT_US);
    while (!(rp_m2m_rd_done && rp_m2m_wr_done))
    {
        if ((uint32_t)(get_us_clock() - t0) >= timeout_us)
            break;
        msleep(1);
        rp_dmb();
    }

    uint32_t dt = (uint32_t)(get_us_clock() - t0);
    RP_BACKEND_TRACE("M2M S%d complete rd=%d wr=%d us=%d",
                     stage, (int)rp_m2m_rd_done, (int)rp_m2m_wr_done, (int)dt);

    if (!(rp_m2m_rd_done && rp_m2m_wr_done))
    {
        rc = -104;
        goto out;
    }

    uint32_t mismatches = 0;
    uint32_t first_bad = 0xFFFFFFFFu;
    uint32_t guard_bad = 0;

    if (rect_probe)
    {
        /* The old diagnostic verifier walked uncached src+dst byte-by-byte.
         * Runtime logs show that this alone costs ~55-58 ms for only 143 KiB;
         * extrapolated to 1.61 MiB it can monopolize raw_v_worker for >0.5 s
         * and make a successful DMA transfer look like a camera freeze.
         *
         * All current matrix rows are 4-byte aligned (2240-byte row and both
         * 2240/6524 pitches are divisible by 4), so compare 32-bit words and
         * yield periodically.  On an unequal word, fall back to byte checks so
         * the reported mismatch count/first offset remain byte-exact. */
        const uint32_t verify_t0 = get_us_clock();
        RP_BACKEND_TRACE("M2M S%d verify begin bytes=%d", stage, (int)rect_dst_bytes);
        msleep(1);

        for (uint32_t y = 0; y < rect_rows; y++)
        {
            const uint8_t *srow = src + y * rect_pitch;
            const uint8_t *drow = dst + y * rect_row_bytes;
            const uint32_t words = rect_row_bytes >> 2;
            const uint32_t *sw = (const uint32_t *)srow;
            const uint32_t *dw = (const uint32_t *)drow;

            for (uint32_t w = 0; w < words; w++)
            {
                if (sw[w] != dw[w])
                {
                    uint32_t x0 = w << 2;
                    for (uint32_t b = 0; b < 4u; b++)
                    {
                        uint32_t x = x0 + b;
                        if (srow[x] != drow[x])
                        {
                            uint32_t off = y * rect_row_bytes + x;
                            if (first_bad == 0xFFFFFFFFu)
                                first_bad = off;
                            mismatches++;
                        }
                    }
                }
            }

            /* Current row size is divisible by four.  Keep a tail path so the
             * verifier stays correct if a later probe uses another width. */
            for (uint32_t x = words << 2; x < rect_row_bytes; x++)
            {
                if (srow[x] != drow[x])
                {
                    uint32_t off = y * rect_row_bytes + x;
                    if (first_bad == 0xFFFFFFFFu)
                        first_bad = off;
                    mismatches++;
                }
            }

            if (((y + 1u) % RP_M2M_VERIFY_YIELD_ROWS) == 0u)
                msleep(1);
        }

        for (uint32_t i = 0; i < rect_guard; i++)
            if (dst[rect_dst_bytes + i] != 0xA5u)
                guard_bad++;

        const uint32_t verify_dt = (uint32_t)(get_us_clock() - verify_t0);
        RP_BACKEND_TRACE("M2M S%d verify end us=%d bad=%d first=%x guard=%d", stage,
                         (int)verify_dt, (int)mismatches, first_bad, (int)guard_bad);
        RP_BACKEND_TRACE("M2M S%d samples s0=%x d0=%x sN=%x dN=%x", stage,
                         (uint32_t)src[0], (uint32_t)dst[0],
                         (uint32_t)src[(rect_rows - 1u) * rect_pitch + rect_row_bytes - 1u],
                         (uint32_t)dst[rect_dst_bytes - 1u]);
        rc = (mismatches || guard_bad) ? -105 : 0;
    }
    else
    {
        for (uint32_t i = 0; i < RP_M2M_BYTES; i++)
        {
            if (src[i] != dst[i])
            {
                if (first_bad == 0xFFFFFFFFu)
                    first_bad = i;
                mismatches++;
            }
        }

        RP_BACKEND_TRACE("M2M S22 verify bad=%d first=%x s0=%x d0=%x sN=%x dN=%x",
                         (int)mismatches, first_bad,
                         (uint32_t)src[0], (uint32_t)dst[0],
                         (uint32_t)src[RP_M2M_BYTES - 1u],
                         (uint32_t)dst[RP_M2M_BYTES - 1u]);
        rc = mismatches ? -105 : 0;
    }

out:
    /* Mirror Canon/M50 Esub1 teardown.  On pre-start probe stages these calls
     * merely remove the CBRs / selected Boomer state installed above. */
    if (cbrs_registered)
    {
        unregister_complete(RP_M2M_WR_CH);
        unregister_complete(RP_M2M_RD_CH);
    }
    if (boomer_configured)
        stop_boomer(RP_M2M_WR_CH);

    /* Canon/M50 normally tear down only after a completed transfer.  Stage 23
     * is intentionally configuration-only, so explicitly reset both channels
     * before freeing their backing buffers / releasing the resource lock.
     * Also reset on any failed/timeout transfer. */
    if (channels_configured && (!transfer_started || rc != 0))
    {
        edmac_reset(RP_M2M_WR_CH);
        edmac_reset(RP_M2M_RD_CH);
        rp_dmb();
        RP_BACKEND_TRACE("M2M S%d cleanup reset PASS", stage);
    }

    if (locked)
        res_unlock();
    if (lock)
        res_destroy(lock);

    if (src_alloc)
        fio_free(src_alloc);
    if (dst_alloc)
        fio_free(dst_alloc);

    RP_BACKEND_TRACE("M2M S%d END rc=%d", stage, rc);
    return rc;
}


/* Persistent mem2mem resource/power session for stage 33.
 *
 * This mirrors M50's edmac_memcpy_res_lock()/unlock() split rather than
 * keeping channel callbacks armed between frames.  The RP EFsVcopy wrappers
 * recovered statically map to the same lifecycle: E0726704 creates/locks the
 * engine resources, E07267AC wakes the subchips, E07267B2 suspends them and
 * E072672E unlocks/destroys the resource entry. */
int raw_vid_rp_m2m_session_start(void)
{
    typedef void *(*res_create_fn)(uint32_t *, uint32_t);
    typedef void (*res_lock_fn)(void *);

    static uint32_t resources[2] = { 0x000100ADu, 0x000100BBu };

    if (!is_camera("RP", "1.6.0"))
        return -220;
    if (rp_m2m_session_active || rp_m2m_session_lock)
        return -221;

    res_create_fn res_create = RP_THUMB_FN(RP_FN_RES_CREATE, res_create_fn);
    res_lock_fn res_lock = RP_THUMB_FN(RP_FN_RES_LOCK, res_lock_fn);

    void *lock = res_create(resources, 2u);
    if (!lock)
        return -222;

    res_lock(lock);
    rp_dmb();

    rp_m2m_session_lock = lock;
    rp_m2m_session_frames = 0;
    rp_m2m_session_errors = 0;
    rp_m2m_session_sum_us = 0;
    rp_m2m_session_min_us = 0;
    rp_m2m_session_max_us = 0;
    rp_m2m_session_active = 1;
    rp_dmb();

    RP_BACKEND_TRACE("M2M33 session START lock=%x", (uint32_t)lock);
    return 0;
}

int raw_vid_rp_m2m_session_stop(void)
{
    typedef void (*res_unlock_fn)(void);
    typedef void (*res_destroy_fn)(void *);

    if (!is_camera("RP", "1.6.0"))
        return 0;
    if (!rp_m2m_session_active && !rp_m2m_session_lock)
        return 0;

    res_unlock_fn res_unlock = RP_THUMB_FN(RP_FN_RES_UNLOCK, res_unlock_fn);
    res_destroy_fn res_destroy = RP_THUMB_FN(RP_FN_RES_DESTROY, res_destroy_fn);

    rp_m2m_session_active = 0;
    rp_dmb();

    void *lock = rp_m2m_session_lock;
    rp_m2m_session_lock = NULL;

    if (lock)
    {
        res_unlock();
        res_destroy(lock);
    }

    RP_BACKEND_TRACE("M2M33 session STOP frames=%d err=%d min=%d max=%d avg=%d",
                     (int)rp_m2m_session_frames,
                     (int)rp_m2m_session_errors,
                     (int)rp_m2m_session_min_us,
                     (int)rp_m2m_session_max_us,
                     rp_m2m_session_frames
                         ? (int)(rp_m2m_session_sum_us / rp_m2m_session_frames)
                         : 0);
    return 0;
}

/* Continuous real-frame crop for stage 33.  Resources/power are session-level,
 * while the transaction itself deliberately stays M50-like and synchronous:
 * reset channels -> Boomer -> CBRs -> pack reset -> addr/size/mode ->
 * WR arm -> RD arm -> final RD kick -> wait both Complete -> CBR teardown. */
int raw_vid_rp_m2m_crop_frame(const void *src_frame, uint32_t src_pitch,
                              uint32_t crop_x, uint32_t crop_y,
                              uint32_t crop_w, uint32_t crop_h,
                              void *dst, uint32_t dst_size,
                              uint32_t *elapsed_us)
{
    typedef void (*ch_fn)(uint32_t);
    typedef void (*boomer_select_fn)(uint32_t, uint32_t);
    typedef void (*register_cbr_fn)(int, void (*)(void *), void *);
    struct rp_d8_edmac_info_frame
    {
        uint32_t off1s;
        uint32_t off1a;
        uint32_t off1b;
        uint32_t off2s;
        uint32_t off2a;
        uint32_t off2b;
        uint32_t off3;
        uint32_t xs;
        uint32_t xa;
        uint32_t xb;
        uint32_t ys;
        uint32_t ya;
        uint32_t yb;
        uint32_t xn;
        uint32_t yn;
    };
    typedef void (*set_addr_fn)(uint32_t, void *);
    typedef void (*set_size_fn)(uint32_t, struct rp_d8_edmac_info_frame *);
    typedef void (*set_mode_fn)(uint32_t, uint32_t);

    if (!is_camera("RP", "1.6.0") || !rp_m2m_session_active ||
        !rp_m2m_session_lock || !src_frame || !dst)
        return -230;
    if (!crop_w || !crop_h || !src_pitch)
        return -231;

    const uint32_t row_bytes = crop_w * BPP / 8u;
    const uint32_t src_x_bytes = ((crop_x + 7u) / 8u) * BPP;
    const uint32_t src_y = (crop_y / 2u) * 2u;
    const uint32_t payload = row_bytes * crop_h;

    if (!row_bytes || src_x_bytes + row_bytes > src_pitch)
        return -232;
    if (payload > dst_size)
        return -233;
    if (src_y + crop_h > (uint32_t)raw_info.height)
        return -234;

    uint8_t *src = (uint8_t *)UNCACHEABLE(src_frame)
                 + src_y * src_pitch + src_x_bytes;
    uint8_t *out = (uint8_t *)UNCACHEABLE(dst);
    if ((((uintptr_t)src | (uintptr_t)out) & 3u) != 0)
        return -235;

    ch_fn edmac_reset = RP_THUMB_FN(RP_FN_EDMAC_RESET, ch_fn);
    ch_fn edmac_final_start_rd = RP_THUMB_FN(RP_FN_EDMAC_FINAL_START_RD, ch_fn);
    register_cbr_fn register_complete =
        RP_THUMB_FN(RP_FN_EDMAC_REGISTER_COMPLETE, register_cbr_fn);
    ch_fn unregister_complete =
        RP_THUMB_FN(RP_FN_EDMAC_UNREGISTER_COMPLETE, ch_fn);
    ch_fn boomer_reset = RP_THUMB_FN(RP_FN_BOOMER_RESET, ch_fn);
    boomer_select_fn boomer_select =
        RP_THUMB_FN(RP_FN_BOOMER_SELECT, boomer_select_fn);
    ch_fn stop_boomer = RP_THUMB_FN(RP_FN_EDMAC_STOP_BOOMER, ch_fn);
    ch_fn reset_pack = RP_THUMB_FN(RP_FN_EDMAC_RESET_PACK, ch_fn);
    ch_fn edmac_arm = RP_THUMB_FN(RP_FN_EDMAC_ARM, ch_fn);
    set_addr_fn set_address = RP_THUMB_FN(RP_FN_EDMAC_SET_ADDRESS, set_addr_fn);
    set_size_fn set_size = RP_THUMB_FN(RP_FN_EDMAC_SET_SIZE, set_size_fn);
    set_mode_fn set_mode = RP_THUMB_FN(RP_FN_EDMAC_SET_MODE, set_mode_fn);

    struct rp_d8_edmac_info_frame src_info = {0};
    struct rp_d8_edmac_info_frame dst_info = {0};
    src_info.off1b = src_pitch - row_bytes;
    src_info.xb = row_bytes;
    src_info.yb = crop_h - 1u;
    dst_info.xb = row_bytes;
    dst_info.yb = crop_h - 1u;

    int cbrs_registered = 0;
    int boomer_configured = 0;
    int transfer_started = 0;
    int rc = -236;

    edmac_reset(RP_M2M_WR_CH);
    edmac_reset(RP_M2M_RD_CH);
    boomer_reset(RP_M2M_WR_CH);
    boomer_select(RP_M2M_WR_CH, RP_M2M_BOOMER_SELECTOR);
    boomer_configured = 1;

    rp_m2m_rd_done = 0;
    rp_m2m_wr_done = 0;
    rp_dmb();

    register_complete((int)RP_M2M_WR_CH, rp_m2m_complete_cbr, (void *)2u);
    register_complete((int)RP_M2M_RD_CH, rp_m2m_complete_cbr, (void *)1u);
    cbrs_registered = 1;

    reset_pack(RP_M2M_WR_CH);
    reset_pack(RP_M2M_RD_CH);
    set_address(RP_M2M_WR_CH, out);
    set_address(RP_M2M_RD_CH, src);
    set_size(RP_M2M_WR_CH, &dst_info);
    set_size(RP_M2M_RD_CH, &src_info);
    set_mode(RP_M2M_WR_CH, 0u);
    set_mode(RP_M2M_RD_CH, 0u);
    rp_dmb();

    if (*(volatile uint32_t *)(RP_M2M_RD_PBLOCK + 0xA0u) != (uint32_t)src ||
        *(volatile uint32_t *)(RP_M2M_WR_PBLOCK + 0xA0u) != (uint32_t)out)
    {
        rc = -237;
        goto out;
    }

    const uint32_t t0 = get_us_clock();
    transfer_started = 1;
    edmac_arm(RP_M2M_WR_CH);
    edmac_arm(RP_M2M_RD_CH);
    edmac_final_start_rd(RP_M2M_RD_CH);

    while (!(rp_m2m_rd_done && rp_m2m_wr_done))
    {
        if ((uint32_t)(get_us_clock() - t0) >= RP_M2M_RECT_TIMEOUT_US)
            break;
        msleep(1);
        rp_dmb();
    }

    const uint32_t dt = (uint32_t)(get_us_clock() - t0);
    if (elapsed_us)
        *elapsed_us = dt;

    if (!(rp_m2m_rd_done && rp_m2m_wr_done))
    {
        rc = -238;
        goto out;
    }

    rc = 0;

out:
    if (cbrs_registered)
    {
        unregister_complete(RP_M2M_WR_CH);
        unregister_complete(RP_M2M_RD_CH);
    }
    if (boomer_configured)
        stop_boomer(RP_M2M_WR_CH);

    if (transfer_started && rc != 0)
    {
        edmac_reset(RP_M2M_WR_CH);
        edmac_reset(RP_M2M_RD_CH);
        rp_dmb();
    }

    rp_m2m_session_frames++;
    if (rc != 0)
        rp_m2m_session_errors++;
    if (elapsed_us)
    {
        const uint32_t dt = *elapsed_us;
        rp_m2m_session_sum_us += dt;
        if (rp_m2m_session_min_us == 0 || dt < rp_m2m_session_min_us)
            rp_m2m_session_min_us = dt;
        if (dt > rp_m2m_session_max_us)
            rp_m2m_session_max_us = dt;
    }

    if (rc != 0 || rp_m2m_session_frames <= 4u ||
        (rp_m2m_session_frames & 31u) == 0u)
    {
        RP_BACKEND_TRACE("M2M33 frame=%d rc=%d rd=%d wr=%d us=%d src=%x dst=%x",
                         (int)rp_m2m_session_frames, rc,
                         (int)rp_m2m_rd_done, (int)rp_m2m_wr_done,
                         elapsed_us ? (int)*elapsed_us : 0,
                         (uint32_t)src, (uint32_t)out);
    }

    return rc;
}
