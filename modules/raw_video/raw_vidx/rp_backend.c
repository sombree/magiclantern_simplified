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
