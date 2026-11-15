#ifndef _RAW_VIDX_RP_CRX_FORMAT_H_
#define _RAW_VIDX_RP_CRX_FORMAT_H_

#include <stdint.h>

/*
 * EOS RP native-CRX integration for MLV.
 *
 * Recorder format revision 2:
 *   MLVI.videoClass = RAW | CRX
 *   CRXI v2         = one stream-level exact Canon CMP1 blob
 *   VIDF.frameData  = exact Canon CRX sample
 *
 * No CRXF block is emitted by the recorder.
 */
#define RP_CRX_BLOCK_CRXI              0x49585243u  /* "CRXI" */
#define RP_CRX_VIDEO_CLASS_FLAG        0x0200u
#define RP_CRX_FORMAT_VERSION          2u
#define RP_CRX_DESCRIPTOR_BYTES        0x2Cu
#define RP_CRX_MAX_SEGMENTS            64u
#define RP_CRX_META_MAX_BYTES          4096u

/* Lossless and C-RAW pipelines are submitted end-to-end at the selected
 * 10/12/14-bit depth.  The recorder validates CMP1.nBits == logical RAWI BPP
 * and therefore writes valueShift=0.
 * A future pipeline with a different numeric mapping must set an explicit
 * different valueShift; readers must not infer it from bit depths. */
#define RP_CRX_VALUE_SHIFT_CURRENT     0

/* Canon copies the submitted descriptor type into result_handle[1].
 * Historical fixed-slot lossless routing used:
 *   14-bit -> type/profile 0, 12-bit -> 4, 10-bit -> 5.
 * Dynamic allocator mode deliberately decouples BPP from descriptor type, so
 * these constants describe only the legacy fixed-route defaults. Result-handle
 * validation must compare handle[1] with the exact submitted descriptor type. */
#define RP_CRX_PROFILE_14BIT            0u
#define RP_CRX_PROFILE_12BIT            4u
#define RP_CRX_PROFILE_10BIT            5u
/* Canon RAW/C-RAW paired D routes recovered from TYPE_MAP and native still
 * runtime:
 *   RAW 0/4/5 -> ID0/8/9
 *   CRAW 2/6/7 -> ID2/10/11
 * The custom 14-bit type2/ID2 route is runtime-proven; 12/10-bit use the
 * corresponding paired type6/type7 routes and remain fail-closed on CMP1
 * nBits/geometry/codec validation. */
#define RP_CRAW_PROFILE_14BIT           2u
#define RP_CRAW_PROFILE_12BIT           6u
#define RP_CRAW_PROFILE_10BIT           7u
#define RP_CRX_PROFILE_INVALID          0xFFFFu

static inline uint16_t rp_crx_profile_for_mode_bpp(int lossy, uint32_t bits)
{
    if (lossy)
    {
        switch (bits)
        {
            case 14u: return RP_CRAW_PROFILE_14BIT;
            case 12u: return RP_CRAW_PROFILE_12BIT;
            case 10u: return RP_CRAW_PROFILE_10BIT;
            default:  return RP_CRX_PROFILE_INVALID;
        }
    }

    switch (bits)
    {
        case 14u: return RP_CRX_PROFILE_14BIT;
        case 12u: return RP_CRX_PROFILE_12BIT;
        case 10u: return RP_CRX_PROFILE_10BIT;
        default:  return RP_CRX_PROFILE_INVALID;
    }
}

static inline uint16_t rp_crx_profile_for_bpp(uint32_t bits)
{
    return rp_crx_profile_for_mode_bpp(0, bits);
}

#pragma pack(push, 1)

/* MLV CRX Container Revision 2 stream configuration.
 * Exact Canon CMP1 bytes immediately follow this fixed 40-byte header. */
struct rp_crxi_v2_block
{
    uint32_t type;          /* "CRXI" */
    uint32_t size;          /* sizeof(header) + cmp1_size */
    uint64_t timestamp;

    uint32_t version;       /* 2 */
    uint32_t header_size;   /* sizeof(struct rp_crxi_v2_block) == 40 */
    uint32_t flags;         /* revision 2: MUST be 0 */
    int32_t  value_shift;   /* CRX decoded -> RAWI numeric-domain mapping */
    uint32_t cmp1_size;     /* exact bytes immediately following header */
    uint32_t reserved;      /* MUST be 0 */
};

#pragma pack(pop)

typedef char rp_crxi_v2_block_must_be_40_bytes[
    (sizeof(struct rp_crxi_v2_block) == 40u) ? 1 : -1
];

#endif
