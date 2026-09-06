/** \file
 * VRAM Compositor interface
 */

/**
 * There's a hardware accelerated compositor running on Digic 6+ cameras.
 * It has couple of variants (pure Ximr, XCM) - see further comments.
 *
 * For more detailed explanation on how this was discovered, look at forums:
 * https://www.magiclantern.fm/forum/index.php?topic=26024
 */

/**
 * Terminology:
 * Zico core    - Xtensa core, running DryOS, used for graphics acceeleration,
 *                GUI rendering and compositing. Connected with GPU (Takumi
 *                GV550 IP core)
 * mzrm         - Marius - Zico remote messaging (?). RPC used to talk between
 *                main ARM core (named Marius on D6) and Zico.
 * Ximr         - Render MIXer (?). Low level compositor on D6 and up.
 * Ximr Context - Data structure representing Ximr configuration, sent via mzrm
 *                using XimrExe() to Zico for render.
 * Input layer  - Layer (described by MARV struct) to be included in mix.
 * XOC          - Ximr Output Chunk. Buffer where final image is rendered.
 * XCM          - Ximr Context Maker. "Userspace" tool to deal with Ximr setup.
 *                Exists on D8 and up, from EOS R (M50 uses pure Ximr)
 */

#include "dryos.h"
#include "bmp.h"
#include "compositor.h"

#ifdef CONFIG_COMPOSITOR_DEDICATED_LAYER
/**
 * refreshVrmsSurface (renamed to VMIX_TransferRectangleToVram somewhere around 850D)
 * is used by Canon code to render Input layers into XOC.
 *
 * On pure XIMR cameras it uses XIMR functions to setup Ximr Context
 */
extern void      refreshVrmsSurface();
extern uint32_t  display_refresh_needed;

/**
 * Stores our layer ID. Defaults to Canon GUI layer, so in case
 * alocation failed - everything falls back to non-compositing behaviour.
 */
int _rgb_vram_layer_id = CANON_GUI_LAYER_ID;
struct MARV * pNewLayer;

/**
 * Not sure if sync_caches() call is needed. It was when I was drawing
 * over Canon buffers, but now when we have our own may be unnecessary.
 */
void _compositor_force_redraw()
{
    display_refresh_needed = 1;
    ml_refresh_display_needed = 1;
    sync_caches();
    //refreshVrmsSurface(); // not needed as we do XimrExe() in bmp.c
}

struct MARV *_compositor_create_layer(uint32_t bmp_w, uint32_t bmp_h)
{
    // buffer for MARV structure
    struct MARV* pNewLayer = malloc(sizeof(struct MARV));
    if(pNewLayer == NULL) return NULL;
    
    // buffer for layer data, +0x100 for alignment
    uint8_t* pBitmapData = malloc((BMP_VRAM_SIZE * 4) + 0x100);
    if(pBitmapData == NULL) {
        free(pNewLayer); // cleanup
        return NULL;
    }
    
    // Align to 0x100 and use uncacheable region.
    // This is expected by some Zico functions.
    pBitmapData = UNCACHEABLE((uint8_t*)((((uintptr_t)pBitmapData + 0x100) >> 8) << 8));

    // fill MARV data
    pNewLayer->signature    = 0x5652414D;  // MARV
    pNewLayer->bitmap_data  = pBitmapData;
    pNewLayer->opacity_data = 0x0;
    pNewLayer->flags        = XIMR_FLAGS_LAYER_RGBA; // see compositor.h
#ifdef CONFIG_DIGIC_X
    extern uint64_t MemifWindow_GetIBusAddress(uint8_t* buf);
    pNewLayer->memif_1      = 0xFFFFFFFF;   // no idea
    pNewLayer->ibus_addr    = MemifWindow_GetIBusAddress(CACHEABLE(pBitmapData));
#endif
    pNewLayer->width        = (uint32_t)bmp_w;
    pNewLayer->height       = (uint32_t)bmp_h;
    // Probably pmem is not needed. No issues observed so far.
    pNewLayer->pmem         = 0x0;

    DryosDebugMsg(0, 15, "MARV 0x%08x, bitmap_data 0x%08x", pNewLayer, pBitmapData);

    return pNewLayer;
}

#ifdef CONFIG_COMPOSITOR_XCM
/**
 * Implementation specific to models with Ximr Context Maker (XCM)
 */

#if !defined(CONFIG_COMPOSITOR_XCM_V1) && !defined(CONFIG_COMPOSITOR_XCM_V2)
#error "CONFIG_COMPOSITOR_XCM enabled, but no XCM type was defined!"
#endif

#ifdef CONFIG_DIGIC_X
#error XCM is not yet implemented for DIGIC_X, see compositor.c for details!
/*
 * Digic X uses the new "IBus" stuff for some kind of memory banking
 * We don't know how to use it yet, and Ximr now requires buffers to be
 * "on specific ibus". Otherwise camera aserts.
 *
 * I left in my best guess on Digic X implementation, but it requires malloc
 * replacment that will satisfy Ximr IBus requirements.
 */
#endif

#ifdef CONFIG_COMPOSITOR_XCM_V1
/*
 * Structures specific to EOS R / PowerShot SX740
 * XCM_SetSourceSurface(), XCM_SetSourceArea(), XOC_SetLayerEnable() calls are
 * done on each redraw (in refreshVrmsSurface() ) based on structures below.
 *
 * VMIX_Layers        List of *MARV for all existing layers
 * VMIX_LayersEnable  Controls if layer with given ID is displayed or not.
 */
extern struct MARV *VMIX_Layers[XIMR_MAX_LAYERS];
#ifdef CONFIG_R
extern uint32_t VMIX_LayersEnable[XIMR_MAX_LAYERS]; // R only, DNE on SX740
#endif // CONFIG_R
#endif // CONFIG_COMPOSITOR_XCM_V1

#ifdef CONFIG_COMPOSITOR_XCM_V2
/*
 * Pure XCM API is used
 */
extern uint32_t XCM_SetSourceSurface(void * pXCM, uint layer, struct MARV * pMARV);
extern uint32_t XCM_SetSourceArea(void * pXCM, uint layer, uint16_t srcX, uint16_t srcY, uint16_t srcW, uint16_t srcH);
extern uint32_t XOC_SetLayerEnable(int p1, int p2, uint layer, int p4);
#ifdef CONFIG_COMPOSITOR_XCM_NATIVE_INDEXED
extern uint32_t XCM_MakeContext(void *pXCM, uint32_t arg1, uint32_t arg2);
extern uint32_t XCM_SetLayerOffset(
    void *pXCM, uint32_t layer, uint16_t x, uint16_t y);
#endif
#endif // CONFIG_COMPOSITOR_XCM_V2


#ifdef CONFIG_COMPOSITOR_XCM_NATIVE_INDEXED
/*
 * Native 8-bit indexed XCM backend.
 *
 * Platforms opt in explicitly and provide the small platform policy hooks
 * declared in compositor.h. The core owns ML layers and their MARV state;
 * platform code owns global LUT state and Canon GUI-layer policy.
 */
#define XCM_NATIVE_LUT_ENTRIES 256u
#define XCM_NATIVE_LUT_BYTES \
    (XCM_NATIVE_LUT_ENTRIES * sizeof(uint32_t))

struct native_indexed_layer
{
    struct MARV marv;
    int layer_id;
};

static struct native_indexed_layer native_indexed_layers[
    COMPOSITOR_NATIVE_LAYER_COUNT] = {
        [COMPOSITOR_NATIVE_LAYER_MENU] = { .layer_id = -1 },
        [COMPOSITOR_NATIVE_LAYER_OVERLAY] = { .layer_id = -1 },
    };

static uint8_t native_indexed_lut_storage[XCM_NATIVE_LUT_BYTES + 0xffu]
    __attribute__((aligned(64)));
static int native_indexed_lut_active = 0;

static volatile uint32_t *native_indexed_lut_uncached(void)
{
    uintptr_t p = (uintptr_t)native_indexed_lut_storage;
    p = (p + 0xffu) & ~(uintptr_t)0xffu;
    return (volatile uint32_t *)UNCACHEABLE((void *)p);
}

static int native_clamp_i32(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Convert ML's 0xAARRGGBB palette entries to XCM type-2 AAVVUUYY. */
static uint32_t native_argb_to_avuy(uint32_t argb)
{
    int a = (int)((argb >> 24) & 0xffu);
    int r = (int)((argb >> 16) & 0xffu);
    int g = (int)((argb >> 8) & 0xffu);
    int b = (int)(argb & 0xffu);
    int y = native_clamp_i32((217 * r + 732 * g + 73 * b) / 1024,
                             0, 255);
    int u = native_clamp_i32(
        (-117 * r - 394 * g + 512 * b) / 1024,
        -128, 127) + 128;
    int v = native_clamp_i32(
        (512 * r - 465 * g - 46 * b) / 1024,
        -128, 127) + 128;

    return ((uint32_t)a << 24) |
           ((uint32_t)v << 16) |
           ((uint32_t)u << 8) |
           (uint32_t)y;
}

static void native_indexed_build_lut(void)
{
    volatile uint32_t *lut = native_indexed_lut_uncached();
    for (uint32_t i = 0; i < XCM_NATIVE_LUT_ENTRIES; i++)
        lut[i] = native_argb_to_avuy(indexed2rgb((uint8_t)i));
}

static int native_indexed_ensure_lut(void)
{
    if (!_pXCM)
        return 1;

    if (!native_indexed_lut_active)
        native_indexed_build_lut();

    int rc = compositor_native_indexed_platform_lut_install(
        (void *)native_indexed_lut_uncached());
    if (rc != 0)
        return 2;

    native_indexed_lut_active = 1;
    return 0;
}

static int native_indexed_role_valid(int role)
{
    return role >= 0 && role < COMPOSITOR_NATIVE_LAYER_COUNT;
}

static int native_indexed_create_layer(int role, uint8_t *indexed_base)
{
    if (!_pXCM || !indexed_base || !rgb_vram_info ||
        !native_indexed_role_valid(role))
        return 1;

    if (_rgb_vram_layer_id == CANON_GUI_LAYER_ID)
        return 7;

    struct native_indexed_layer *slot = &native_indexed_layers[role];

    if (slot->layer_id >= 0)
    {
        struct MARV *owner = XCM_GetSourceSurface(
            _pXCM, (uint32_t)slot->layer_id);
        if (owner == &slot->marv || owner == NULL)
            return 0;

        slot->layer_id = -1;
    }

    int layer = XIMR_MAX_LAYERS - 1;
    while (layer >= 0)
    {
        if (layer != _rgb_vram_layer_id &&
            XCM_GetSourceSurface(_pXCM, (uint32_t)layer) == NULL)
            break;
        layer--;
    }
    if (layer < 0)
        return 2;

    slot->marv.signature = 0x5652414D;
    slot->marv.bitmap_data = (uint8_t *)UNCACHEABLE(indexed_base);
    slot->marv.opacity_data = 0;
    slot->marv.flags = XIMR_FLAGS_LAYER_INDEXED_8BPP;
    slot->marv.width = BMPPITCH;
    slot->marv.height = BMP_VRAM_SIZE / BMPPITCH;
    slot->marv.pmem = 0;

    if (XCM_SetSourceSurface(_pXCM, (uint)layer, &slot->marv) != 0)
        return 4;

    if (XCM_SetSourceArea(
            _pXCM, (uint)layer,
            (uint16_t)(-BMP_W_MINUS), (uint16_t)(-BMP_H_MINUS),
            (uint16_t)(BMP_W_PLUS + BMP_W_MINUS),
            (uint16_t)(BMP_H_PLUS + BMP_H_MINUS)) != 0)
    {
        XCM_SetSourceSurface(_pXCM, (uint)layer, NULL);
        return 5;
    }

    if (XCM_SetLayerOffset(_pXCM, (uint32_t)layer, 0, 0) != 0)
    {
        XCM_SetSourceSurface(_pXCM, (uint)layer, NULL);
        return 6;
    }

    if (XOC_SetLayerEnable(0, 0, (uint)layer, 0) != 0)
    {
        XCM_SetSourceSurface(_pXCM, (uint)layer, NULL);
        return 8;
    }

    slot->layer_id = layer;
    DryosDebugMsg(0, 15, "Native XCM layer role=%d layer=%d",
        role, layer);
    return 0;
}

static int native_indexed_set_owned_visible(int role, int enable)
{
    if (!native_indexed_role_valid(role))
        return 1;

    struct native_indexed_layer *slot = &native_indexed_layers[role];
    if (slot->layer_id < 0)
        return 0;

    if (XCM_GetSourceSurface(_pXCM, (uint32_t)slot->layer_id) !=
        &slot->marv)
    {
        slot->layer_id = -1;
        return 2;
    }

    return (int)XOC_SetLayerEnable(
        0, 0, (uint)slot->layer_id, enable ? 1 : 0);
}

void compositor_native_indexed_menu_enter(void)
{
    compositor_native_indexed_platform_menu_enter();
}

void compositor_native_indexed_menu_exit(void)
{
    compositor_native_indexed_platform_menu_exit();
}

int compositor_native_indexed_prepare(int role, uint8_t *indexed_base)
{
    if (!_pXCM || !indexed_base || !rgb_vram_info ||
        !native_indexed_role_valid(role))
        return 10;

    int rc = native_indexed_ensure_lut();
    if (rc != 0)
        return 31 + rc;

    rc = native_indexed_create_layer(role, indexed_base);
    if (rc != 0)
        return 20 + rc;

    struct native_indexed_layer *slot = &native_indexed_layers[role];
    if (slot->layer_id < 0)
        return 29;

    struct MARV *owner = XCM_GetSourceSurface(
        _pXCM, (uint32_t)slot->layer_id);
    if (owner != NULL && owner != &slot->marv)
    {
        slot->layer_id = -1;
        return 30;
    }

    slot->marv.bitmap_data = (uint8_t *)UNCACHEABLE(indexed_base);
    rc = (int)XCM_SetSourceSurface(
        _pXCM, (uint)slot->layer_id, &slot->marv);
    if (rc != 0)
        return 40;

    rc = (int)XCM_SetSourceArea(
        _pXCM, (uint)slot->layer_id,
        (uint16_t)(-BMP_W_MINUS), (uint16_t)(-BMP_H_MINUS),
        (uint16_t)(BMP_W_PLUS + BMP_W_MINUS),
        (uint16_t)(BMP_H_PLUS + BMP_H_MINUS));
    if (rc != 0)
        return 41;

    rc = (int)XCM_SetLayerOffset(
        _pXCM, (uint32_t)slot->layer_id, 0, 0);
    if (rc != 0)
        return 42;

    rc = (int)XOC_SetLayerEnable(0, 0, (uint)_rgb_vram_layer_id, 0);
    if (rc != 0)
        return 43;

    int other = role == COMPOSITOR_NATIVE_LAYER_MENU ?
        COMPOSITOR_NATIVE_LAYER_OVERLAY : COMPOSITOR_NATIVE_LAYER_MENU;
    native_indexed_set_owned_visible(other, 0);

    rc = native_indexed_set_owned_visible(role, 1);
    if (rc != 0)
    {
        XOC_SetLayerEnable(0, 0, (uint)_rgb_vram_layer_id, 1);
        return 44;
    }

    if (role == COMPOSITOR_NATIVE_LAYER_MENU)
        compositor_native_indexed_platform_menu_reassert();

    rc = (int)XCM_MakeContext(_pXCM, 0, 0);
    if (rc != 0)
        return 50;

    return 0;
}

int compositor_native_indexed_hide_all(void)
{
    if (!_pXCM)
        return 1;

    native_indexed_set_owned_visible(COMPOSITOR_NATIVE_LAYER_MENU, 0);
    native_indexed_set_owned_visible(COMPOSITOR_NATIVE_LAYER_OVERLAY, 0);

    if (_rgb_vram_layer_id != CANON_GUI_LAYER_ID)
        XOC_SetLayerEnable(0, 0, (uint)_rgb_vram_layer_id, 0);

    return (int)XCM_MakeContext(_pXCM, 0, 0);
}

void compositor_native_indexed_disable(void)
{
    if (!_pXCM)
        return;

    compositor_native_indexed_menu_exit();

    if (_rgb_vram_layer_id != CANON_GUI_LAYER_ID)
        XOC_SetLayerEnable(0, 0, (uint)_rgb_vram_layer_id, 1);

    for (int role = 0; role < COMPOSITOR_NATIVE_LAYER_COUNT; role++)
    {
        struct native_indexed_layer *slot = &native_indexed_layers[role];
        int layer = slot->layer_id;
        if (layer >= 0 &&
            XCM_GetSourceSurface(_pXCM, (uint32_t)layer) == &slot->marv)
        {
            XOC_SetLayerEnable(0, 0, (uint)layer, 0);
            XCM_SetSourceSurface(_pXCM, (uint)layer, NULL);
        }
        slot->layer_id = -1;
    }

    if (native_indexed_lut_active)
        compositor_native_indexed_platform_lut_restore();
    native_indexed_lut_active = 0;
    XCM_MakeContext(_pXCM, 0, 0);
}

int compositor_native_indexed_is_active(void)
{
    if (native_indexed_lut_active)
        return 1;

    for (int role = 0; role < COMPOSITOR_NATIVE_LAYER_COUNT; role++)
        if (native_indexed_layers[role].layer_id >= 0)
            return 1;

    return 0;
}
#endif

/*
 * CONFIG_COMPOSITOR_XCM_V2 models use a single array of structures, and won't
 * override settings of the layers that we've created.
 * Left for reference only:
 *
 * struct RENDERER_LayerMedatata
 * {
 *   struct MARV* pMARV;  // pointer to layer MARV
 *    uint32_t w;   // layer width
 *    uint32_t h;   // layer height
 * };
 * extern struct RENDERER_LayerMedatata RENDERER_LayersMetadata[XIMR_MAX_LAYERS];
 *
 * ======
 *
 * On models since 850D(?), XCM_SetColorMatrixType() was added to XCM API.
 * extern uint32_t XCM_SetColorMatrixType(void * pXCM, uint layer, uint matrixType);
 * As tested on 850D: matrixType 0, 1, 2
 * 0: RGB->BT709FULL
 * 1: RGB->BT709VIDEO
 * 2: RGB->BT709FULL (?!) WINSYS_getMixColorMatrixIndex supports only 0-1
 *
 * Since XCM defaults to 0 on a new layer, we ommit this call.
 */

/*
 * Create a new VRAM (MARV) structure, alloc buffer for VRAM.
 * Call compositor to enable newly created layer.
 */
int compositor_layer_setup()
{
    // So far it seems that default GUI layer is always 0
    int newLayerID = 0;

    // Find first available, non used layer slot
    while( newLayerID < XIMR_MAX_LAYERS 
      && XCM_GetSourceSurface(_pXCM, newLayerID) != NULL)
    {
        newLayerID++;
    }

    DryosDebugMsg(0, 15, "Found %d layers", newLayerID);
    if(newLayerID >= XIMR_MAX_LAYERS)
    {
        DryosDebugMsg(0, 15, "Too many layers: %d/%d, aborting!",
                newLayerID, XIMR_MAX_LAYERS);
        return 1;
    }

    // compute our layer size
    uint32_t bmp_w = BMP_W_PLUS - BMP_W_MINUS;
    uint32_t bmp_h = BMP_H_PLUS - BMP_H_MINUS;

    // create layer
    pNewLayer = _compositor_create_layer(bmp_w, bmp_h);
    if(pNewLayer == NULL){
        DryosDebugMsg(0, 15, "Failed to create a new layer. Falling back to _rgb_vram_info");
        return 1;
    }
    /*
     * The code below seems to be R specific. RP/R6 seems to use single struct
     * for that.
     */

#ifdef CONFIG_COMPOSITOR_XCM_V1
    VMIX_Layers[newLayerID] = pNewLayer;
#ifdef CONFIG_R
    VMIX_LayersEnable[newLayerID] = 1;
#endif // CONFIG_R
#endif // CONFIG_COMPOSITOR_XCM_V1

#ifdef CONFIG_COMPOSITOR_XCM_V2
    XCM_SetSourceSurface(_pXCM, newLayerID, pNewLayer);
    XCM_SetSourceArea(_pXCM, newLayerID, -BMP_W_MINUS, -BMP_H_MINUS, BMP_W_PLUS + BMP_W_MINUS, BMP_H_PLUS + BMP_H_MINUS);
    XOC_SetLayerEnable(0, 0, newLayerID, 1); // seems layer is enabled by default
#endif // CONFIG_COMPOSITOR_XCM_V2

    // save rgb_vram_info as last step, in case something above fails.
    rgb_vram_info   = pNewLayer;
    _rgb_vram_layer_id = newLayerID;

    // erase buffer and force redraw
    rgba_buffer_clear();
    return 0;
}

/*
// TODO: Write implementationf for pure Ximr, if we ever want it.
#elif defined(CONFIG_COMPOSITOR_XIMR)
*/
#endif // CONFIG_COMPOSITOR_XCM

#endif // CONFIG_COMPOSITOR_DEDICATED_LAYER
