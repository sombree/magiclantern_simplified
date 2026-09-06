/** \file
 * Function overrides needed for RP 1.6.0
 */
/*
 * Copyright (C) 2021 Magic Lantern Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <dryos.h>
#include <property.h>
#include <bmp.h>
#include <config.h>
#include <consts.h>
#include <lens.h>
#include <edmac.h>

// fake WINSYS_BMP_DIRTY_BIT_NEG
int winsys_bmp_dirty_bit_neg = 0;

/** GUI **/
//see comments in stub.S
void gui_init_end(void){ }

//Not sure if all args are uint.
extern void gui_enqueue_message(uint32_t, uint32_t, uint32_t, uint32_t);
void GUI_Control(int bgmt_code, int obj, int arg, int unknown){
    gui_enqueue_message(0, bgmt_code, obj, arg);
}

void LoadCalendarFromRTC(struct tm *tm)
{
    _LoadCalendarFromRTC(tm, 0, 0, 16);
}

int get_task_info_by_id(int unknown_flag, int task_id, void *task_attr)
{
    // task_id is something like two u16s concatenated.  The flag argument,
    // present on D45 but not on D678 allows controlling if the task info request
    // uses the whole thing, or only the low half.
    //
    // ML calls with this set to 1, meaning task_id is used as is,
    // if 0, the high half is masked out first.
    //
    // D678 doesn't have the 1 option, we use the low half as index
    // to find the full value.
    struct task *task = first_task + (task_id & 0xffff);
    return _get_task_info_by_id(task->taskId, task_attr);
}

#ifdef CONFIG_COMPOSITOR_XCM_NATIVE_INDEXED
/** Native indexed XCM platform policy. */
#define RP_XCM_LUT_PTR_OFFSET  0x00u
#define RP_XCM_LUT_TYPE_OFFSET 0x04u
#define RP_CANON_OSD_LAYER_ID  0u
#define RP_CANON_AF_LAYER_ID   1u

extern uint32_t XCM_SetLUT(void *pXCM, void *lut, uint32_t range);
extern uint32_t XOC_SetLayerEnable(int p1, int p2, uint layer, int p4);

static uint32_t rp_native_saved_lut_ptr = 0;
static uint32_t rp_native_saved_lut_type = 0;
static uint32_t rp_native_lut_ptr = 0;
static int rp_native_lut_saved = 0;
static struct MARV *rp_native_menu_owner[2] = { NULL, NULL };
static int rp_native_menu_hidden = 0;

static uint32_t rp_xcm_root_u32(uint32_t offset)
{
    return *(volatile uint32_t *)((uintptr_t)_pXCM + offset);
}

int compositor_native_indexed_platform_lut_install(void *lut)
{
    if (!_pXCM || !lut)
        return 1;

    uint32_t lut_ptr = (uint32_t)(uintptr_t)lut;

    if (!rp_native_lut_saved)
    {
        uint32_t current_ptr = rp_xcm_root_u32(RP_XCM_LUT_PTR_OFFSET);
        uint32_t current_type = rp_xcm_root_u32(
            RP_XCM_LUT_TYPE_OFFSET);

        /* Do not replace a LUT installed by another XCM client. */
        if (current_ptr != 0)
            return 2;

        rp_native_saved_lut_ptr = current_ptr;
        rp_native_saved_lut_type = current_type;
        rp_native_lut_ptr = lut_ptr;
        rp_native_lut_saved = 1;
    }

    uint32_t current_ptr = rp_xcm_root_u32(RP_XCM_LUT_PTR_OFFSET);
    uint32_t current_type = rp_xcm_root_u32(RP_XCM_LUT_TYPE_OFFSET);

    if (current_ptr == lut_ptr)
        return 0;

    if (current_ptr != rp_native_saved_lut_ptr ||
        current_type != rp_native_saved_lut_type)
        return 3;

    int rc = (int)XCM_SetLUT(_pXCM, lut, 0);
    if (rc != 0 || rp_xcm_root_u32(RP_XCM_LUT_PTR_OFFSET) != lut_ptr)
        return 4;

    return 0;
}

void compositor_native_indexed_platform_lut_restore(void)
{
    if (!_pXCM || !rp_native_lut_saved)
        return;

    if (rp_xcm_root_u32(RP_XCM_LUT_PTR_OFFSET) == rp_native_lut_ptr)
    {
        XCM_SetLUT(
            _pXCM, (void *)(uintptr_t)rp_native_saved_lut_ptr,
            rp_native_saved_lut_type);
    }

    rp_native_saved_lut_ptr = 0;
    rp_native_saved_lut_type = 0;
    rp_native_lut_ptr = 0;
    rp_native_lut_saved = 0;
}

void compositor_native_indexed_platform_menu_enter(void)
{
    if (!_pXCM)
        return;

    rp_native_menu_owner[0] = XCM_GetSourceSurface(
        _pXCM, RP_CANON_OSD_LAYER_ID);
    rp_native_menu_owner[1] = XCM_GetSourceSurface(
        _pXCM, RP_CANON_AF_LAYER_ID);
    rp_native_menu_hidden = 1;

    if (rp_native_menu_owner[0])
        XOC_SetLayerEnable(0, 0, RP_CANON_OSD_LAYER_ID, 0);
    if (rp_native_menu_owner[1])
        XOC_SetLayerEnable(0, 0, RP_CANON_AF_LAYER_ID, 0);

    DryosDebugMsg(0, 15, "RP XCM menu enter C0=%x C1=%x",
        (uint32_t)(uintptr_t)rp_native_menu_owner[0],
        (uint32_t)(uintptr_t)rp_native_menu_owner[1]);
}

void compositor_native_indexed_platform_menu_exit(void)
{
    if (!_pXCM || !rp_native_menu_hidden)
        return;

    if (rp_native_menu_owner[0] &&
        XCM_GetSourceSurface(_pXCM, RP_CANON_OSD_LAYER_ID) ==
            rp_native_menu_owner[0])
    {
        XOC_SetLayerEnable(0, 0, RP_CANON_OSD_LAYER_ID, 1);
    }

    if (rp_native_menu_owner[1] &&
        XCM_GetSourceSurface(_pXCM, RP_CANON_AF_LAYER_ID) ==
            rp_native_menu_owner[1])
    {
        XOC_SetLayerEnable(0, 0, RP_CANON_AF_LAYER_ID, 1);
    }

    rp_native_menu_owner[0] = NULL;
    rp_native_menu_owner[1] = NULL;
    rp_native_menu_hidden = 0;
    DryosDebugMsg(0, 15, "RP XCM menu exit");
}

void compositor_native_indexed_platform_menu_reassert(void)
{
    if (!_pXCM || !rp_native_menu_hidden)
        return;

    if (rp_native_menu_owner[0] &&
        XCM_GetSourceSurface(_pXCM, RP_CANON_OSD_LAYER_ID) ==
            rp_native_menu_owner[0])
    {
        XOC_SetLayerEnable(0, 0, RP_CANON_OSD_LAYER_ID, 0);
    }

    if (rp_native_menu_owner[1] &&
        XCM_GetSourceSurface(_pXCM, RP_CANON_AF_LAYER_ID) ==
            rp_native_menu_owner[1])
    {
        XOC_SetLayerEnable(0, 0, RP_CANON_AF_LAYER_ID, 0);
    }
}
#endif

/** File I/O **/

/**
 * _FIO_GetFileSize returns now 64bit size in form of struct.
 * This probably should be integrated into fio-ml for CONFIG_DIGIC_VIII
 */
extern int _FIO_GetFileSize64(const char *, void *);
int _FIO_GetFileSize(const char * filename, uint32_t * size){
    uint32_t size64[2];
    int code = _FIO_GetFileSize64(filename, &size64);
    *size = size64[0]; //return "lower" part
    return code;
}

/** WRONG: temporary overrides to get CONFIG_HELLO_WORLD working **/

void SetEDmac(unsigned int channel, void *address, struct edmac_info *ptr, int flags)
{
    return;
}

void ConnectWriteEDmac(unsigned int channel, unsigned int where)
{
    return;
}

void ConnectReadEDmac(unsigned int channel, unsigned int where)
{
    return;
}

void StartEDmac(unsigned int channel, int flags)
{
    return;
}

void AbortEDmac(unsigned int channel)
{
    return;
}

void RegisterEDmacCompleteCBR(int channel, void (*cbr)(void*), void* cbr_ctx)
{
    return;
}

void UnregisterEDmacCompleteCBR(int channel)
{
    return;
}

void RegisterEDmacAbortCBR(int channel, void (*cbr)(void*), void* cbr_ctx)
{
    return;
}

void UnregisterEDmacAbortCBR(int channel)
{
    return;
}

void RegisterEDmacPopCBR(int channel, void (*cbr)(void*), void* cbr_ctx)
{
    return;
}

void UnregisterEDmacPopCBR(int channel)
{
    return;
}

void _EngDrvOut(uint32_t reg, uint32_t value)
{
    return;
}

uint32_t shamem_read(uint32_t addr)
{
    return 0;
}

void _engio_write(uint32_t* reg_list)
{
    return;
}

unsigned int UnLockEngineResources(struct LockEntry *lockEntry)
{
    return 0;
}
