/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *************************************************************************** */

#include "can_reva.h"

/*********** Macros ***********/
#define MAJOR_VERSION_SHIFT             8
#define MAJOR_VERSION_CMSIS             2
#define MINOR_VERSION_CMSIS             8
#define MAJOR_VERSION_MSDK              1
#define MINOR_VERSION_MSDK              0

// Prefix these with MXC_CAN_ and put in can.h?
#define SINGLE_FILT_STD_ID_RTR_SHIFT    (4U)
#define SINGLE_FILT_STD_ID_SHIFT        (5U)
#define SINGLE_FILT_STD_ID_MASK         (0x7FFU)
#define SINGLE_FILT_STD_ID(id)          ((id & SINGLE_FILT_STD_ID_MASK) << SINGLE_FILT_STD_ID_SHIFT)
#define SINGLE_FILT_EXT_ID_RTR_SHIFT    (2U)
#define SINGLE_FILT_EXT_ID_SHIFT        (3U)
#define SINGLE_FILT_EXT_ID_MASK         (0x1FFFFFFFU)
#define SINGLE_FILT_EXT_ID(id)          ((id & SINGLE_FILT_EXT_ID_MASK) << SINGLE_FILT_EXT_ID_SHIFT)

#define DUAL_FILT_STD_ID_RTR_SHIFT      (4U)
#define DUAL_FILT_STD_ID_SHIFT          (5U)
#define DUAL_FILT_STD_ID_MASK           (0x7FFU)
#define DUAL_FILT_STD_ID(id)            ((id & DUAL_FILT_STD_ID_MASK) << DUAL_FILT_STD_ID_SHIFT)
#define DUAL_FILT_EXT_ID_SHIFT          (13U)
#define DUAL_FILT_EXT_ID_MASK           (0x1FFFE000U)
#define DUAL_FILT_EXT_ID(id)            ((id & DUAL_FILT_EXT_ID_MASK) >> DUAL_FILT_EXT_ID_SHIFT);

#define HALFWORD_BYTESWAP(half_wd)      ((half_wd & 0xFF00) >> 8) | ((half_wd & 0xFF) << 8)
#define WORD_BYTESWAP(wd)               ((wd & 0xFF000000) >> 24) | ((wd & 0xFF0000) >> 8) | ((wd & 0xFF00) << 8) | ((wd & 0xFF) << 24)


/*********** Global Variables ***********/
mxc_can_object_event_cb_t obj_evt_cb = NULL;
mxc_can_unit_event_cb_t unit_evt_cb = NULL;
static uint32_t filt_in_use[MXC_CAN_INSTANCES] = { 0 };
static mxc_can_obj_cfg_t obj_state[MXC_CAN_INSTANCES] = { MXC_CAN_OBJ_CFG_INACTIVE };
static uint32_t dma_tx0[18];
static uint32_t dma_tx1[18];
static uint32_t dma_rx0[18];
static uint32_t dma_rx1[18];

/*********** Functions ***********/
static int getNumBytes(uint32_t dlc, uint32_t fdf, uint32_t rtr)
{
    int num_bytes = 0;
    if(rtr) {
        return 0;
    }
    else if(dlc > 8 && fdf) {
        switch(dlc & 0xF) {
            case 9:
            case 10:
            case 11:
            case 12:
                num_bytes = 8 + (dlc & 0x7) * 4;
                break;
            case 13:
                num_bytes = 32;
                break;
            case 14:
                num_bytes = 48;
                break;
            case 15:
                num_bytes = 64;
                break;
        }
    }
    else if(dlc > 8 && !fdf) {
        num_bytes = 8;
    }
    else {
        num_bytes = dlc;
    }

    return num_bytes;
}

mxc_can_drv_version_t MXC_CAN_RevA_GetVersion(void)
{
    mxc_can_drv_version_t version;
    version.api = (MAJOR_VERSION_CMSIS << MAJOR_VERSION_SHIFT) | MINOR_VERSION_CMSIS;
    version.drv = (MAJOR_VERSION_MSDK << MAJOR_VERSION_SHIFT) | MINOR_VERSION_MSDK;
    return version;
}

mxc_can_capabilities_t MXC_CAN_RevA_GetCapabilities(void)
{
    mxc_can_capabilities_t capabilities;
    capabilities.num_objects = MXC_CAN_INSTANCES;
    capabilities.reentrant_operation = 1;
    capabilities.fd_mode = 1;
    capabilities.restricted_mode = 1;
    capabilities.monitor_mode = 1;
    capabilities.internal_loopback = 1;
    capabilities.external_loopback = 0;
    capabilities.rsv = 0;
    return capabilities;
}

int MXC_CAN_RevA_Init(mxc_can_unit_event_cb_t unit_cb, mxc_can_object_event_cb_t obj_cb)
{
    obj_evt_cb = obj_cb;
    unit_evt_cb = unit_cb;

    return E_NO_ERROR;
}

int MXC_CAN_RevA_UnInit(void)
{
    obj_evt_cb = NULL;
    unit_evt_cb = NULL;

    return E_NO_ERROR;
}

int MXC_CAN_RevA_PowerControl(mxc_can_pwr_ctrl_t pwr)
{
    if(pwr == MXC_CAN_PWR_CTRL_OFF) {
        return E_NO_ERROR;
    }

    mxc_can_reva_regs_t* can;
    for(int i = 0; i < MXC_CAN_INSTANCES; i++) {
    	if(obj_state[i] != MXC_CAN_OBJ_CFG_INACTIVE) {
			can = (mxc_can_reva_regs_t*) MXC_CAN_GET_CAN(i);

			if(pwr == MXC_CAN_PWR_CTRL_SLEEP) {
				MXC_CAN_SetMode(MXC_CAN_MODE_NORMAL);
				can->mode |= MXC_F_CAN_REVA_MODE_SLP;
				while(!(can->mode & MXC_F_CAN_REVA_MODE_SLP));
				MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_INACTIVE);
			}
			else if (pwr == MXC_CAN_PWR_CTRL_FULL) {
				can->mode &= ~MXC_F_CAN_REVA_MODE_SLP;
				MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_ACTIVE);
			}
    	}
    }

    return E_NO_ERROR;
}

int MXC_CAN_RevA_EnableInt(mxc_can_reva_regs_t* can, uint8_t en, uint8_t ext_en)
{
    can->inten |= en;
    can->einten |= (ext_en & (MXC_F_CAN_REVA_EINTEN_RX_TO | MXC_F_CAN_REVA_EINTEN_RX_THD));

    return E_NO_ERROR;
}

int MXC_CAN_RevA_DisableInt(mxc_can_reva_regs_t* can, uint8_t dis, uint8_t ext_dis)
{
    can->inten &= ~dis;
    can->einten &= ~(ext_dis & (MXC_F_CAN_REVA_EINTEN_RX_TO | MXC_F_CAN_REVA_EINTEN_RX_THD));

    return E_NO_ERROR;
}

int MXC_CAN_RevA_GetFlags(mxc_can_reva_regs_t* can, uint8_t* flags, uint8_t* ext_flags)
{
    *flags = can->intfl;
    *ext_flags = can->eintfl;

    return E_NO_ERROR;
}

int MXC_CAN_RevA_ClearFlags(mxc_can_reva_regs_t* can, uint8_t flags, uint8_t ext_flags)
{
    can->intfl = flags;
    can->eintfl = (ext_flags & (MXC_F_CAN_REVA_EINTFL_RX_TO | MXC_F_CAN_REVA_EINTFL_RX_THD));

    return E_NO_ERROR;
}

int MXC_CAN_RevA_GetBitRate(mxc_can_reva_regs_t* can, mxc_can_bitrate_sel_t sel, int can_clk)
{
    int num_tq = 1;
    int prescaler;

    switch(sel) {
        case MXC_CAN_BITRATE_SEL_NOMINAL:
            prescaler = ((can->bustim0 & MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV) >> MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV_POS) + 1;
            num_tq += ((can->bustim1 & MXC_F_CAN_REVA_BUSTIM1_TSEG1) >> MXC_F_CAN_REVA_BUSTIM1_TSEG1_POS) + 1;  // Find total number of time quanta
            num_tq += ((can->bustim1 & MXC_F_CAN_REVA_BUSTIM1_TSEG2) >> MXC_F_CAN_REVA_BUSTIM1_TSEG2_POS) + 1;
            break;
        case MXC_CAN_BITRATE_SEL_FD_DATA:
            prescaler = ((can->dbt_sspp & MXC_F_CAN_REVA_DBT_SSPP_DBRP) >> MXC_F_CAN_REVA_DBT_SSPP_DBRP_POS) + 1;
            num_tq += ((can->dbt_sspp & MXC_F_CAN_REVA_DBT_SSPP_DSEG1) >> MXC_F_CAN_REVA_DBT_SSPP_DSEG1_POS) + 1;
            num_tq += ((can->dbt_sspp & MXC_F_CAN_REVA_DBT_SSPP_DSEG2) >> MXC_F_CAN_REVA_DBT_SSPP_DSEG2_POS) + 1;
            break;
        default:
            return E_BAD_PARAM;
    }

    return (can_clk / prescaler) / num_tq;    // bitrate = (src_clk_freq / prescaler) / total_num_time_quanta
}

int MXC_CAN_RevA_SetBitRate(int can_clk, mxc_can_bitrate_sel_t sel, uint32_t bitrate, uint8_t seg1, uint8_t seg2, uint8_t sjw)
{
    //Find prescaler
    mxc_can_reva_regs_t* can;
    uint32_t num_tq = 1; 
    uint32_t prescaler = 1;

    for(int i = 0; i < MXC_CAN_INSTANCES; i++) {
        can = (mxc_can_reva_regs_t*) MXC_CAN_GET_CAN(i);
        can->mode |= MXC_F_CAN_REVA_MODE_RST;

        switch(sel) {
            case MXC_CAN_BITRATE_SEL_NOMINAL:
                if(seg1 > MXC_CAN_NOMINAL_MAX_SEG1TQ || seg2 > MXC_CAN_NOMINAL_MAX_SEG2TQ || sjw > MXC_CAN_NOMINAL_MAX_SJWTQ) {
                    return E_BAD_PARAM;
                }

                num_tq += seg1 + seg2;
                prescaler = can_clk / (bitrate * num_tq);
                if(prescaler > MXC_CAN_NOMINAL_MAX_PRESCALER) {
                    return E_INVALID;
                }

                MXC_SETFIELD(can->bustim0, MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV, ((prescaler-1) << MXC_F_CAN_REVA_BUSTIM0_BR_CLKDIV_POS));
                MXC_SETFIELD(can->bustim0, MXC_F_CAN_REVA_BUSTIM0_SJW, ((sjw-1) << MXC_F_CAN_REVA_BUSTIM0_SJW_POS));
                MXC_SETFIELD(can->bustim1, MXC_F_CAN_REVA_BUSTIM1_TSEG1, ((seg1-1) << MXC_F_CAN_REVA_BUSTIM1_TSEG1_POS));
                MXC_SETFIELD(can->bustim1, MXC_F_CAN_REVA_BUSTIM1_TSEG2, ((seg2-1) << MXC_F_CAN_REVA_BUSTIM1_TSEG2_POS));
                //can->fdctrl &= ~MXC_F_CAN_REVA_FDCTRL_EXTBT;

                MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NBRP, ((prescaler-1) << MXC_F_CAN_REVA_NBT_NBRP_POS));
                MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NSEG1, ((seg1-1) << MXC_F_CAN_REVA_NBT_NSEG1_POS));
                MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NSEG2, ((seg2-1) << MXC_F_CAN_REVA_NBT_NSEG2_POS));
                MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NSJW, ((sjw-1) << MXC_F_CAN_REVA_NBT_NSJW_POS));
                break;
            case MXC_CAN_BITRATE_SEL_FD_DATA:
                if(seg1 > MXC_CAN_FD_DATA_MAX_SEG1TQ || seg2 > MXC_CAN_FD_DATA_MAX_SEG2TQ || sjw > MXC_CAN_FD_DATA_MAX_SJWTQ) {
                    return E_BAD_PARAM;
                }

                num_tq += seg1 + seg2;
                prescaler = can_clk / (bitrate * num_tq);
                if(prescaler > MXC_CAN_FD_DATA_MAX_PRESCALER) {
                    return E_INVALID;
                }

                MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DBRP, ((prescaler-1) << MXC_F_CAN_REVA_DBT_SSPP_DBRP_POS));
                MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DSEG1, ((seg1-1) << MXC_F_CAN_REVA_DBT_SSPP_DSEG1_POS));
                MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DSEG2, ((seg2-1) << MXC_F_CAN_REVA_DBT_SSPP_DSEG2_POS));
                MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DSJW, ((sjw-1) << MXC_F_CAN_REVA_DBT_SSPP_DSJW_POS));
                break;
            default:
                can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
                return E_BAD_PARAM;
        }

        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
    }

    return E_NO_ERROR;
}

int MXC_CAN_RevA_SetMode(mxc_can_mode_t mode)
{
    mxc_can_reva_regs_t* can;

    if(mode < MXC_CAN_MODE_INITIALIZATION || mode > MXC_CAN_MODE_LOOPBACK_INTERNAL) {
        return E_BAD_PARAM;
    }

    for(int i = 0; i < MXC_CAN_INSTANCES; i++) {
        can = (mxc_can_reva_regs_t*) MXC_CAN_GET_CAN(i);
        can->mode |= MXC_F_CAN_REVA_MODE_RST;

        // Clear any existing operating modes
        can->fdctrl &= ~MXC_F_CAN_REVA_FDCTRL_REOM; // Should these be cleared?
        can->mode &= ~MXC_F_CAN_REVA_MODE_LOM;
        can->test &= ~MXC_F_CAN_REVA_TEST_LBEN;

        switch(mode) {
            case MXC_CAN_MODE_INITIALIZATION:
                continue;
            case MXC_CAN_MODE_NORMAL:
                break;
            case MXC_CAN_MODE_RESTRICTED:
                can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_REOM;
                break;
            case MXC_CAN_MODE_MONITOR:
                can->mode |= MXC_F_CAN_REVA_MODE_LOM;
                break;
            case MXC_CAN_MODE_LOOPBACK_INTERNAL:
                can->test |= MXC_F_CAN_REVA_TEST_LBEN;
                break;
        }

        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
    }

    return E_NO_ERROR;
}

mxc_can_obj_capabilities_t MXC_CAN_RevA_ObjectGetCapabilities(mxc_can_reva_regs_t* can)
{
    mxc_can_obj_capabilities_t obj_cap;
    obj_cap.tx = 1;
    obj_cap.rx = 1;
    obj_cap.rx_rtr_tx_data = 0;
    obj_cap.tx_rtr_rx_data = 0;
    obj_cap.multiple_filters = 1;
    obj_cap.exact_filtering = 1;
    obj_cap.mask_filtering = 1;
    obj_cap.range_filtering = 0;
    obj_cap.message_depth = 1;
    obj_cap.reserved = 0;
    return obj_cap;
}

int MXC_CAN_RevA_ObjectSetFilter(mxc_can_reva_regs_t* can, mxc_can_filt_cfg_t cfg, uint32_t id, uint32_t arg)
{
    uint32_t op_type = (cfg & MXC_CAN_FILT_OP_TYPE_MASK);
    uint32_t filt_sel = (cfg & MXC_CAN_FILT_SEL_MASK);
    uint32_t dual_filt_sel;

    if(filt_sel == MXC_CAN_FILT_CFG_DUAL_GEN) {                                                     // If using general middleware dual filter select, select filter
        if(op_type == MXC_CAN_FILT_CFG_EXACT_ADD || op_type == MXC_CAN_FILT_CFG_MASK_ADD) {
            filt_sel = ((filt_in_use[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)] + 1) * 2) << MXC_CAN_FILT_SEL_SHIFT;
        }
        else if(op_type == MXC_CAN_FILT_CFG_EXACT_DEL || op_type == MXC_CAN_FILT_CFG_MASK_DEL) {
            filt_sel = (filt_in_use[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)] * 2) << MXC_CAN_FILT_SEL_SHIFT;;
        }
    }

    can->mode |= MXC_F_CAN_REVA_MODE_RST;
    if(op_type == MXC_CAN_FILT_CFG_EXACT_ADD || op_type == MXC_CAN_FILT_CFG_MASK_ADD) {
        if(filt_in_use[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)] >= MXC_CAN_FILT_PER_OBJ) {
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            return E_NONE_AVAIL;
        }
        if(op_type == MXC_CAN_FILT_CFG_EXACT_ADD) {
            arg = 0xFFFFFFFF;
        }

        dual_filt_sel = 1;
        switch(filt_sel) {
            case MXC_CAN_FILT_CFG_DUAL1_STD_ID:
                dual_filt_sel = 0;
            case MXC_CAN_FILT_CFG_DUAL2_STD_ID:
                can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
                id = DUAL_FILT_STD_ID(id);
                can->acr16[dual_filt_sel] = HALFWORD_BYTESWAP(id);
                arg = DUAL_FILT_STD_ID((~arg)) | (1 << DUAL_FILT_STD_ID_RTR_SHIFT);
                can->amr16[dual_filt_sel] = HALFWORD_BYTESWAP(arg);
                can->amr8[1] |= 0x0F;       //Don't care about filtering data bytes in this function
                can->amr8[3] |= 0x0F;
                break;

            case MXC_CAN_FILT_CFG_DUAL1_EXT_ID:
                dual_filt_sel = 0;
            case MXC_CAN_FILT_CFG_DUAL2_EXT_ID:
                can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
                id = DUAL_FILT_EXT_ID(id);
                can->acr16[dual_filt_sel] = HALFWORD_BYTESWAP(id);
                arg = DUAL_FILT_EXT_ID((~arg));
                can->amr16[dual_filt_sel] = HALFWORD_BYTESWAP(arg);
                break;

            case MXC_CAN_FILT_CFG_SINGLE_STD_ID:
                can->mode |= MXC_F_CAN_REVA_MODE_AFM;
                id = SINGLE_FILT_STD_ID(id);
                can->acr16[0] = HALFWORD_BYTESWAP(id);
                arg = SINGLE_FILT_STD_ID((~arg)) | (1 << SINGLE_FILT_STD_ID_RTR_SHIFT);
                can->amr16[0] = HALFWORD_BYTESWAP(arg);
                can->acr16[1] = 0xFFFF;     //Don't care about filtering data bytes in this function.
                can->amr16[1] = 0xFFFF;
                break;

            case MXC_CAN_FILT_CFG_SINGLE_EXT_ID:
                can->mode |= MXC_F_CAN_REVA_MODE_AFM;
                id = SINGLE_FILT_EXT_ID(id);
                can->acr32 = WORD_BYTESWAP(id);
                arg = SINGLE_FILT_EXT_ID(~arg) | (1 << SINGLE_FILT_EXT_ID_RTR_SHIFT);
                can->amr32 = WORD_BYTESWAP(arg);
                break;

            default:
                can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
                return E_BAD_PARAM;
        }

        filt_in_use[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)]++;
    }
    else if(op_type == MXC_CAN_FILT_CFG_EXACT_DEL || op_type == MXC_CAN_FILT_CFG_MASK_DEL) {
        if(filt_in_use[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)] == 0) {
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            return E_NONE_AVAIL;
        }

        switch(filt_sel) {
            case MXC_CAN_FILT_CFG_DUAL1_STD_ID:
            case MXC_CAN_FILT_CFG_DUAL1_EXT_ID:
                can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
                can->acr16[0] = 0;
                can->amr16[0] = 0xFFFF;
                break;
            case MXC_CAN_FILT_CFG_DUAL2_STD_ID:
            case MXC_CAN_FILT_CFG_DUAL2_EXT_ID:
                can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
                can->acr16[1] = 0;
                can->amr16[1] = 0xFFFF;
                break;
            case MXC_CAN_FILT_CFG_SINGLE_STD_ID:
            case MXC_CAN_FILT_CFG_SINGLE_EXT_ID:
                can->mode |= MXC_F_CAN_REVA_MODE_AFM;
                can->acr32 = 0;
                can->amr32 = 0xFFFFFFFF;
                break;
            default:
                can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
                return E_BAD_PARAM;
        }

        filt_in_use[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)]--;
    }
    else {
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        return E_BAD_PARAM;
    }
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    return E_NO_ERROR;
}

// Signal event?, config wakeup timer?
int MXC_CAN_RevA_ObjectConfigure(mxc_can_reva_regs_t* can, mxc_can_obj_cfg_t cfg)
{
    switch(cfg) {
        case MXC_CAN_OBJ_CFG_INACTIVE:
            MXC_CAN_PowerControl(MXC_CAN_PWR_CTRL_SLEEP);
            break;
        case MXC_CAN_OBJ_CFG_TXRX:
        case MXC_CAN_OBJ_CFG_RSV:
            MXC_CAN_PowerControl(MXC_CAN_PWR_CTRL_FULL);
            can->mode |= MXC_F_CAN_REVA_MODE_RST;
            can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_FDEN | MXC_F_CAN_REVA_FDCTRL_BRSEN | MXC_F_CAN_REVA_FDCTRL_EXTBT | MXC_F_CAN_REVA_FDCTRL_ISO;
            // can->amr32 = 0xFFFFFFFF;         Default filter: all or none?
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            break;
        case MXC_CAN_OBJ_CFG_RX_RTR_TX_DATA:
        case MXC_CAN_OBJ_CFG_TX_RTR_RX_DATA:
        default:
            return E_BAD_PARAM;
    }
    obj_state[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)] = cfg;
    return E_NO_ERROR;
}

int MXC_CAN_RevA_WriteTXFIFO(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info, const uint8_t* data, uint8_t size, bool dma)
{
    uint8_t txbuf_idx = 0;
    uint32_t txbuf_data = 0;
    int num_bytes = getNumBytes(info->dlc, info->fdf, info->rtr);
    uint32_t* dma_buf;
    uint32_t dma_idx = 0;

    if(info->rtr && (info->fdf || info->brs || info->esi)) {
        return E_INVALID;
    }
    else if(!info->fdf && (info->brs || info->esi)) {
        return E_INVALID;
    }
    else if(data == NULL) {
        return E_NULL_PTR;
    }
    else if(can->txscnt < (num_bytes + 5)) {
        return E_BAD_STATE;
    }

    if(dma) {
        if(can == (mxc_can_reva_regs_t*) MXC_CAN0) {
            dma_buf = dma_tx0;
        }
        else if(can == (mxc_can_reva_regs_t*) MXC_CAN1) {
            dma_buf = dma_tx1;
        }
        else {
            return E_BAD_PARAM;
        }
    }

    // Set frame format bits/bytes
    txbuf_data =  MXC_CAN_BUF_CFG_RTR(info->rtr) | MXC_CAN_BUF_CFG_FDF(info->fdf) | MXC_CAN_BUF_CFG_BRS(info->brs) | MXC_CAN_BUF_CFG_DLC(info->dlc);
    if(info->msg_id & MXC_CAN_MSG_INFO_IDE_BIT) {
        txbuf_data |= MXC_CAN_BUF_CFG_IDE;
        txbuf_data |= MXC_CAN_BUF_CFG_EXT_ID_TX1(info->msg_id) << 8;
        txbuf_data |= MXC_CAN_BUF_CFG_EXT_ID_TX2(info->msg_id) << 16;
        txbuf_data |= MXC_CAN_BUF_CFG_EXT_ID_TX3(info->msg_id) << 24;

        if(dma) {
            dma_buf[dma_idx] = txbuf_data;
            dma_idx++;
        }
        else {
            can->txfifo32 = txbuf_data;
        }

        txbuf_data = MXC_CAN_BUF_CFG_EXT_ID_TX4(info->msg_id) | MXC_CAN_BUF_CFG_EXT_ID_ESI(info->esi);
        txbuf_idx = 1;
    }
    else {
        txbuf_data |= MXC_CAN_BUF_CFG_STD_ID_TX1(info->msg_id) << 8;
        txbuf_data |= (MXC_CAN_BUF_CFG_STD_ID_TX2(info->msg_id) | MXC_CAN_BUF_CFG_STD_ID_ESI(info->esi)) << 16;
        txbuf_idx = 3;
    }

    if(info->rtr) {         // No data bytes in RTR frames
        if(dma) {
            dma_buf[dma_idx] = txbuf_data;
            dma_idx++;
        }
        else {
            can->txfifo32 = txbuf_data;
        }
        return E_NO_ERROR;
    }

    // Set data bytes
    for(int i = 0; i < num_bytes; i++) {
        if(txbuf_idx == 0) {
            if(dma) {
                dma_buf[dma_idx] = txbuf_data;
                dma_idx++;
            }
            else {
                can->txfifo32 = txbuf_data;
            }
            txbuf_data = 0;
        }

        if(i < size) {
            txbuf_data |= data[i] << (txbuf_idx << 3);
        }

        txbuf_idx = (txbuf_idx + 1) % 4;
    }

    if(dma) {
        dma_buf[dma_idx] = txbuf_data;
        dma_idx++;
    }
    else {
        can->txfifo32 = txbuf_data;
    }

    return num_bytes;
}

int MXC_CAN_RevA_ReadRXFIFO(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info, uint8_t* data, uint8_t size, bool dma)
{
    uint8_t rxfifo_idx = 0;
    uint32_t rx_data = 0;
    uint8_t rxbuf_parse = 0;
    uint32_t* rxbuf;
    uint8_t rxbuf_idx = 0;

    if(info == NULL || data == NULL) {
        return E_NULL_PTR;
    }
    else if(!dma && can->rxdcnt == 0) {
        return E_BAD_STATE;
    }

    if(dma) {
        if(can == (mxc_can_reva_regs_t*) MXC_CAN0) {
            rxbuf = dma_rx0;
        }
        else if(can == (mxc_can_reva_regs_t*) MXC_CAN1) {
            rxbuf = dma_rx1;
        }
        else {
            return E_BAD_PARAM;
        }
    }

    // Extract status and ID bits
    if(dma) {
        rx_data = rxbuf[rxbuf_idx];
        rxbuf_idx++;
    }
    else { 
        rx_data = can->rxfifo32;
    }
    rxbuf_parse = rx_data & 0xFF;
    info->dlc = rxbuf_parse & MXC_CAN_BUF_CFG_DLC(0xF);
    info->brs = !!(rxbuf_parse & MXC_CAN_BUF_CFG_BRS(1));
    info->fdf = !!(rxbuf_parse & MXC_CAN_BUF_CFG_FDF(1));

    if(rx_data & MXC_CAN_BUF_CFG_IDE) {
        info->msg_id = MXC_CAN_MSG_INFO_IDE_BIT;

        rxbuf_parse = (rx_data & 0xFF00) >> 8;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX1(rxbuf_parse);

        rxbuf_parse = (rx_data & 0xFF0000) >> 16;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX2(rxbuf_parse);

        rxbuf_parse = (rx_data & 0xFF000000) >> 24;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX3(rxbuf_parse);

        if(dma) {
            rx_data = rxbuf[rxbuf_idx];
            rxbuf_idx++;
        }
        else { 
            rx_data = can->rxfifo32;
        }
        rxbuf_parse = rx_data & 0xFF; 
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX4(rxbuf_parse);
        info->rtr = !!(rxbuf_parse & MXC_CAN_BUF_CFG_EXT_ID_RTR(1));
        info->esi = !!(rxbuf_parse & MXC_CAN_BUF_CFG_EXT_ID_ESI(1));
        rxfifo_idx = 1;
    }
    else {
        info->rtr = !!(rxbuf_parse & MXC_CAN_BUF_CFG_STD_ID_RTR(1));

        rxbuf_parse = (rx_data & 0xFF00) >> 8;
        info->msg_id = MXC_CAN_BUF_CFG_STD_ID_RX1(rxbuf_parse);

        rxbuf_parse = (rx_data & 0xFF0000) >> 16;
        info->msg_id |= MXC_CAN_BUF_CFG_STD_ID_RX2(rxbuf_parse);
        info->esi = !!(rxbuf_parse & MXC_CAN_BUF_CFG_STD_ID_ESI(1));
        rxfifo_idx = 3;
    }

    if(info->rtr) {         // No data bytes in RTR frames
        return E_NO_ERROR;
    }

    int num_bytes = getNumBytes(info->dlc, info->fdf, info->rtr);
    int rv = num_bytes;
    for(int i = 0; i < num_bytes; i++) {
        if(rxfifo_idx == 0) {
            if(dma) {
                rx_data = rxbuf[rxbuf_idx];
                rxbuf_idx++;
            }
            else { 
                rx_data = can->rxfifo32;
            }
        }

        if(i < size) {
            data[i] = (rx_data & (0xFF << (rxfifo_idx << 3))) >> (rxfifo_idx << 3);
        }
        else {
            rv = size;
        }
        rxfifo_idx = (rxfifo_idx + 1) % 4;
    }

    return rv;
}

int MXC_CAN_RevA_MessageSend(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info, const uint8_t* data, uint8_t size)
{
    int err;

    while(!(can->stat & MXC_F_CAN_REVA_STAT_TXBUF));

    if((err = MXC_CAN_WriteTXFIFO(MXC_CAN_GET_IDX((mxc_can_regs_t*)can), info, data, size)) < E_NO_ERROR) {
        return err;
    }

    can->cmd = MXC_F_CAN_REVA_CMD_TXREQ;
    while(!(can->intfl & MXC_F_CAN_REVA_INTFL_TX)) {
        if(can->intfl & MXC_F_CAN_REVA_INTFL_ERWARN) {                       //Abort transmission/return if errors received?
            if(can->stat & MXC_F_CAN_REVA_STAT_BUS_OFF) {
                if(unit_evt_cb != NULL) {
                    MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_BUS_OFF);
                }
                can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            }
            else if(can->stat & MXC_F_CAN_REVA_STAT_ERR) {
                if(unit_evt_cb != NULL) {
                    MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_WARNING);
                }
            }
        }

        if(can->intfl & MXC_F_CAN_REVA_INTFL_ERPSV) {
            if(can->txerr > 0x80 || can->rxerr > 0x80) {
                if(unit_evt_cb != NULL) {
                    MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_PASSIVE);
                }
            }
            else {
                MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_ACTIVE);
            }
        }
    }

    if(can->intfl & MXC_F_CAN_REVA_INTFL_TX) {
        MXC_CAN_SignalObjectEvent(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), MXC_CAN_OBJ_EVT_TX_COMPLETE);
    }

    return can->intfl;
}

int MXC_CAN_RevA_MessageSendAsync(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info, const uint8_t* data, uint8_t size)
{
    int err;

    if(!(can->stat & MXC_F_CAN_REVA_STAT_TXBUF)) {
        return E_BAD_PARAM;
    }

    if((err = MXC_CAN_WriteTXFIFO(MXC_CAN_GET_IDX((mxc_can_regs_t*)can), info, data, size)) < E_NO_ERROR) {
        return err;
    }

    MXC_CAN_ClearFlags(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), (MXC_F_CAN_REVA_INTFL_ERWARN | MXC_F_CAN_REVA_INTFL_ERPSV | MXC_F_CAN_REVA_INTFL_TX), 0);  
    MXC_CAN_EnableInt(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), (MXC_F_CAN_REVA_INTEN_ERWARN | MXC_F_CAN_REVA_INTEN_ERPSV | MXC_F_CAN_REVA_INTEN_TX), 0);

    can->cmd = MXC_F_CAN_REVA_CMD_TXREQ;

    return err;
}

int MXC_CAN_RevA_MessageSendDMA(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info, const uint8_t* data, uint8_t size)
{
    int err;
    if((err = MXC_CAN_RevA_WriteTXFIFO(can, info, data, size, true)) < E_NO_ERROR) {
        return err;
    }

    int ch;
    if((ch = MXC_DMA_AcquireChannel()) < E_NO_ERROR) {
        return ch;
    }

    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;

    if(can == (mxc_can_reva_regs_t*) MXC_CAN0) {
        config.reqsel = MXC_DMA_REQUEST_CAN0TX;
        srcdst.source = dma_tx0;
    }
    else if(can == (mxc_can_reva_regs_t*) MXC_CAN1) {
        config.reqsel = MXC_DMA_REQUEST_CAN1TX;
        srcdst.source = dma_tx1;
    }
    config.ch = ch;
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = ch;
    srcdst.len = getNumBytes(info->dlc, info->fdf, info->rtr) + MXC_CAN_DMA_LEN(info->msg_id);
    if(srcdst.len % 4) {
        srcdst.len += (4 - (srcdst.len % 4));
    }
    MXC_DMA_ConfigChannel(config, srcdst);

    mxc_dma_adv_config_t advConfig;
    advConfig.ch = ch;
    advConfig.prio = MXC_DMA_PRIO_HIGH;
    advConfig.reqwait_en = 0;
    advConfig.tosel = 0;
    advConfig.pssel = 0;
    advConfig.burst_size = 4;
    MXC_DMA_AdvConfigChannel(advConfig);

    can->mode |= MXC_F_CAN_REVA_MODE_RST;
    can->mode |= MXC_F_CAN_REVA_MODE_DMA;
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    MXC_DMA_EnableInt(ch);
    MXC_DMA_Start(ch);
    MXC_DMA_SetChannelInterruptEn(ch, 0, 1);

    can->cmd = MXC_F_CAN_REVA_CMD_TXREQ;

    return E_NO_ERROR;
}

int MXC_CAN_RevA_MessageRead(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info, uint8_t* data, uint8_t size)
{
    int err = 0;

    // Check for bad parameters
    if(info == NULL) {
        return E_NULL_PTR;
    }

    while(can->rxdcnt == 0) {   // Wait for message to be received (limit wait time?)
        if(can->stat & MXC_F_CAN_REVA_STAT_DOR) {
           MXC_CAN_SignalObjectEvent(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), MXC_CAN_OBJ_EVT_RX_OVERRUN);
        }

        if(can->intfl & MXC_F_CAN_REVA_INTFL_ERWARN) {                       //return errors here?
            if(can->stat & MXC_F_CAN_REVA_STAT_BUS_OFF) {
                if(unit_evt_cb != NULL) {
                    MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_BUS_OFF);
                }
                can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            }
            else if(can->stat & MXC_F_CAN_REVA_STAT_ERR) {
                if(unit_evt_cb != NULL) {
                    MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_WARNING);
                }
            }
        }

        if(can->intfl & MXC_F_CAN_REVA_INTFL_ERPSV) {
            if(can->txerr > 0x80 || can->rxerr > 0x80) {
                if(unit_evt_cb != NULL) {
                    MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_PASSIVE);
                }
            }
            else {
                MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_ACTIVE);
            }
        }
    }


    // Check if any messages are available to read
    if(can->rxdcnt > 0) {
       if((err = MXC_CAN_ReadRXFIFO(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), info, data, size, false)) < E_NO_ERROR) {
           return err;
       }

       MXC_CAN_SignalObjectEvent(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), MXC_CAN_OBJ_EVT_RX);
    }
    else {
        return E_BAD_STATE;
    }

    return err;
}

int MXC_CAN_RevA_MessageReadAsync(mxc_can_reva_regs_t* can)
{   
    MXC_CAN_ClearFlags(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), (MXC_F_CAN_REVA_INTFL_ERWARN | MXC_F_CAN_REVA_INTFL_ERPSV | MXC_F_CAN_REVA_INTFL_RX | MXC_F_CAN_REVA_INTFL_DOR), 0); 
    MXC_CAN_EnableInt(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), (MXC_F_CAN_REVA_INTEN_ERWARN | MXC_F_CAN_REVA_INTEN_ERPSV | MXC_F_CAN_REVA_INTEN_RX | MXC_F_CAN_REVA_INTEN_DOR), 0);

    return E_NO_ERROR;
}

int MXC_CAN_RevA_MessageReadDMA(mxc_can_reva_regs_t* can, mxc_can_msg_info_t* info)
{
    int ch;
    if((ch = MXC_DMA_AcquireChannel()) < E_NO_ERROR) {
        return ch;
    }

    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;

    if(can == (mxc_can_reva_regs_t*) MXC_CAN0) {
        config.reqsel = MXC_DMA_REQUEST_CAN0RX;
        srcdst.dest = dma_rx0;
    }
    else if(can == (mxc_can_reva_regs_t*) MXC_CAN1) {
        config.reqsel = MXC_DMA_REQUEST_CAN1RX;
        srcdst.dest = dma_rx1;
    }
    config.ch = ch;
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = ch;
    srcdst.len = getNumBytes(info->dlc, info->fdf, info->rtr) + MXC_CAN_DMA_LEN(info->msg_id);
    if(srcdst.len % 4) {
        srcdst.len += (4 - (srcdst.len % 4));
    }
    MXC_DMA_ConfigChannel(config, srcdst);

    mxc_dma_adv_config_t advConfig;
    advConfig.ch = ch;
    advConfig.prio = MXC_DMA_PRIO_HIGH;
    advConfig.reqwait_en = 0;
    advConfig.tosel = 0;
    advConfig.pssel = 0;
    advConfig.burst_size = 4;
    MXC_DMA_AdvConfigChannel(advConfig);

    can->mode |= MXC_F_CAN_REVA_MODE_RST;
    can->mode |= MXC_F_CAN_REVA_MODE_DMA;
    MXC_SETFIELD(can->mode, MXC_F_CAN_REVA_MODE_RXTRIG, MXC_S_CAN_REVA_MODE_RXTRIG_1W);
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    MXC_DMA_EnableInt(ch);
    MXC_DMA_Start(ch);
    MXC_DMA_SetChannelInterruptEn(ch, 0, 1);

    return E_NO_ERROR;
}

void MXC_CAN_RevA_Handler(mxc_can_reva_regs_t* can)
{
    uint8_t flags;
    uint8_t ext_flags;

    MXC_CAN_GetFlags(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), &flags, &ext_flags);
    MXC_CAN_ClearFlags(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), flags, ext_flags);

    if(flags & MXC_F_CAN_REVA_INTFL_ERWARN) {                       //Abort transmission if errors received?
        if(can->stat & MXC_F_CAN_REVA_STAT_BUS_OFF) {
            if(unit_evt_cb != NULL) {
                MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_BUS_OFF);
            }
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        }
        else if(can->stat & MXC_F_CAN_REVA_STAT_ERR) {
            if(unit_evt_cb != NULL) {
                MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_WARNING);
            }
        }
    }

    if(flags & MXC_F_CAN_REVA_INTFL_ERPSV) {
        if(can->txerr > 0x80 || can->rxerr > 0x80) {
            if(unit_evt_cb != NULL) {
                MXC_CAN_SignalUnitEvent(MXC_CAN_UNIT_EVT_PASSIVE);
            }
        }
    }

    if(flags & MXC_F_CAN_REVA_INTFL_TX) {
        if(obj_evt_cb != NULL) {
            MXC_CAN_SignalObjectEvent(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), MXC_CAN_OBJ_EVT_TX_COMPLETE);
        }
    }

    if(flags & MXC_F_CAN_REVA_INTFL_RX) {
        if(obj_evt_cb != NULL) {
            MXC_CAN_SignalObjectEvent(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), MXC_CAN_OBJ_EVT_RX);
        }
    }

    if(flags & MXC_F_CAN_REVA_INTFL_DOR) {
        if(obj_evt_cb != NULL) {
            MXC_CAN_SignalObjectEvent(MXC_CAN_GET_IDX((mxc_can_regs_t*) can), MXC_CAN_OBJ_EVT_RX_OVERRUN);
        }
    }
}

int MXC_CAN_RevA_Control(mxc_can_ctrl_t ctrl, uint32_t ctrl_arg)
{
    mxc_can_reva_regs_t* can;

    if(ctrl == MXC_CAN_CTRL_SET_FD_MODE) {
        return E_NO_ERROR;
    }
    else if(ctrl == MXC_CAN_CTRL_ABORT_TX) {
        can = (mxc_can_reva_regs_t*) MXC_CAN_GET_CAN(ctrl_arg);
        if(can == 0) {
            return E_BAD_PARAM;
        }

        can->cmd = MXC_F_CAN_REVA_CMD_ABORT;
    }
    else if(ctrl == MXC_CAN_CTRL_RETRANSMISSION) {
        for(int i = 0; i < MXC_CAN_INSTANCES; i++) {
            can = (mxc_can_reva_regs_t*) MXC_CAN_GET_CAN(i);
            can->mode |= MXC_F_CAN_REVA_MODE_RST;
            if(ctrl_arg) {
                can->fdctrl &= ~MXC_F_CAN_REVA_FDCTRL_DAR;
            }
            else {
                can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_DAR;
            }
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        }
    }
    else if(ctrl == MXC_CAN_CTRL_TRANSCEIVER_DLY) {
        for(int i = 0; i < MXC_CAN_INSTANCES; i++) {
            can = (mxc_can_reva_regs_t*) MXC_CAN_GET_CAN(i);
            can->mode |= MXC_F_CAN_REVA_MODE_RST;
            can->txdecmp = MXC_F_CAN_REVA_TXDECMP_TDCEN | (ctrl_arg & MXC_F_CAN_REVA_TXDECMP_TDCO);
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        }
    }
    else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_CAN_RevA_SetWakeupTimer(mxc_can_reva_regs_t* can, uint8_t prescaler, uint16_t wup_filter_tm, uint32_t wup_expire_tm)
{
    can->mode |= MXC_F_CAN_REVA_MODE_RST;
    MXC_SETFIELD(can->wupclkdiv, MXC_F_CAN_REVA_WUPCLKDIV_WUPDIV, prescaler);
    MXC_SETFIELD(can->wupft, MXC_F_CAN_REVA_WUPFT_WUPFT, wup_filter_tm);
    MXC_SETFIELD(can->wupet, MXC_F_CAN_REVA_WUPET_WUPET, wup_expire_tm);
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    return E_NO_ERROR;
}

mxc_can_stat_t MXC_CAN_RevA_GetStatus(mxc_can_reva_regs_t* can)
{
    mxc_can_stat_t stat;

    if(obj_state[MXC_CAN_GET_IDX((mxc_can_regs_t*) can)] == MXC_CAN_OBJ_CFG_INACTIVE) {
        stat.unit_state = MXC_CAN_UNIT_STATE_INACTIVE;
    }
    else if(can->stat & MXC_F_CAN_REVA_STAT_BUS_OFF) {
        stat.unit_state = MXC_CAN_UNIT_STATE_BUS_OFF;
    }
    else if((can->mode & MXC_F_CAN_REVA_MODE_LOM) || (can->fdctrl & MXC_F_CAN_REVA_FDCTRL_REOM)) {
        stat.unit_state = MXC_CAN_UNIT_STATE_PASSIVE;
    }
    else {
        stat.unit_state = MXC_CAN_UNIT_STATE_ACTIVE;
    }

    stat.tx_err_cnt = can->txerr;
    stat.rx_err_cnt = can->rxerr;
    stat.last_error_code = can->ecc;

    switch(stat.last_error_code & MXC_CAN_ECC_ERROR_CODE_MASK) {
        case MXC_F_CAN_REVA_ECC_BER:
            stat.last_error_code = MXC_CAN_LEC_BIT_ERR;
            break;
        case MXC_F_CAN_REVA_ECC_STFER:
            stat.last_error_code = MXC_CAN_LEC_STUFF_ERR;
            break;
        case MXC_F_CAN_REVA_ECC_CRCER:
            stat.last_error_code = MXC_CAN_LEC_CRC_ERR;
            break;
        case MXC_F_CAN_REVA_ECC_FRMER:
            stat.last_error_code = MXC_CAN_LEC_FORM_ERR;
            break;
        case MXC_F_CAN_REVA_ECC_ACKER:
            stat.last_error_code = MXC_CAN_LEC_ACK_ERR;
            break;
        default:
            stat.last_error_code = MXC_CAN_LEC_NO_ERR;
    }

    return stat;
}

void MXC_CAN_RevA_SignalUnitEvent(mxc_can_unit_evt_t event)
{
    if(unit_evt_cb != NULL) {
        unit_evt_cb(event);
    }
}

void MXC_CAN_RevA_SignalObjectEvent(uint32_t can_idx, mxc_can_obj_evt_t event)
{
    if(obj_evt_cb != NULL) {
        obj_evt_cb(can_idx, event);
    }
}
