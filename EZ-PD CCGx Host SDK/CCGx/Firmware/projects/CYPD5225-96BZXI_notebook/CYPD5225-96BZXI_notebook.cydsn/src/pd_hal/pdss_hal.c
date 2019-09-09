/**
 * @file pdss_hal.c
 *
 * @brief @{CCG PD PHY driver module source file for CCG3 and CCG4 devices.@}
 */

/*
 *
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#include <config.h>
#include <hal_ccgx.h>
#include <pd.h>
#include <pd_protocol.h>
#include <dpm_intern.h>
#include <dpm.h>
#include <typec_manager.h>
#include <pdss_hal.h>
#include <ccgx_regs.h>
#include <status.h>
#include <gpio.h>
#include <hpd.h>
#include <timer.h>
#include <utils.h>
#include <chgb_hal.h>
#include <battery_charging.h>

/*
 * Ignore mask for Rev2 PD header
 */
#define PD_MSG_HDR_REV2_IGNORE_MASK   (0x8010)

/* INTR3 filter configurations. */
#define PDSS_INTR3_FILT_SEL_4_CLK_LF 1
#define PDSS_INTR3_FILT_SEL_5_CLK_LF 2
#define PDSS_INTR3_FILT_SEL_6_CLK_LF 3

#define CSA_GAIN            (1u)
#define CSA_VREF_SEL        (0x3F)

#define LF_CLK_CYCLE_US     (31u)

/*
 * Bit field that controls the PCTRL and CCTRL FETs.
 * CCG3 silicon has PCTRL FETs flipped (silicon defect).
 */

#ifdef CCG3
#define VBUS_P_NGDO_EN_LV_0 (1 << 13)
#define VBUS_P_NGDO_EN_LV_1 (1 << 12)
#else /* CCG4P */
#define VBUS_P_NGDO_EN_LV_0 (1 << 12)
#define VBUS_P_NGDO_EN_LV_1 (1 << 13)
#endif /* CCG3 */
#define VBUS_C_NGDO_EN_LV_0 (1 << 3)
#define VBUS_C_NGDO_EN_LV_1 (1 << 4)

#ifdef CCG3
#define VBUS_P_PLDN_EN_LV_0 (1 << 15)
#define VBUS_P_PLDN_EN_LV_1 (1 << 14)
#else /* CCG4P */
#define VBUS_P_PLDN_EN_LV_0 (1 << 14)
#define VBUS_P_PLDN_EN_LV_1 (1 << 15)
#endif /* CCG3 */
#define VBUS_C_PLDN_EN_LV_0 (1 << 5)
#define VBUS_C_PLDN_EN_LV_1 (1 << 6)

/*
 *  Input ladder voltages, code limits and step sizes from Table 26 of the
 *  HardIP BROS (001-98391).
 */
#define UVOV_LADDER_BOT     (2750u)
#define UVOV_LADDER_MID     (9000u)
#define UVOV_LADDER_TOP     (21500u)
#define UVOV_CODE_BOT       (0)
#define UVOV_CODE_MID       (25u)
#define UVOV_CODE_TOP       (50u)
#define UVOV_CODE_MAX       (63u)
#define UVOV_CODE_6V0       (13u)
#define UVOV_LO_STEP_SZ     (250u)
#define UVOV_HI_STEP_SZ     (500u)

/* VCONN leaker control value. */
#define PDSS_VCONN_LEAKER_SEL_200_UA_VAL        (1u)

/*
 * This is the default value to be used for HPD_CTRL1 register.
 * This value translates to:
 * 0x096: IRQ MIN time (0.25 ms)
 * 0x500: IRQ MAX time (2.2 ms)
 * 0: FLUSH_QUEUE
 * 0: LOOPBACK_EN
 * 1: RESET_HPD_STATE
 */
#define PDSS_HPD_CTRL1_DEFAULT_VALUE            (0x80530096)

/*
 * This is the default value to be used for HPD_CTRL3 register.
 * HPD Block detects IRQ as LOW across deepsleep if IRQ width
 * is in 1.8 - 2ms range. Therefore STABLE LOW minimum time is
 * pushed out to 2.2ms instead of 2ms.
 */
#define PDSS_HPD_CTRL3_DEFAULT_VALUE            (0x0005304B)

/* PDSS DDFT mux configuration source values. */
#define PDSS_DDFT_MUX_NCELL_DDFT0_VALUE         (38)
#define PDSS_DDFT_MUX_NCELL_DDFT1_VALUE         (39)

/* PDSS NCELL DDFT mux configuration source values. */
#define PDSS_NCELL_MUX_VBUS_C_PLDN_EN_0_VALUE   (23)
#define PDSS_NCELL_MUX_VBUS_C_PLDN_EN_1_VALUE   (24)
#define PDSS_NCELL_MUX_VBUS_P_PLDN_EN_0_VALUE   (27)
#define PDSS_NCELL_MUX_VBUS_P_PLDN_EN_1_VALUE   (28)

/* Rp trim table address on old CCG silicon. */
#define SFLASH_PD0_RPTRIM_ADDR_OLD     (0x0FFFF079)
#define SFLASH_PD1_RPTRIM_ADDR_OLD     (0x0FFFF085)

/* Rp trim table address on new CCG silicon. */
#ifdef CCG3

#define SFLASH_PD_RPTRIM_TABLE_ADDR    (0x0FFFF280)
#define SFLASH_PD0_RPTRIM_ADDR_NEW     (0x0FFFF282)
#define SFLASH_PD1_RPTRIM_ADDR_NEW     (0x0FFFF285)

#else /* !CCG3 */

#ifdef CCG4
#define SFLASH_PD_RPTRIM_TABLE_ADDR    (0x0FFFF500)
#define SFLASH_PD0_RPTRIM_ADDR_NEW     (0x0FFFF502)
#define SFLASH_PD1_RPTRIM_ADDR_NEW     (0x0FFFF505)
#else
#error "Unsupported device family."
#endif /* CCG4 */

#endif /* CCG3 */

/* Signature that indicates validity of new Rp trim table. */
#define SFLASH_PD_RPTRIM_TABLE_SIG0    (0xE5)
#define SFLASH_PD_RPTRIM_TABLE_SIG1    (0xAD)

/* Ordered sets for transmission. */
const uint32_t os_table[SOP_INVALID] =
{
    0x8E318u,      /**< SOP Default. */
    0x31B18u,      /**< SOP Prime. */
    0x360D8u,      /**< SOP Double Prime. */
    0x34F98u,      /**< SOP Prime Debug. */
    0x89A78u,      /**< SOP Double Prime Debug. */
    0xE7393u,      /**< Hard Reset. */
    0xE0F8Cu       /**< Cable Reset. */
};

/* Swap CNTRL default settings to 0.52V reference */
#define SWAP_CNTRL0_DFLT_VAL                    (2u << PDSS_SWAP_CNTRL_0_CMP_FS_VSEL_POS)

/* Swap CTRL default settings for FRS receive. This settings are based on 5Mhz clock
 * to the block.
 */
#define FRS_RX_SWAP_CTRL0_SWAPR_SOURCE_SEL_POS  (2u)

#define FRS_RX_SWAP_CTRL1_DFLT_VAL              ((175u << PDSS_SWAP_CTRL1_IRQ_MIN_POS)| \
        (650u << PDSS_SWAP_CTRL1_IRQ_MAX_POS)|\
        (PDSS_SWAP_CTRL1_RESET_SWAP_STATE))

#define FRS_RX_SWAP_CTRL2_DFLT_VAL              ((40u << PDSS_SWAP_CTRL2_GLITCH_WIDTH_LOW_POS)| \
        (1u << PDSS_SWAP_CTRL2_GLITCH_WIDTH_HIGH_POS))

#define FRS_RX_SWAP_CTRL3_DFLT_VAL              ((300u << PDSS_SWAP_CTRL3_STABLE_LOW_POS)|\
        (1200u << PDSS_SWAP_CTRL3_STABLE_HIGH_POS))

#define FRS_RX_SWAP_CTRL4_DFLT_VAL              (10u << PDSS_SWAP_CTRL4_IRQ_SPACING_POS)

#define FRS_RX_SWAP_CTRL5_DFLT_VAL              ((25u << PDSS_SWAP_CTRL5_LONG_HIGH_POS) |\
        (1200u << PDSS_SWAP_CTRL5_LONG_LOW_POS))

/* Callback used for notification of any input supply change. */
pd_supply_change_cbk_t gl_ccg_supply_changed_cb = NULL;

/**
 * @struct pdss_status_t
 * @brief Structure to hold PDSS IP status.
 */
typedef struct
{
    /** PD phy callback. */
    pd_phy_cbk_t pd_phy_cbk;

    /** The received PD packet. */
    pd_packet_extd_t rx_pkt;

    /** The tx data pointer. */
    uint32_t* tx_dat_ptr;

    /** ADC block variables. */
    volatile uint16_t adc_vddd_mv[PD_ADC_NUM_ADC];

    /** ADC callback. */
    PD_ADC_CB_T adc_cb[PD_ADC_NUM_ADC];

    /** The tx data count. */
    uint8_t tx_dobj_count;

    /** The tx data pointer. */
    uint8_t volatile tx_obj_sent;

    /* Holds current transmission is unchunked or not */
    uint8_t volatile tx_unchunked;

    /** Holds retry count. */
    int8_t volatile retry_cnt;

    /**
     * Flag to indicate a message has been transmitted and we're waiting for
     * GoodCRC.
     */
    uint8_t volatile tx_done;

    /**
     * Flag to indicate currently received message is unchunked extended message
     */
    bool volatile rx_unchunked;

    /**
     * Length of currently being received extended unchunked messages in 32 bits units
     */
    uint8_t volatile rx_unchunk_len;

    /**
     * Count in 32 bits units of no of words read from rx memory for extended unchunked
     * message.
     */
    uint8_t volatile rx_unchunk_count;

    /**
     * Read memory location where the HAL should read the next portion of data from.
     */
    uint8_t volatile rx_read_location;

#if BATTERY_CHARGING_ENABLE
    /** PD phy callback. */
    bc_phy_cbk_t bc_phy_cbk;
    int volatile bc_qc_pulse_count;
#endif /* BATTERY_CHARGING_ENABLE */

} pdss_status_t;

#ifdef CCG4
/** Pointer array for HW IP register structure. */
static PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
{
    PDSS0
#if CCG_PD_DUALPORT_ENABLE
        ,
    PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
};
#else /* CCG3 */
static PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] = {PDSS};
#endif /* CCG4/CCG3 */

#ifdef CCG3
/* OVP callback pointer storage. */
PD_ADC_CB_T gl_ccg3_ovp_cb;

#if VBUS_OVP_ENABLE
/* VBus OVP mode. */
static vbus_ovp_mode_t gl_vbus_ovp_mode;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_OCP_ENABLE
/* VBus OCP mode. */
static uint8_t gl_vbus_ocp_mode;

/* VBus OCP software debounce in ms. */
static uint8_t gl_ocp_sw_db_ms;
#endif /* VBUS_OCP_ENABLE */

/* SBU1 and SBU2 switch state. */
sbu_switch_state_t gl_sbu1_state;
sbu_switch_state_t gl_sbu2_state;

/* AUX1 and AUX2 resistor configuration. */
aux_resistor_config_t gl_aux1_config = AUX_NO_RESISTOR;
aux_resistor_config_t gl_aux2_config = AUX_NO_RESISTOR;
#endif /* CCG3 */

/** PDSS status. */
static pdss_status_t gl_pdss_status[NO_OF_TYPEC_PORTS];

#if (VCONN_OCP_ENABLE) && (CCG4_DOCK)
extern void app_vconn_ocp_cbk(uint8_t port, bool comp_out);
#endif /* VCONN_OCP_ENABLE && CCG4_DOCK */

/**
 * Type C voltage thresholds (in mV) as per Section 4.11.3 of Type C
 * specification Rev1.
 */
const uint8_t thresholds[4][4] =
{
    {PD_CMP_VSEL_0_2_V, PD_CMP_VSEL_1_77_V, 0, 0}, /* Rp USB default row. */
    {PD_CMP_VSEL_0_4_V, PD_CMP_VSEL_1_77_V, 0, 0}, /* Rp 1.5A row. */
    {PD_CMP_VSEL_0_8_V, PD_CMP_VSEL_2_6_V, 0, 0}, /* Rp 3A row. */
    {PD_CMP_VSEL_0_2_V, PD_CMP_VSEL_0_655_V, PD_CMP_VSEL_1_235_V, PD_CMP_VSEL_2_6_V} /* RD row. */
};

void pdss_intr0_handler(uint8_t port);
CY_ISR_PROTO(pdss_port0_intr0_handler);
CY_ISR_PROTO(pdss_port1_intr0_handler);

void pdss_intr1_handler(uint8_t port);
CY_ISR_PROTO(pdss_port0_intr1_handler);
CY_ISR_PROTO(pdss_port1_intr1_handler);

/* HPD transmit enable per PD port. */
static bool hpd_transmit_enable[NO_OF_TYPEC_PORTS] =
{
    false
#if CCG_PD_DUALPORT_ENABLE
        ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* HPD receive enable per PD port. */
static bool hpd_receive_enable[NO_OF_TYPEC_PORTS] =
{
    false
#if CCG_PD_DUALPORT_ENABLE
        ,
    false
#endif
};

/* HPD event callback per PD port. */
static hpd_event_cbk_t hpd_cbks[NO_OF_TYPEC_PORTS] =
{
    NULL
#if CCG_PD_DUALPORT_ENABLE
        ,
    NULL
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* Configuration provided from solution. */
static PD_ADC_INPUT_T   pd_vbus_detach_adc_input    = PD_ADC_INPUT_AMUX_A;
static PD_ADC_ID_T      pd_vbus_detach_adc_id       = PD_ADC_ID_1;
static bool             pd_vbus_mon_internal        = false;
static uint8_t          pd_vbus_mon_divider         = 11u;
static uint8_t          pd_ngdo_spacing             = 0x0au;

#ifdef CCG3
static bool             pd_dual_fet                 = true;
static pd_fet_dr_t      pd_pctrl_drive              = PD_FET_DR_N_JN_FET;
static pd_fet_dr_t      pd_cctrl_drive              = PD_FET_DR_N_JN_FET;
#else /* CCG4 */
static bool             pd_dual_fet                 = false;
static pd_fet_dr_t      pd_pctrl_drive              = PD_FET_DR_ACTIVE_HIGH;
static pd_fet_dr_t      pd_cctrl_drive              = PD_FET_DR_ACTIVE_LOW;
#endif /* CCG */

void pd_hal_set_cc_ovp_pending(uint8_t port)
{
    /* Not supported in this HAL. */
}

uint16_t pd_hal_measure_vbus(uint8_t port)
{
    uint8_t level;

    pd_adc_calibrate (port, APP_VBUS_POLL_ADC_ID);
    level = pd_adc_sample(port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);

    return pd_adc_get_vbus_voltage(port, APP_VBUS_POLL_ADC_ID, level);
}

PD_ADC_ID_T pd_hal_get_vbus_detach_adc(void)
{
    return pd_vbus_detach_adc_id;
}

PD_ADC_INPUT_T pd_hal_get_vbus_detach_input(void)
{
    return pd_vbus_detach_adc_input;
}

void pd_hal_set_vbus_detach_params(PD_ADC_ID_T adc_id, PD_ADC_INPUT_T adc_inp)
{
    pd_vbus_detach_adc_input = adc_inp;
    pd_vbus_detach_adc_id    = adc_id;
}

/* Function to register callbacks for input supply change. */
void pd_hal_set_supply_change_evt_cb(pd_supply_change_cbk_t cb)
{
    gl_ccg_supply_changed_cb = cb;
}

void pd_hal_set_vbus_mon_divider(uint8_t divider)
{
    pd_vbus_mon_divider = divider;
}

void pd_hal_enable_internal_vbus_mon(bool enable)
{
    pd_vbus_mon_internal = enable;
}

void pd_hal_set_fet_drive(pd_fet_dr_t pctrl_drive, pd_fet_dr_t cctrl_drive)
{
    pd_pctrl_drive = pctrl_drive;
    pd_cctrl_drive = cctrl_drive;
}

void pd_hal_dual_fet_config(bool dual_fet, uint8_t spacing)
{
    pd_dual_fet = dual_fet;
    pd_ngdo_spacing = spacing;
}

/* Function definitions. */

CY_ISR(pdss_port0_intr0_handler)
{
    pdss_intr0_handler(TYPEC_PORT_0_IDX);
}

CY_ISR(pdss_port1_intr0_handler)
{
    pdss_intr0_handler(TYPEC_PORT_1_IDX);
}

CY_ISR(pdss_port0_intr1_handler)
{
    pdss_intr1_handler(TYPEC_PORT_0_IDX);
}

CY_ISR(pdss_port1_intr1_handler)
{
    pdss_intr1_handler(TYPEC_PORT_1_IDX);
}

ccg_status_t pd_hal_init(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t tmp;

    if (port == TYPEC_PORT_0_IDX)
    {
        /* Register sync interrupt handler. */
        CyIntDisable(PD_PORT0_INTR0);
        (void)CyIntSetVector(PD_PORT0_INTR0, &pdss_port0_intr0_handler);
        CyIntEnable(PD_PORT0_INTR0);

        /* Register ganged interrupt handler. */
        CyIntDisable(PD_PORT0_INTR1);
        (void)CyIntSetVector(PD_PORT0_INTR1, &pdss_port0_intr1_handler);
        CyIntEnable(PD_PORT0_INTR1);
    }
    else
    {
        /* Register sync interrupt handler. */
        CyIntDisable(PD_PORT1_INTR0);
        (void)CyIntSetVector(PD_PORT1_INTR0, &pdss_port1_intr0_handler);
        CyIntEnable(PD_PORT1_INTR0);

        /* Register ganged interrupt handler. */
        CyIntDisable(PD_PORT1_INTR1);
        (void)CyIntSetVector(PD_PORT1_INTR1, &pdss_port1_intr1_handler);
        CyIntEnable(PD_PORT1_INTR1);
    }

#ifdef CCG3
    /* Settings as recommended by design */
    pd->vreg_ctrl |= PDSS_VREG_CTRL_VREG_ISO_N;
    pd->vsys_ctrl |= PDSS_VSYS_CTRL_VSYS_ISO_N;
#endif /* CCG3 */

    /* Enable the PD block. */
    pd->ctrl |= PDSS_CTRL_IP_ENABLED;

    /* Power up the block. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;

    /* Turn off PHY deepsleep. References require 100us to stabilize. */
    pd->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;

    /*
     * Enable deep-sleep current reference outputs.
     * NOTE: This is necessary for CCG3 but not for CCG4.
     */
    pd->dpslp_ref_ctrl |= PDSS_DPSLP_REF_CTRL_IGEN_EN;
    CyDelayUs(100);

    /* Set trim settings. */
    tmp = pd->s8usbpd_trim_6 & ~PDSS_S8USBPD_TRIM_6_V1P575_TRIM_MASK;
    tmp |= SILICON_TRIM6_V1P575_TRIM_VALUE;
    pd->s8usbpd_trim_6 = tmp;

    tmp = pd->s8usbpd_trim_3 & ~PDSS_S8USBPD_TRIM_3_V0P55_TRIM_MASK;
    tmp |= SILICON_TRIM3_V0P55_TRIM_VALUE;
    pd->s8usbpd_trim_3 = tmp;

    /*
     * Clearing of TX_TRIM field is enough for Rp = 1.5A or Default termination
     * at initialization time. Later TX_TRIM could be updated for Rp = 3A
     * termination.
     */
    pd->s8usbpd_trim_0 &= (~(PDSS_S8USBPD_TRIM_0_TX_TRIM_MASK));

#ifdef CCG3
    if (pd_vbus_mon_internal)
    {
        /*
         * The UVP comparator shall be reserved for VBUS internal divider. The
         * divider setting shall be set to match 1/11 (same as CCG4). Since the
         * nearest match value for 1/11 is 8.25, the UVP_IN shall be set to
         * corresponding value of 22. Load the same value into OV until OVP is
         * being added. This is to ensure that voltage does not cross beyond
         * expected values.
         */
        tmp = pd->uvov_ctrl & ~(PDSS_UVOV_CTRL_UV_IN_MASK | PDSS_UVOV_CTRL_OV_IN_MASK |
                PDSS_UVOV_CTRL_UVOV_ADFT_CTRL_MASK | PDSS_UVOV_CTRL_PD_UVOV);
        tmp |= ((22 << PDSS_UVOV_CTRL_UV_IN_POS) | (22 << PDSS_UVOV_CTRL_OV_IN_POS) |
                PDSS_UVOV_CTRL_UVOV_ISO_N | PDSS_UVOV_CTRL_UVOV_ADFT_EN |
                (1 << PDSS_UVOV_CTRL_UVOV_ADFT_CTRL_POS));
        pd->uvov_ctrl = tmp;
    }

    /*
     * CDT 264973: The clock divider to drive the charge pump should be the lowest
     * (fastest clock) so that the gate drivers are turned on quickly. This setting
     * corresponds to the timing specs for the device.
     */
    pd->ngdo_ctrl_0 &= ~(PDSS_NGDO_CTRL_0_NGDO_VBUS_C_CLKSEL_LV_MASK | PDSS_NGDO_CTRL_0_NGDO_VBUS_P_CLKSEL_LV_MASK);

#endif /* CCG3 */

#if ((defined(CCG4PD3)) && (VBUS_FET_INTERNAL_CTRL))
    /*
     * Configure the FET internal drives. The drive mode and initial values
     * are expected to be loaded via Creator schematic and only the MUX control
     * is being updated here.
     */
    if (port == TYPEC_PORT_0_IDX)
    {
        /* P1_PCTRL0 = 1.6, HSIOM_MODE_P0_DDFT0. */
        hsiom_set_config(VBUS_P_CTRL0_GPIO_P1, HSIOM_MODE_P0_DDFT0);
        /* P1_CCTRL0 = P1.7, HSIOM_MODE_P0_SWAPT_OUT0. */
        hsiom_set_config(VBUS_C_CTRL0_GPIO_P1, HSIOM_MODE_P0_SWAPT_OUT0);
        if (pd_dual_fet)
        {
            /* P1_PCTRL1 = 2.1, HSIOM_MODE_P0_DDFT1. */
            hsiom_set_config(VBUS_P_CTRL1_GPIO_P1, HSIOM_MODE_P0_DDFT1);
            /* P1_CCTRL1 = P3.0, HSIOM_MODE_P0_SWAPT_OUT1. */
            hsiom_set_config(VBUS_C_CTRL1_GPIO_P1, HSIOM_MODE_P0_SWAPT_OUT1);
        }
    }
#if (CCG_PD_DUALPORT_ENABLE)
    if (port == TYPEC_PORT_1_IDX)
    {
        /* P2_PCTRL0 = 4.2, HSIOM_MODE_P1_DDFT0. */
        hsiom_set_config(VBUS_P_CTRL0_GPIO_P2, HSIOM_MODE_P1_DDFT0);
        /* P2_CCTRL0 = P4.1, HSIOM_MODE_P1_SWAPT_OUT0. */
        hsiom_set_config(VBUS_C_CTRL0_GPIO_P2, HSIOM_MODE_P1_SWAPT_OUT0);
        if (pd_dual_fet)
        {
            /* P2_PCTRL1 = 3.7, HSIOM_MODE_P1_DDFT1. */
            hsiom_set_config(VBUS_P_CTRL1_GPIO_P2, HSIOM_MODE_P1_DDFT1);
            /* P2_CCTRL1 = P3.6, HSIOM_MODE_P1_SWAPT_OUT1. */
            hsiom_set_config(VBUS_C_CTRL1_GPIO_P2, HSIOM_MODE_P1_SWAPT_OUT1);
        }
    }
#endif /* (CCG_PD_DUALPORT_ENABLE) */

#endif /* ((defined(CCG4PD3)) && (VBUS_FET_INTERNAL_CTRL)) */

#if ((defined(CCG3)) || (defined(CCG4PD3)))
    /*
     * Bypass the synchronizer for fault detection in CLK_LF domain, so that
     * spacing takes effect right after fault detection.
     */
    pd->ngdo_ctrl_p |= PDSS_NGDO_CTRL_P_BYPASS_2DFF;
    pd->ngdo_ctrl_c |= PDSS_NGDO_CTRL_C_BYPASS_2DFF;
#endif /* ((defined(CCG3)) || (defined(CCG4PD3))) */

    /* Connect comparator 1 to AmuxA. */
    gl_pdss_status[port].adc_vddd_mv[PD_ADC_ID_0] = PD_ADC_DEFAULT_VDDD_VOLT_MV;
    gl_pdss_status[port].adc_vddd_mv[PD_ADC_ID_1] = PD_ADC_DEFAULT_VDDD_VOLT_MV;

    /* Initialize ADCs. */
    pd_adc_init(port, PD_ADC_ID_0);
    pd_adc_init(port, PD_ADC_ID_1);

    return CCG_STAT_SUCCESS;
}

void pd_hal_cleanup(uint8_t port)
{
    (void)port;
    /* Do nothing. */
}

void pd_phy_vbus_detach_cbk(uint8_t port, bool comp_out)
{
    /* Do nothing. */
}

#if (NO_OF_TYPEC_PORTS == 1)
static void pd_typec_wake(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Enable PUMP */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);
    CyDelayUs(50);
}

static void pd_typec_sleep(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Disable PUMP */
    pd->pump_ctrl |= (PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);
}
#endif /* (NO_OF_TYPEC_PORTS == 1) */

void pd_phy_detect_cc_rise (uint8_t port, bool rp_connected)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint8_t threshold_row = RD_ROW_NO;

    if (rp_connected)
        threshold_row = dpm_stat->src_cur_level_live;

    /* Connect the Up comparator to CC1. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CMP_UP_CC1V2;

    /* Connect the Dn comparator to CC2. */
    pd->cc_ctrl_0 |=  PDSS_CC_CTRL_0_CMP_DN_CC1V2;

    /*
     * Set the Up comparator on CC1 and the Dn comparator on CC2 for rising threshold.
     */
    pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK);
    pd->cc_ctrl_0 |= ((thresholds[threshold_row][0]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
        ((thresholds[threshold_row][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);

    pd->intr_1_cfg &= ~(PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK | PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK);
    pd->intr_1_cfg |= ((PD_ADC_INT_RISING) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS) |
        ((PD_ADC_INT_RISING) << PDSS_INTR_1_CFG_VCMP_DN_CFG_POS);

    CyDelayUs (20);
    pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED);
    pd->intr1_mask |= (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED);

    /* If comparators have already triggered, then set the interrupts and return. */
    if (pd->status & PDSS_STATUS_VCMP_UP_STATUS)
    {
        pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
    }
    else
    {
        if (pd->status & PDSS_STATUS_VCMP_DN_STATUS)
        {
            pd->intr1_set |= PDSS_INTR1_VCMP_DN_CHANGED;
        }
    }
}

bool pd_phy_deepsleep(uint8_t port)
{
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t level;
    uint32_t status;

    if (!(dpm_stat->dpm_enabled))
    {
        return true;
    }

    if (dpm_stat->connect == true)
    {
        /* Set LA comparator for wakeup. */
        pd->intr1 = PDSS_INTR1_VCMP_LA_CHANGED;
        pd->intr1_mask |= PDSS_INTR1_VCMP_LA_CHANGED;

        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            if (dpm_stat->attached_dev == DEV_AUD_ACC)
            {
                /* Set UP and DN comparators on the two CC lines to signal detach when voltage rises above VRa. */
                pd_phy_detect_cc_rise (port, true);
                return true;
            }
            else
            {
                /* Set the Up comparator on the active CC line to signal a detach based on the Rp level. */
                pd->intr_1_cfg &= ~ PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK;
                pd->intr_1_cfg |= (PD_ADC_INT_RISING) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS;
                pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED;
                pd->intr1_mask |= PDSS_INTR1_VCMP_UP_CHANGED;

                /* If the comparator has already triggered, set the interrupt and return. */
                if (pd->status & PDSS_STATUS_VCMP_UP_STATUS)
                {
                    pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                }
            }
        }
        else
        {
            /* Set for Rp change.  */
            status = pd->status & (PDSS_STATUS_VCMP_UP_STATUS | PDSS_STATUS_VCMP_DN_STATUS);

            /* Set the Up comparator on Active CC line to signal a detach based on the Rp level. */
            pd->intr_1_cfg &= ~ (PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK | PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK);
            pd->intr_1_cfg |= ((PD_ADC_INT_BOTH) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS) |
                ((PD_ADC_INT_BOTH) << PDSS_INTR_1_CFG_VCMP_DN_CFG_POS);

            pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED;
            pd->intr1_mask |= PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED;

            /* If the comparator has already triggered, set the interrupt and return. */
            if(dpm_stat->cc_old_status.state != pd_typec_get_cc_status(port).state)
            {
                /* Fire any one interrupt to wakeup */
                pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                return true;
            }

#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
            if(dpm_stat->fr_rx_en_live == false)
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */
            {
                /* Set the VBus detach comparator as per current detach threshold. */
                level = pd_get_vbus_adc_level(port, pd_vbus_detach_adc_id,
                        dpm_get_sink_detach_voltage(port), dpm_get_sink_detach_margin(port));

                /*
                 * The following call will also check if the comparator has
                 * triggered and set the interrupt.
                 * */
                pd_adc_comparator_ctrl(port, pd_vbus_detach_adc_id, pd_vbus_detach_adc_input,
                        level, PD_ADC_INT_RISING, pd_phy_vbus_detach_cbk);
            }
        }
    }
    else
    {
        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            /*
             * If Ra is present on only 1 CC line, then set the Up comparator
             * on the Ra line for a rising edge as per the Rp level. Check if
             * the comparator has already triggered, then set the interrupt and
             * return.
             *
             * Set the Dn comparator on the other line for a falling edge as
             * per the Rp level. Check if the comparator has already triggered,
             * then set the interrupt and return.
             *
             * Otherwise,
             *
             * Set the Up comparator on CC1 for a falling edge as per the Rp
             * level. Set the Dn comparator on CC2 for a falling edge as per
             * the Rp level. If the comparators have already triggered, then
             * set the respective interrupt and return.
             */

            /* Connect  UP comparator to CC1 */
            pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CMP_UP_CC1V2;

            /* Connect Down comparator to CC2 */
            pd->cc_ctrl_0 |=  PDSS_CC_CTRL_0_CMP_DN_CC1V2;

            /* Set threshold to Ra level to check if Ra is present on single CC line. */
            pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK);
            pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
                ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
            CyDelayUs(10);

            status = pd->status;

#if (NO_OF_TYPEC_PORTS == 1)
            uint8_t cc1_edge = PD_ADC_INT_DISABLED;
            uint8_t cc2_edge = PD_ADC_INT_DISABLED;

            /* Apply resistor based Rp and remove current source Rp */
            pd->cc_ctrl_1 |= PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
            pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_RP_CC1_EN | PDSS_CC_CTRL_0_RP_CC2_EN);
            pd_typec_sleep(port);
            pd->intr_1_cfg &= ~(PDSS_INTR_1_CFG_CC1_CFG_MASK | PDSS_INTR_1_CFG_CC2_CFG_MASK |
                    PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK | PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK);

            pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK);

            if ((status & (PDSS_STATUS_VCMP_UP_STATUS | PDSS_STATUS_VCMP_DN_STATUS)) == PDSS_STATUS_VCMP_DN_STATUS)
            {
                cc1_edge = PD_ADC_INT_RISING;
                cc2_edge = PD_ADC_INT_FALLING;

                /* Set threshold at Ra level on CC1 and Rd level on CC2. */
                pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
                    (PD_CMP_VSEL_1_77_V << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
            }
            else if ((status & (PDSS_STATUS_VCMP_UP_STATUS | PDSS_STATUS_VCMP_DN_STATUS)) == PDSS_STATUS_VCMP_UP_STATUS)
            {
                cc1_edge = PD_ADC_INT_FALLING;
                cc2_edge = PD_ADC_INT_RISING;

                /* Set threshold at Rd level on CC1 and Ra level on CC2. */
                pd->cc_ctrl_0 |= (PD_CMP_VSEL_1_77_V << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
                    ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
            }
            else
            {
                cc1_edge = PD_ADC_INT_FALLING;
                cc2_edge = PD_ADC_INT_FALLING;

                /* Set threshold at Rd level on both CC1 and CC2. */
                pd->cc_ctrl_0 |= (PD_CMP_VSEL_1_77_V << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
                    (PD_CMP_VSEL_1_77_V << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
            }
            CyDelayUs(10);

            pd->intr_1_cfg |= ((cc1_edge) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS) |
                ((cc2_edge) << PDSS_INTR_1_CFG_VCMP_DN_CFG_POS);
            pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED |
                    PDSS_INTR1_VCMP_DN_CHANGED |
                    PDSS_INTR1_VCMP_LA_CHANGED |
                    PDSS_INTR1_CC1_CHANGED |
                    PDSS_INTR1_CC2_CHANGED);
            pd->intr1_mask |= (PDSS_INTR1_CC1_CHANGED | PDSS_INTR1_CC2_CHANGED |
                    PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED);

            /* If the comparators have  already triggered, then set the interrupts and return. */
            if(((cc1_edge == PD_ADC_INT_RISING) && ((pd->status & PDSS_STATUS_VCMP_UP_STATUS) != 0)) ||
                    ((cc1_edge == PD_ADC_INT_FALLING) && ((pd->status & PDSS_STATUS_VCMP_UP_STATUS) == 0)) ||
                    ((cc2_edge == PD_ADC_INT_RISING) && ((pd->status & PDSS_STATUS_VCMP_DN_STATUS) != 0)) ||
                    ((cc2_edge == PD_ADC_INT_FALLING) && ((pd->status & PDSS_STATUS_VCMP_DN_STATUS) == 0)))
            {
                /* Fire anyone to wakeup*/
                pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                return true;
            }
#else /* (NO_OF_TYPEC_PORTS != 1) */
            if ((status & PDSS_STATUS_VCMP_UP_STATUS) == 0)
            {
                if (status & PDSS_STATUS_VCMP_DN_STATUS)
                {
                    pd->cc_ctrl_0 &= ~ PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK;
                    pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][1]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
                    pd->intr_1_cfg &= ~ (PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK | PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK);
                    pd->intr_1_cfg |= ((PD_ADC_INT_RISING) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS) |
                        ((PD_ADC_INT_FALLING) << PDSS_INTR_1_CFG_VCMP_DN_CFG_POS);

                    CyDelayUs(10);
                    pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED);
                    pd->intr1_mask |= (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED);

                    /* If the comparators have already triggered, then set the interrupts and return. */
                    if (pd->status & PDSS_STATUS_VCMP_UP_STATUS)
                    {
                        pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                        return true;
                    }

                    if ((pd->status & PDSS_STATUS_VCMP_DN_STATUS) == 0)
                    {
                        pd->intr1_set |= PDSS_INTR1_VCMP_DN_CHANGED;
                    }

                    return true;
                }
            }

            if ((status & PDSS_STATUS_VCMP_DN_STATUS) == 0)
            {
                if (status & PDSS_STATUS_VCMP_UP_STATUS)
                {
                    pd->cc_ctrl_0 &= ~ PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK;
                    pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][1]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS);
                    pd->intr_1_cfg &= ~ (PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK | PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK);
                    pd->intr_1_cfg |= ((PD_ADC_INT_FALLING) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS) |
                        ((PD_ADC_INT_RISING) << PDSS_INTR_1_CFG_VCMP_DN_CFG_POS);

                    CyDelayUs(10);
                    pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED);
                    pd->intr1_mask |= (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED);

                    /* If the comparators have already triggered, then set the interrupts and return. */
                    if ((pd->status & PDSS_STATUS_VCMP_UP_STATUS) == 0)
                    {
                        pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                        return true;
                    }
                    if (pd->status & PDSS_STATUS_VCMP_DN_STATUS)
                    {
                        pd->intr1_set |= PDSS_INTR1_VCMP_DN_CHANGED;
                    }

                    return true;
                }
            }

            /*
             * Set the Up comparator on CC1 and the Dn comparator on CC2 for
             * falling threshold equivalent to VRd.
             */
            pd->cc_ctrl_0 &= ~ (PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK);
            pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][1]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
                ((thresholds[dpm_stat->src_cur_level_live][1]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);

            pd->intr_1_cfg &= ~ (PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK | PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK);
            pd->intr_1_cfg |= ((PD_ADC_INT_FALLING) << PDSS_INTR_1_CFG_VCMP_UP_CFG_POS) |
                ((PD_ADC_INT_FALLING) << PDSS_INTR_1_CFG_VCMP_DN_CFG_POS);

            CyDelayUs(10);
            pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED);
            pd->intr1_mask |= (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED);

            /* If the comparators have  already triggered, then set the interrupts and return. */
            if ((pd->status & PDSS_STATUS_VCMP_UP_STATUS) == 0)
            {
                pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                return true;
            }

            if ((pd->status & PDSS_STATUS_VCMP_DN_STATUS) == 0)
            {
                pd->intr1_set |= PDSS_INTR1_VCMP_DN_CHANGED;
            }
#endif /* (NO_OF_TYPEC_PORTS == 1) */
        }
        else
        {
            pd_phy_detect_cc_rise (port, false);
        }
    }

    return true;
}

bool pd_phy_wakeup(void)
{
    uint32_t i;
    PPDSS_REGS_T pd;
    dpm_status_t* dpm_stat;

    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        dpm_stat = dpm_get_status(i);
        pd = gl_pdss[i];

        if (!(dpm_stat->dpm_enabled))
        {
            continue;
        }

        /* Disable the deepsleep interrupts. */
        pd->intr1_mask &= ~(PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED);
        pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED;

        if ((dpm_stat->connect == true) && (dpm_stat->cur_port_role == PRT_ROLE_SINK))
        {
#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
            if(dpm_stat->fr_rx_en_live == false)
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */
            {
                /* Disable the detach detection comparator. */
                pd_adc_comparator_ctrl(i, pd_vbus_detach_adc_id, 0, 0, 0, NULL);
            }
        }
#if (NO_OF_TYPEC_PORTS == 1)
        else if(dpm_stat->connect == false)
        {
            if(dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
            {
                pd_typec_wake(i);

                /* Remove deepsleep Rp and put normal Rp */
                pd->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RP_CC1_EN | PDSS_CC_CTRL_0_RP_CC2_EN);
                pd->cc_ctrl_1 &= ~PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
            }
        }
#endif /* (NO_OF_TYPEC_PORTS == 1) */
    }

    return true;
}

ccg_status_t pd_phy_init(uint8_t port, pd_phy_cbk_t cbk)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    pdss_stat->pd_phy_cbk = cbk;

#if CCG_PD_REV3_ENABLE
    /*Configure RX_SOP_GOOG_CRC_EN_CTRL*/
    pd->rx_sop_good_crc_en_ctrl = RX_SOP_GOOD_CRC_EN_CTRL_REV3_CFG;
#else
    /*Configure RX_SOP_GOOG_CRC_EN_CTRL*/
    pd->rx_sop_good_crc_en_ctrl = RX_SOP_GOOD_CRC_EN_CTRL_CFG;
#endif /* CCG_PD_REV3_ENABLE */

    /*Enable Auto Good CRC for all messages except goodcrc*/
    pd->rx_default_sop_goodcrc_ctrl_0 = AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG;
    pd->rx_default_sop_goodcrc_ctrl_1 = AUTO_DATA_MESSAGE_GOODCRC_MASK_CFG;

    pd->rx_dbl_prime_sop_goodcrc_ctrl_0 = AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG;
    pd->rx_dbl_prime_sop_goodcrc_ctrl_1 = AUTO_DATA_MESSAGE_GOODCRC_MASK_CFG;

    pd->rx_prime_sop_goodcrc_ctrl_0 = AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG;
    pd->rx_prime_sop_goodcrc_ctrl_1 = AUTO_DATA_MESSAGE_GOODCRC_MASK_CFG;

#if CCG_PD_REV3_ENABLE
    /* Configure Extended Header Info register */
    pd->header_info = HEADER_INFO_CFG;
    pd->rx_default_sop_goodcrc_ctrl_2 = AUTO_EXTD_MESSAGE_GOODCRC_MASK_CFG;
    pd->rx_dbl_prime_sop_goodcrc_ctrl_2 = AUTO_EXTD_MESSAGE_GOODCRC_MASK_CFG;
    pd->rx_prime_sop_goodcrc_ctrl_2 = AUTO_EXTD_MESSAGE_GOODCRC_MASK_CFG;
#endif /* CCG_PD_REV3_ENABLE */

    /*Configure RX_CC reg*/
    pd->rx_cc = RX_CC_CFG;

    /*Configure RX_ORDER_SET_CTRL*/
    pd->rx_order_set_ctrl = RX_ORDER_SET_CTRL_CFG;

    /*Configure CRC_COUNTER reg*/
    pd->crc_counter = CRC_COUNTER_CFG;

    /*Configure INTER_PACKET_COUNTER reg*/
    pd->inter_packet_counter = INTER_PACKET_COUNTER_CFG;

    /*Disable all PD interrupts*/
    pd->intr0_mask &= ~PD_INTR_MASK;

    /*Configure DEBUG_CC_2 reg to disable cc monitoring during idle gap before
     * transmitting goodcrc and set expected goodrc message header mask*/
    pd->debug_cc_2 &= ~PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_MASK;
    pd->debug_cc_2 |= (EXPECTED_GOOD_CRC_HDR_MASK << PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS) |
        PDSS_DEBUG_CC_2_DIS_CC_MON_AUTO_CRC;

    /*Configure SOP_PRIME and SOP_DPRIME Auto Goodrc Header*/
    pd->tx_goodcrc_msg_order_set = TX_SOP_PRIME_DPRIME_GD_CRC_HDR_DFLT;

    return CCG_STAT_SUCCESS;
}

void pd_phy_refresh_roles(uint8_t port)
{
#if CCG_PD_REV3_ENABLE

    PPDSS_REGS_T pd = gl_pdss[port];
    dpm_status_t* dpm_stat = dpm_get_status(port);
    uint32_t temp;

    temp = pd->rx_order_set_ctrl;
    temp &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;
    temp |= (EN_DEFAULT_SOP_DET_VAL | EN_RX_HARD_RESET_DET_VAL);
    if(dpm_stat->contract_exist == false)
    {
        if(dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            /*Enable SOP_PRIME only*/
            temp |= EN_PRIME_SOP_DET_VAL;
        }
    }
    else
    {
        if(dpm_stat->cur_port_type == PRT_TYPE_DFP)
        {
            /*Enable SOP_PRIME/SOP_DPRIME*/
            temp |= (EN_PRIME_SOP_DET_VAL | EN_DBL_SOP_DET_VAL);
        }
    }
    if(dpm_stat->spec_rev_sop_live >= PD_REV3)
    {
        /* Update goodcrc mask */
        pd->debug_cc_2 |= (EXPECTED_GOOD_CRC_HDR_DIFF_MASK_REV3 << PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS);

        /* Enable Extended RX */
        pd->header_info |= PDSS_HEADER_INFO_EN_RX_EXTENDED_DATA;

        /* Only a VCONN source can talk to cable */
        if((dpm_stat->vconn_logical == false) || (dpm_stat->cbl_dsc == false))
        {
            temp &= ~(EN_PRIME_SOP_DET_VAL | EN_DBL_SOP_DET_VAL);
        }
        else
        {
            /* While in implicit contract, only source can talk to cable. */
            if ((dpm_stat->contract_exist != false) || (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
            {
                /* Enable SOP_PRIME/SOP_DPRIME */
                temp |= (EN_PRIME_SOP_DET_VAL | EN_DBL_SOP_DET_VAL);
            }
        }
    }
    else
    {
        /* Update goodcrc mask */
        pd->debug_cc_2 &= ~(EXPECTED_GOOD_CRC_HDR_DIFF_MASK_REV3 << PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS);

        /* Disable Extended RX/TX*/
        pd->header_info &= ~(PDSS_HEADER_INFO_EN_RX_EXTENDED_DATA | PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA);
    }

    pd->rx_order_set_ctrl = temp;

    temp = pd->tx_ctrl;
    temp &= ~PDSS_TX_CTRL_GOODCRC_MSG_BITS_MASK;
    temp |= ( CTRL_MSG_GOOD_CRC | PD_DR_PR_ROLE(dpm_stat->cur_port_type, dpm_stat->cur_port_role));
    temp |= (PD_REV2 << PD_REV_POS);
    pd->tx_ctrl = temp;

    /* Configure SOP_PRIME and SOP_DPRIME Auto Goodrc Header */
    temp = CTRL_MSG_GOOD_CRC;
    temp |= (PD_REV2 << PD_REV_POS);
    pd->tx_goodcrc_msg_order_set = (temp << 16)| temp;

#else

    PPDSS_REGS_T pd = gl_pdss[port];
    dpm_status_t* dpm_stat = dpm_get_status(port);
    uint32_t temp;

    temp = pd->rx_order_set_ctrl;
    temp &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;
    temp |= (EN_DEFAULT_SOP_DET_VAL | EN_RX_HARD_RESET_DET_VAL);
    if(dpm_stat->contract_exist == false)
    {
        if(dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            /*Enable SOP_PRIME only*/
            temp |= EN_PRIME_SOP_DET_VAL;
        }
    }
    else
    {
        if(dpm_stat->cur_port_type == PRT_TYPE_DFP)
        {
            /*Enable SOP_PRIME/SOP_DPRIME*/
            temp |= (EN_PRIME_SOP_DET_VAL | EN_DBL_SOP_DET_VAL);
        }
    }
    pd->rx_order_set_ctrl = temp;

    temp = pd->tx_ctrl;
    temp &= ~PDSS_TX_CTRL_GOODCRC_MSG_BITS_MASK;
    temp |= (TX_SOP_GD_CRC_HDR_DFLT | PD_DR_PR_ROLE(dpm_stat->cur_port_type, dpm_stat->cur_port_role));
    pd->tx_ctrl = temp;

#endif /* CCG_PD_REV3_ENABLE */
}

void pd_phy_en_unchunked_tx(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->header_info |= PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
}

void pd_phy_dis_unchunked_tx(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->header_info &= ~PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
}

bool pd_phy_load_data_in_mem(uint8_t port, bool start)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];
    uint8_t i;
    uint8_t tx_ptr = pd->sram_ptr & PDSS_SRAM_PTR_TX_FUNC_RD_PTR_MASK;
    uint8_t start_idx = 0;
    uint8_t mem_size = 0;

    if(pdss_stat->tx_dobj_count == 0)
    {
        return false;
    }

    if(start == true)
    {
        mem_size = PDSS_MAX_TX_MEM_SIZE;
    }
    else
    {
        mem_size = PDSS_MAX_TX_MEM_HALF_SIZE;
        if(tx_ptr < PDSS_MAX_TX_MEM_HALF_SIZE)
        {
            start_idx = PDSS_MAX_TX_MEM_HALF_SIZE;
        }
    }

    /* Copy the data into the Tx memory. */
    for (i = start_idx; i < (start_idx+ mem_size); i++)
    {
        pd->tx_mem_data[i] = pdss_stat->tx_dat_ptr[pdss_stat->tx_obj_sent];
        pdss_stat->tx_obj_sent++;
        if(pdss_stat->tx_obj_sent >= pdss_stat->tx_dobj_count)
        {
            return false;
        }
    }

    return true;
}

#if CCG_PD_REV3_ENABLE
void pd_phy_read_data_from_mem(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];
    uint8_t i;
    uint8_t start_idx = pdss_stat->rx_read_location;
    uint8_t mem_size = PDSS_MAX_RX_MEM_HALF_SIZE;

    /* Copy the data from rx memory. */
    for (i = start_idx; i < (start_idx + mem_size); i++)
    {
        if(pdss_stat->rx_unchunk_count >= pdss_stat->rx_unchunk_len)
        {
            return;
        }
        pdss_stat->rx_pkt.dat[pdss_stat->rx_unchunk_count] = (pd_do_t)pd->rx_mem_data[i];
        pdss_stat->rx_unchunk_count++;
    }

    /* Flip to the other half of the SRAM for the next read. */
    pdss_stat->rx_read_location = (pdss_stat->rx_read_location == 0) ?
        PDSS_MAX_RX_MEM_HALF_SIZE: 0;
}
#endif /* CCG_PD_REV3_ENABLE */

bool pd_phy_load_msg(uint8_t port, sop_t sop, uint8_t retries,
        uint8_t dobj_count, uint32_t header, bool unchunked, uint32_t* buf)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];
    uint16_t exp_hdr;

    pdss_stat->tx_dat_ptr = buf;
    pdss_stat->tx_dobj_count = dobj_count;
    pdss_stat->tx_unchunked = unchunked;


    /* Configure SOP ordered set. */
    pd->tx_sop_order_set = os_table[sop];

    /*
     * Configure the expected sop type and expected GoodCRC. Expected GoodCRC
     * mask was already set by pd_phy_init(). SOP type in hardware is sop + 1.
     */
    exp_hdr = header & (~EXPECTED_GOOD_CRC_CLEAR_MASK);
    exp_hdr |= CTRL_MSG_GOOD_CRC;
    pd->rx_expect_goodcrc_msg = exp_hdr | ((sop + 1) <<
            PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_SOP_POS);

    /* Load the header in the Tx header register. */
    pd->tx_header = header;

    /* Save the number of requested retries. */
    pdss_stat->retry_cnt = (int8_t)retries;

    return true;
}

bool pd_phy_send_msg(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
    uint32_t rval;

    pd->intr0_mask &= ~PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

    /* Clear Tx interrupts. */
    pd->intr0 = (TX_INTERRUPTS | PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE |
            PDSS_INTR0_TX_RETRY_ENABLE_CLRD | PDSS_INTR0_CC_NO_VALID_DATA_DETECTED |
            PDSS_INTR0_TX_SRAM_HALF_END);

    if (pdss_stat->retry_cnt < 0)
    {
        /* Create this interrupt to stop transmission. */
        pd->intr0_set |= PDSS_INTR0_CRC_RX_TIMER_EXP;
        return true;
    }

    if (pd->status & (PDSS_STATUS_RX_BUSY | PDSS_STATUS_SENDING_GOODCRC_MSG))
    {
        pdss_stat->retry_cnt--;

        pd->intr0_mask |= PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

        /*
         * Notify the protocol layer so that it can start a timer so as to
         * avoid an infinite wait on the channel going idle.
         */
        pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_COLLISION);

        return true;
    }

    pdss_stat->tx_obj_sent = 0;
    if(pd_phy_load_data_in_mem(port, true) == true)
    {
        /* Enable TX SRAM HALF END interrupt */
        pd->intr0_mask |= PDSS_INTR0_TX_SRAM_HALF_END;
    }
    /* Enable Tx interrupts. */
    pd->intr0_mask |= TX_INTERRUPTS;

    /* Checks if unchunked TX need to be enabled */
    if(pdss_stat->tx_unchunked == true)
    {
        pd->header_info |= PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
    }
    else
    {
        pd->header_info &= ~PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
    }

    rval = pd->tx_ctrl;
    if (pdss_stat->retry_cnt != 0)
    {
        rval |= PDSS_TX_CTRL_TX_RETRY_ENABLE;
    }
    else
    {
        /* No retries. */
        rval &= ~PDSS_TX_CTRL_TX_RETRY_ENABLE;
    }
    rval |= PDSS_TX_CTRL_TX_GO;

    pdss_stat->tx_done = false;
    pdss_stat->retry_cnt--;

    /* Begin transmission. */
    pd->tx_ctrl = rval;

    return true;
}

ccg_status_t pd_prot_stop(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd_prot_rx_dis(port, false);
    pd_phy_reset_rx_tx_sm (port);

    pd->intr0_mask &= ~(TX_INTERRUPTS | RST_TX_INTERRUPTS |
            PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
            PDSS_INTR0_CC_NO_VALID_DATA_DETECTED|
            PDSS_INTR0_TX_SRAM_HALF_END);
    pd->intr0 = (TX_INTERRUPTS | RST_TX_INTERRUPTS |
            PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
            PDSS_INTR0_CC_NO_VALID_DATA_DETECTED|
            PDSS_INTR0_TX_SRAM_HALF_END);

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_prot_rx_en(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    pdss_stat->rx_unchunked = false;

    /* Clear and enable RX interrupts. */
    pd->intr0 = (RX_INTERRUPTS |RCV_INTR_MASK);

#if CCG_PD_REV3_ENABLE
    dpm_status_t* dpm_stat = dpm_get_status(port);
    pd->intr2 = PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET |PDSS_INTR2_RX_SRAM_OVER_FLOW;
    if(dpm_stat->spec_rev_sop_live >=PD_REV3)
    {
        pd->intr2_mask |= PDSS_INTR2_EXTENDED_MSG_DET;
    }
#endif /* CCG_PD_REV3_ENABLE */

    pd->intr0_mask |= RX_INTERRUPTS;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_prot_rx_dis(uint8_t port, uint8_t hard_reset_en)
{
    uint32_t temp;
    PPDSS_REGS_T pd = gl_pdss[port];

#if CCG_PD_REV3_ENABLE
    pd->intr2_mask &= ~(PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET);
    pd->intr2 = (PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET |
            PDSS_INTR2_RX_SRAM_OVER_FLOW);
#endif /* CCG_PD_REV3_ENABLE */

    if (hard_reset_en == false)
    {
        /* Disable Rx.*/
        pd->rx_order_set_ctrl &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;

        /* Disable and clear all Rx interrupts.*/
        pd->intr0_mask &= ~RX_INTERRUPTS;
    }

    if (hard_reset_en == true)
    {
        /* Enable only Hard Reset reception. */
        temp = pd->rx_order_set_ctrl;
        temp &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;
        temp |= EN_RX_HARD_RESET_DET_VAL;
        pd->rx_order_set_ctrl = temp;

        /* Enable only the Hard Reset received interrrupt. */
        temp = pd->intr0_mask;
        temp &= ~RX_INTERRUPTS;
        temp |=  PDSS_INTR0_RCV_RST;
        pd->intr0_mask = temp;
    }

    pd->intr0 = RX_INTERRUPTS;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_send_bist_cm2(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Enable Tx regulator. */
    pd->tx_ctrl |= PDSS_TX_CTRL_TX_REG_EN;

    /* Delay to let the Tx regulator turn on. */
    CyDelayUs(50);

    /* Start BIST CM2. */
    pd->tx_ctrl |= PDSS_TX_CTRL_EN_TX_BIST_CM2;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_abort_bist_cm2(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Stop BIST CM2. */
    pd->tx_ctrl &= ~PDSS_TX_CTRL_EN_TX_BIST_CM2;

    /* Disable Tx regulator. */
    pd->tx_ctrl &= ~PDSS_TX_CTRL_TX_REG_EN;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_abort_tx_msg(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    pdss_stat->tx_done = false;

    pd->intr0_mask &= ~(TX_INTERRUPTS | PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
            PDSS_INTR0_CC_NO_VALID_DATA_DETECTED | PDSS_INTR0_TX_SRAM_HALF_END);
    pd->intr0 = (TX_INTERRUPTS | PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
            PDSS_INTR0_CC_NO_VALID_DATA_DETECTED |PDSS_INTR0_TX_SRAM_HALF_END);

    return CCG_STAT_SUCCESS;
}

void pd_phy_reset_rx_tx_sm(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Stop any ongoing transmission */
    pd->debug_ctrl |= (PDSS_DEBUG_CTRL_RESET_TX | PDSS_DEBUG_CTRL_RESET_RX);
    CyDelayUs(5);
    pd->debug_ctrl &= ~(PDSS_DEBUG_CTRL_RESET_TX | PDSS_DEBUG_CTRL_RESET_RX);
}

ccg_status_t pd_phy_send_reset(uint8_t port, sop_t sop)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t loops = 10;

    /* Send a Hard Reset or Cable Reset. */
    pd->intr0 = RST_TX_INTERRUPTS;
    pd->intr0_mask |= RST_TX_INTERRUPTS;

    pd->tx_hard_cable_order_set = os_table[sop];

    if (sop == HARD_RESET)
    {
        pd_phy_reset_rx_tx_sm (port);

        /* Wait while there is valid data on the CC line. */
        while ((loops > 0) &&
                ((pd->status & (
                                PDSS_STATUS_CC_DATA_VALID |
                                PDSS_STATUS_RX_BUSY |
                                PDSS_STATUS_TX_BUSY |
                                PDSS_STATUS_SENDING_GOODCRC_MSG
                               )
                 ) != 0)
              )
        {
            loops--;
            CyDelayUs (10);
        }

        if (
                (pd->status & (
                               PDSS_STATUS_CC_DATA_VALID |
                               PDSS_STATUS_RX_BUSY |
                               PDSS_STATUS_TX_BUSY |
                               PDSS_STATUS_SENDING_GOODCRC_MSG
                              )
                ) != 0)
        {
            /* Return busy so that the reset gets attempted again at a later time. */
            return CCG_STAT_BUSY;
        }
    }

    pd->tx_ctrl |= PDSS_TX_CTRL_TX_SEND_RST;
    return CCG_STAT_SUCCESS;
}

pd_packet_extd_t *pd_phy_get_rx_packet(uint8_t port)
{
    return &gl_pdss_status[port].rx_pkt;
}

bool pd_phy_is_busy(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (((pd->status & (PDSS_STATUS_CC_DATA_VALID |
                        PDSS_STATUS_RX_BUSY |
                        PDSS_STATUS_TX_BUSY |
                        PDSS_STATUS_SENDING_GOODCRC_MSG
                       )) != 0 ) ||
            (pd->intr0 & PDSS_INTR0_RCV_RST))
    {
        return true;
    }

    return false;
}

void pdss_intr0_handler(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
#if ((CCG_HPD_RX_ENABLE) || (!CCG_PD_REV3_ENABLE))
    uint32_t  i;
#endif /* ((CCG_HPD_RX_ENABLE) || (!CCG_PD_REV3_ENABLE)) */
    uint32_t rval;
#if CCG_PD_REV3_ENABLE
    dpm_status_t* dpm_stat = dpm_get_status(port);
#endif /* CCG_PD_REV3_ENABLE */

    if (pd->intr0_masked != 0)
    {
        /*
         * Receive interrupt handling.
         */
        if (pd->intr0_masked & PDSS_INTR0_RCV_RST)
        {
            pdss_stat->tx_done = false;
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_RST);
            pd->intr0 = (PDSS_INTR0_RCV_RST | PDSS_INTR0_EOP_ERROR);
        }

        if (pd->intr0_masked & PDSS_INTR0_TX_PACKET_DONE)
        {
            pd->intr0 = PDSS_INTR0_TX_PACKET_DONE;
            pdss_stat->tx_done = true;
        }

#if CCG_PD_REV3_ENABLE
        if (pd->intr2_masked & PDSS_INTR2_EXTENDED_MSG_DET)
        {
            if(pd->intr2 & PDSS_INTR2_CHUNK_DET)
            {
                pdss_stat->rx_unchunked = false;
            }
            else
            {
                pdss_stat->rx_unchunked = true;
                /* Store total byte count and initialize byte received */
                /* Length in 32 bit units */
                pdss_stat->rx_unchunk_len = DIV_ROUND_UP(((pd_hdr_t)pd->rx_header).hdr.data_size, 4);
                if(pdss_stat->rx_unchunk_len > MAX_EXTD_PKT_WORDS)
                {
                    pdss_stat->rx_unchunk_len = MAX_EXTD_PKT_WORDS;
                }

                pdss_stat->rx_unchunk_count = 0;
                pdss_stat->rx_read_location = 0;
            }
            /* Extended message detected */
            /* Clear interrupt */
            pd->intr2 = PDSS_INTR2_CHUNK_DET | PDSS_INTR2_EXTENDED_MSG_DET;
        }

        if(pd->intr0_masked & PDSS_INTR0_RX_SRAM_HALF_END)
        {
            /* Store data in extended buf and update count*/
            pd_phy_read_data_from_mem(port);
            pd->intr0 = PDSS_INTR0_RX_SRAM_HALF_END;
        }

#endif /* CCG_PD_REV3_ENABLE */

        if (pd->intr0_masked & PDSS_INTR0_RX_STATE_IDLE)
        {
            rval =  pd->intr0;

            if (rval & PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE)
            {
                if (((rval & PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE) != 0) &&
                        ((rval & PDSS_INTR0_EOP_ERROR) == 0))
                {
                    if (pdss_stat->tx_done == true)
                    {
                        pdss_stat->tx_done = false;

                        /* Stop retries due to CRC countdown expiry. */
                        pd->rx_expect_goodcrc_msg |= PDSS_RX_EXPECT_GOODCRC_MSG_DISABLE_RX_CRC_TIMER;

                        pd->intr0_mask &= ~TX_INTERRUPTS;
                        pd->intr0 = TX_INTERRUPTS;

                        /* Successful transmission notification. */
                        pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_SUCCESS);
                    }
                }
            }

            if (rval & PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE)
            {
                if ((rval & PDSS_INTR0_EOP_ERROR) == 0)
                {
                    pdss_stat->tx_done = false;

                    /* Disable and clear PDSS_INTR0_CC_NO_VALID_DATA_DETECTED. */
                    pd->intr0_mask &= ~PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;
                    pd->intr0 = PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

                    /*
                     * Copy the received packet and analyze it. If the packet
                     * is valid with good msg id and if a Tx message is active,
                     * stop the Tx message and send a tx discard response to the
                     * upper layer. Also do not create a packet received event
                     * just now.
                     */
                    pdss_stat->rx_pkt.sop = (((pd->status & PDSS_STATUS_SOP_TYPE_DETECTED_MASK) >>
                                PDSS_STATUS_SOP_TYPE_DETECTED_POS) - 1);

                    /* Copy out the header from the PD hardware. */
                    pdss_stat->rx_pkt.hdr.val = pd->rx_header;
#if CCG_PD_REV3_ENABLE
                    if(dpm_stat->spec_rev_sop_live <= PD_REV2)
                    {
                        /* Ignore reserved bits */
                        pdss_stat->rx_pkt.hdr.val &= ~(PD_MSG_HDR_REV2_IGNORE_MASK);
                    }
#else
                    pdss_stat->rx_pkt.hdr.val &= ~(PD_MSG_HDR_REV2_IGNORE_MASK);
#endif /* CCG_PD_REV3_ENABLE */

                    /* Copy the data from the hardware buffer to the software buffer. */
                    pdss_stat->rx_pkt.len = pdss_stat->rx_pkt.hdr.hdr.len;
                    pdss_stat->rx_pkt.msg = pdss_stat->rx_pkt.hdr.hdr.msg_type;
                    pdss_stat->rx_pkt.data_role = pdss_stat->rx_pkt.hdr.hdr.data_role;

#if CCG_PD_REV3_ENABLE
                    if(pdss_stat->rx_unchunked == false)
                    {
                        mem_copy_word((uint32_t*)pdss_stat->rx_pkt.dat,
                                (uint32_t*)pd->rx_mem_data, pdss_stat->rx_pkt.len);
                    }
                    else
                    {
                        pd_phy_read_data_from_mem(port);
                    }
#else
                    for (i = 0; i < pdss_stat->rx_pkt.len; i++)
                    {
                        pdss_stat->rx_pkt.dat[i].val = pd->rx_mem_data[i];
                    }
#endif /* CCG_PD_REV3_ENABLE*/
                    pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG);
                }
            }

            pd->intr0 = RCV_INTR_MASK;

#if CCG_PD_REV3_ENABLE
            pdss_stat->rx_unchunked = false;
            pd->intr2 = PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET | PDSS_INTR2_RX_SRAM_OVER_FLOW;
#endif /* CCG_PD_REV3_ENABLE */
        }

        if (pd->intr0_masked & PDSS_INTR0_TX_GOODCRC_MSG_DONE)
        {
            /* Create a packet received event. */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG_CMPLT);
            pd->intr0 = PDSS_INTR0_TX_GOODCRC_MSG_DONE;
        }

        if (pd->intr0_masked & PDSS_INTR0_COLLISION_TYPE3)
        {
            /* Create a packet received event. */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG_CMPLT);
            pd->intr0 = PDSS_INTR0_COLLISION_TYPE3;
        }

        if (pd->intr0_masked & PDSS_INTR0_CC_NO_VALID_DATA_DETECTED)
        {
            /* Disable the interrupt. */
            pd->intr0_mask &= ~PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;
            pd->intr0 = PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

            /* Notify the protocol layer to stop the phy busy max limit timer. */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_PHY_IDLE);
        }

        /*
         * Tx interrupt handling.
         */

        if(pd->intr0_masked & PDSS_INTR0_TX_SRAM_HALF_END)
        {
            if (pd_phy_load_data_in_mem(port, false) == false)
            {
                pd->intr0_mask &= ~PDSS_INTR0_TX_SRAM_HALF_END;
            }
            pd->intr0 = PDSS_INTR0_TX_SRAM_HALF_END;
        }

        if (pd->intr0_masked & PDSS_INTR0_CRC_RX_TIMER_EXP)
        {
            pdss_stat->tx_done = false;

            if (pdss_stat->retry_cnt < 0)
            {
                /* Transmission failed. */

                pd->intr0_mask &= ~TX_INTERRUPTS;
                pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_FAILED);
            }
            else
            {
                if (pdss_stat->retry_cnt != 0)
                {
                    /* Clear and enable the TX retry enable cleared interrupt if required */
                    pd->intr0 = PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
                    /* Delay to remove any race */
                    CyDelayUs(5);
                    if (pd->tx_ctrl & PDSS_TX_CTRL_TX_RETRY_ENABLE)
                    {
                        pd->intr0_mask |= PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
                    }
                    else
                    {
                        /* Delay so that IP works otherwise if clear and set is too fast
                         * retry can fail */
                        CyDelayUs(5);
                        pd->tx_ctrl |= PDSS_TX_CTRL_TX_RETRY_ENABLE;
                    }
                }
                pdss_stat->retry_cnt--;
            }
            pd->intr0 = PDSS_INTR0_CRC_RX_TIMER_EXP;
        }

        if (pd->intr0_masked & PDSS_INTR0_TX_RETRY_ENABLE_CLRD)
        {
            CyDelayUs(5);
            /* Enable retry. */
            pd->tx_ctrl |= PDSS_TX_CTRL_TX_RETRY_ENABLE;

            /* Disable interrupts. */
            pd->intr0_mask &= ~PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
            pd->intr0 = PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
        }

        if (pd->intr0_masked & (PDSS_INTR0_COLLISION_TYPE1 | PDSS_INTR0_COLLISION_TYPE2))
        {
            /*
             * Notify the protocol layer so that it can start a timer so as to
             * avoid an infinite wait on the channel going idle.
             */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_COLLISION);

            /* Clear interrupts and enable the channel idle interrupt. */
            pd->intr0 = (PDSS_INTR0_COLLISION_TYPE1 | PDSS_INTR0_COLLISION_TYPE2 |
                    PDSS_INTR0_CC_NO_VALID_DATA_DETECTED);
            pd->intr0_mask |= PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

        }

        /*
         * Reset interrupt handling.
         */
        if (pd->intr0_masked & PDSS_INTR0_TX_HARD_RST_DONE)
        {
            pdss_stat->tx_done = false;
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_RST_SUCCESS);
            pd->intr0_mask &= ~RST_TX_INTERRUPTS;
            pd->intr0 = RST_TX_INTERRUPTS;
        }

        if (pd->intr0_masked & PDSS_INTR0_COLLISION_TYPE4)
        {
            pdss_stat->tx_done = false;
            pd->intr0_mask &= ~RST_TX_INTERRUPTS;
            pd->intr0 = RST_TX_INTERRUPTS;
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_RST_COLLISION);
        }
    }

    if (pd->intr2_masked != 0)
    {
        /* Handle the queue interrupt, instead of specific plug/unplug/irq interrupts. */
        if ((pd->intr2_masked & PDSS_INTR2_HPD_QUEUE) != 0)
        {
            /* Clear the interrupt and send callbacks for all queued events. */
            pd->intr2 = PDSS_INTR2_HPD_QUEUE;

#if CCG_HPD_RX_ENABLE
            if (hpd_cbks[port] != NULL)
            {
                i = pd->hpd_queue;
                if (HPD_GET_EVENT_0(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_0(i));
                if (HPD_GET_EVENT_1(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_1(i));
                if (HPD_GET_EVENT_2(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_2(i));
                if (HPD_GET_EVENT_3(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_3(i));
            }
#endif /* CCG_HPD_RX_ENABLE */
        }

        if ((pd->intr2_masked & PDSS_INTR2_HPDT_COMMAND_DONE) != 0)
        {
            /* Clear the interrupt and send the callback. */
            pd->intr2 = PDSS_INTR2_HPDT_COMMAND_DONE;
            if (hpd_cbks[port] != NULL)
                hpd_cbks[port] (port, HPD_COMMAND_DONE);
        }

#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
        if(pd->intr2_masked & (PDSS_INTR2_SWAP_IRQ | PDSS_INTR2_SWAP_UNPLUGED) )
        {
            /* Disable frs receive interrupts */
            pd->intr2_mask &= ~(PDSS_INTR2_SWAP_IRQ | PDSS_INTR2_SWAP_UNPLUGED);

            /* Disable the swap controller */
            pd->swap_ctrl1 |= PDSS_SWAP_CTRL1_RESET_SWAP_STATE;

            /* Disable TX discard on swap */
            pd->debug_cc_1 &= ~PDSS_TX_STOP_ON_SWAP_MASK;

            /* Stop any ongoing transmission */
            pd_phy_reset_rx_tx_sm (port);

            /* Clear pending rx interrupts */
            pd->intr0 = RCV_INTR_MASK | PDSS_INTR0_COLLISION_TYPE3 |PDSS_INTR0_TX_GOODCRC_MSG_DONE;
            pdss_stat->rx_unchunked = false;
            pd->intr2 = PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET | PDSS_INTR2_RX_SRAM_OVER_FLOW;

            /* Turn Off the consumer fet */
            dpm_stat->app_cbk->psnk_disable(port, NULL);
            dpm_stat->skip_scan = true;

            if(pdss_stat->pd_phy_cbk != NULL)
            {
                pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_FRS_SIG_RCVD);
            }

            /* Cannot clear interrupt here as this will cause auto fet turn on
             * to assume no FRS signal */
            CyIntClearPending(PD_PORT0_INTR0 + port);
        }

        if(pd->intr2_masked & PDSS_INTR2_VSWAP_VBUS_LESS_5_DONE)
        {
#if (!VBUS_FET_INTERNAL_CTRL)
            /* If internal FET control is not available, we need to manually enable the provider FET. */
            if (port == TYPEC_PORT_0_IDX)
                APP_VBUS_SRC_FET_ON_P1();
#if CCG_PD_DUALPORT_ENABLE
            else
                APP_VBUS_SRC_FET_ON_P2();
#endif /* CCG_PD_DUALPORT_ENABLE */
#endif /* (!VBUS_FET_INTERNAL_CTRL) */

            pd_frs_rx_disable(port);
        }
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
        if(pd->intr2_masked & PDSS_INTR2_SWAP_COMMAND_DONE)
        {
            pd_frs_tx_disable(port);
            dpm_stat->fr_tx_en_live = false;

            /* Stop any ongoing transmission */
            pd_phy_reset_rx_tx_sm (port);

            /* Change Rp to allow sink to initiate AMS */
            typec_change_rp(port, RP_TERM_RP_CUR_3A);

            /* Turn On the sink fet */
            dpm_stat->app_cbk->psnk_enable(port);
            /* Stop sourcing power. */
            dpm_stat->app_cbk->psrc_disable(port, NULL);

            if(pdss_stat->pd_phy_cbk != NULL)
            {
                pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_FRS_SIG_SENT);
            }
        }

#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */

    }
}

/*
 * Type C functionality.
 */
void pd_typec_dis_up_dn_cmp_filter(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->intr_1_cfg &= ~(PDSS_INTR_1_CFG_VCMP_DN_FILT_EN | PDSS_INTR_1_CFG_VCMP_UP_FILT_EN);
}

void pd_typec_en_dp_dn_cmp_filter(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->intr_1_cfg |= (PDSS_INTR_1_CFG_VCMP_DN_FILT_EN | PDSS_INTR_1_CFG_VCMP_UP_FILT_EN);
}

ccg_status_t pd_typec_init(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_HYST_MODE | PDSS_CC_CTRL_0_EN_HYST | PDSS_CC_CTRL_0_CMP_LA_VSEL_MASK);
    pd->cc_ctrl_0 |= (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS);

    /*
     * Up/Down comparators filter will only be enabled before going to
     * deepsleep and disabled after coming out of deepsleep.
     */
    pd_typec_dis_up_dn_cmp_filter(port);

    /* Disable filter on comparator 1 and comparator 2 outputs. */
    pd->intr_1_cfg &= ~(PDSS_INTR_1_CFG_CMP_OUT1_FILT_EN | PDSS_INTR_1_CFG_CMP_OUT2_FILT_EN);

    /* Always enable the pump. */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP | PDSS_PUMP_CTRL_BYPASS_LV);

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_typec_start(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

#if (NO_OF_TYPEC_PORTS == 1)
    pd->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;

    /* Power up the block. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;

    /* Enable PUMP */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);
    CyDelayUs(50);

#endif /* (NO_OF_TYPEC_PORTS == 1) */

    /* Enable V5V change detect interrupt. */
    pd->intr1         = PDSS_INTR1_V5V_CHANGED;
    pd->intr1_mask   |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
    pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_COMP;

    pd->cc_ctrl_0 |= (PDSS_CC_CTRL_0_CMP_EN | PDSS_CC_CTRL_0_RX_EN);

    return CCG_STAT_SUCCESS;
}

void pd_typec_rd_enable(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t temp;

#if (NO_OF_TYPEC_PORTS == 1)
    pd->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;

    /* Power up the block. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;
#endif /* (NO_OF_TYPEC_PORTS == 1) */

    pd->cc_ctrl_0 |= (PDSS_CC_CTRL_0_CMP_EN | PDSS_CC_CTRL_0_RX_EN);

    /* Enable Rd on both CC lines. */
    temp = pd->cc_ctrl_0;
    temp |= (PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC1_DB_DIS);
    temp |= (PDSS_CC_CTRL_0_RD_CC2_EN | PDSS_CC_CTRL_0_RD_CC2_DB_DIS);
    temp &= ~PDSS_CC_CTRL_0_DFP_EN;
    pd->cc_ctrl_0 = temp;
}

ccg_status_t pd_typec_stop(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Clear and disable the V5V changed detect interrupt. */
    pd->intr1       = PDSS_INTR1_V5V_CHANGED;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;

#if (NO_OF_TYPEC_PORTS == 1)

    /* Power down the block. */
    pd->cc_ctrl_0 |= PDSS_CC_CTRL_0_PWR_DISABLE;

    /* Disable PUMP */
    pd->pump_ctrl |= (PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);

    /* Turn off references. */
    pd->dpslp_ref_ctrl |= PDSS_DPSLP_REF_CTRL_PD_DPSLP;

    /* Turn off comparators. */
    pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_EN | PDSS_CC_CTRL_0_RX_EN);

#endif /* (NO_OF_TYPEC_PORTS == 1) */

    return CCG_STAT_SUCCESS;
}

void pd_typec_snk_update_trim(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    dpm_status_t* dpm_stat = dpm_get_status(port);

    if (dpm_stat->cur_port_role == PRT_ROLE_SINK)
    {
        pd->s8usbpd_trim_0 &= ~PDSS_S8USBPD_TRIM_0_TX_TRIM_MASK;
        if (dpm_stat->snk_cur_level == RD_3A)
        {
            pd->s8usbpd_trim_0 |= (TRIM0_TX_TRIM_VALUE_3A <<
                    PDSS_S8USBPD_TRIM_0_TX_TRIM_POS);
        }
    }
}

void pd_typec_en_rp(uint8_t port, uint8_t channel, rp_term_t rp_val)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t *rptrim_table_p = (uint8_t *)SFLASH_PD_RPTRIM_TABLE_ADDR;
    uint8_t rp_mode;
    uint32_t temp;
    rp_mode = rp_val;

    /* Check whether the new Rp trim table is valid. If not, use the old table address. */
    if ((rptrim_table_p[0] != SFLASH_PD_RPTRIM_TABLE_SIG0) || (rptrim_table_p[1] != SFLASH_PD_RPTRIM_TABLE_SIG1))
    {
#if CCG_PD_DUALPORT_ENABLE
        if (port != TYPEC_PORT_0_IDX)
            rptrim_table_p = (uint8_t *)SFLASH_PD1_RPTRIM_ADDR_OLD;
        else
#endif /* CCG_PD_DUALPORT_ENABLE */
            rptrim_table_p = (uint8_t *)SFLASH_PD0_RPTRIM_ADDR_OLD;
    }
    else
    {
#if CCG_PD_DUALPORT_ENABLE
        if (port != TYPEC_PORT_0_IDX)
            rptrim_table_p = (uint8_t *)SFLASH_PD1_RPTRIM_ADDR_NEW;
        else
#endif /* CCG_PD_DUALPORT_ENABLE */
            rptrim_table_p = (uint8_t *)SFLASH_PD0_RPTRIM_ADDR_NEW;
    }

#if (NO_OF_TYPEC_PORTS == 1)

    /* Enable PUMP */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);

#endif /* (NO_OF_TYPEC_PORTS == 1) */

    pd->s8usbpd_trim_0 &= ~PDSS_S8USBPD_TRIM_0_TX_TRIM_MASK;

    /* CDT 257771 fix for trim values */
    pd->s8usbpd_trim_1 &= ~PDSS_S8USBPD_TRIM_1_RP_CC1_TRIM_MASK;
    pd->s8usbpd_trim_2 &= ~PDSS_S8USBPD_TRIM_2_RP_CC2_TRIM_MASK;

    /* Set cc trim from sflash */
    pd->s8usbpd_trim_1 |= rptrim_table_p[rp_val];
    pd->s8usbpd_trim_2 |= rptrim_table_p[rp_val];

    if (rp_val == RP_TERM_RP_CUR_3A)
    {
        /* Transmitter trim */
        pd->s8usbpd_trim_0 |= (TRIM0_TX_TRIM_VALUE_3A <<
                PDSS_S8USBPD_TRIM_0_TX_TRIM_POS);
        rp_mode++;
    }

    /* Set Rp mode and enable references for source operation. */
    temp = pd->cc_ctrl_0;
    temp &= ~PDSS_CC_CTRL_0_RP_MODE_MASK;
    temp |= (rp_mode << PDSS_CC_CTRL_0_RP_MODE_POS) | PDSS_CC_CTRL_0_DFP_EN;

    if (channel == CC_CHANNEL_1)
    {
        temp |= PDSS_CC_CTRL_0_RP_CC1_EN;
    }
    else
    {
        temp |= PDSS_CC_CTRL_0_RP_CC2_EN;
    }
    pd->cc_ctrl_0 = temp;
}

void pd_typec_dis_rp(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (channel == CC_CHANNEL_1)
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_RP_CC1_EN;
    }
    else
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_RP_CC2_EN;
    }
}

void pd_typec_en_dpslp_rp(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->cc_ctrl_1 |= PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
}

void pd_typec_dis_dpslp_rp(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->cc_ctrl_1 &= ~PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
}

void pd_typec_en_deadbat_rd(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t temp;

    temp = pd->cc_ctrl_0;

    /* Re-enable dead battery Rd. */
    temp &= ~(PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS);

    /* Remove trimmed Rd. */
    temp &= ~(PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC2_EN);

    pd->cc_ctrl_0 = temp;
}

void pd_typec_en_rd(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t temp;

#if (NO_OF_TYPEC_PORTS == 1)

    /* Disable PUMP */
    pd->pump_ctrl |= (PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);

#endif /* (NO_OF_TYPEC_PORTS == 1) */

    temp = pd->cc_ctrl_0;
    if (channel == CC_CHANNEL_1)
    {
        temp |= (PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC1_DB_DIS);
    }
    else
    {
        temp |= (PDSS_CC_CTRL_0_RD_CC2_EN | PDSS_CC_CTRL_0_RD_CC2_DB_DIS);
    }

    temp &= ~PDSS_CC_CTRL_0_DFP_EN;
    pd->cc_ctrl_0 = temp;
}

void pd_typec_dis_rd(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (channel == CC_CHANNEL_1)
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_RD_CC1_EN;
        pd->cc_ctrl_0 |= PDSS_CC_CTRL_0_RD_CC1_DB_DIS;
    }
    else
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_RD_CC2_EN;
        pd->cc_ctrl_0 |= PDSS_CC_CTRL_0_RD_CC2_DB_DIS;
    }
}

/* Returns the current status on the CC line (rp_cc_status_t or rd_cc_status_t). */
static uint8_t pd_typec_get_rp_rd_status(uint8_t port, uint8_t channel, bool rd_idx)
{
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t rval = 0;
    uint32_t temp;
    uint8_t out;
    uint32_t status;
    bool change = false;

    /* Set default output. */
    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        out = RP_OPEN;
    }
    else
    {
        if (rd_idx == 0)
        {
            out = RD_RA;
        }
        else
        {
            out = RD_USB;
        }
    }

    /* Connect both the Up/Dn comparators to the active CC line. */
    if (channel == CC_CHANNEL_2)
    {
        rval = (PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2);
    }
    temp = pd->cc_ctrl_0 & (PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2);

    if ( temp != rval)
    {
        pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2);
        pd->cc_ctrl_0 |= rval;

        change = true;
    }

    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        /*
         * Set the threshold of the Dn comparator to Ra level and the Up
         * comparator to Rp open level.
         */
        rval = ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS) |
            ((thresholds[dpm_stat->src_cur_level_live][1]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS);
    }
    else
    {
        /* Set the Dn comparator to vRdUSB and the Up comparator to vRd1.5A. */
        rval = ((thresholds[RD_ROW_NO][rd_idx]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS) |
            ((thresholds[RD_ROW_NO][rd_idx + 1]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS);
    }

    temp = pd->cc_ctrl_0 & (PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK | PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK);
    if (temp != rval)
    {
        pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK | PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK);
        pd->cc_ctrl_0 |= rval;
        change = true;
    }

    if (change == true)
    {
        /* Delay to allow references to settle. */
        CyDelayUs (50);
    }

    status = pd->status;
    if (((status & PDSS_STATUS_VCMP_DN_STATUS) != 0) && ((status & PDSS_STATUS_VCMP_UP_STATUS) == 0))
    {
        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            out = RP_RD;
        }
        else
        {
            out = RD_1_5A;
        }
    }

    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        if (((status & PDSS_STATUS_VCMP_DN_STATUS) == 0) && ((status & PDSS_STATUS_VCMP_UP_STATUS) == 0))
        {
            out = RP_RA;
        }
    }

    if (dpm_stat->cur_port_role == PRT_ROLE_SINK)
    {
        if (((status & PDSS_STATUS_VCMP_DN_STATUS) != 0) && ((status & PDSS_STATUS_VCMP_UP_STATUS) != 0))
        {
            out = RD_3A;
        }
    }

    return out;
}

cc_state_t pd_typec_get_cc_status(uint8_t port)
{
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t polarity = dpm_stat->polarity;
    cc_state_t new_state;
    uint8_t i;

    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        if ((dpm_stat->attach) && (pd->status & (PDSS_STATUS_TX_BUSY | PDSS_STATUS_CC_DATA_VALID)))
        {
            new_state = dpm_stat->cc_old_status ;
        }
        else
        {
            /* Scan both CC lines: the active CC line should be scanned last. */
            if (polarity)
            {
                new_state.cc[0] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_1, 0);
                new_state.cc[1] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_2, 0);
            }
            else
            {
                new_state.cc[1] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_2, 0);
                new_state.cc[0] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_1, 0);
            }
        }
    }
    else
    {
        if (dpm_stat->attach == true)
        {
            if (pd->status & (PDSS_STATUS_TX_BUSY | PDSS_STATUS_CC_DATA_VALID))
            {
                new_state = dpm_stat->cc_status;
            }
            else
            {
                if(dpm_stat->snk_rp_detach_en == true)
                {
                    new_state.cc[polarity] = pd_typec_get_rp_rd_status(port, polarity, 0);
                }
                if((new_state.cc[polarity] != RD_RA) ||(dpm_stat->snk_rp_detach_en == false))
                {
                    /* Only the active CC line needs to be scanned. */
                    new_state.cc[polarity] = pd_typec_get_rp_rd_status(port, polarity, 1);
                }
                new_state.cc[dpm_stat->rev_pol] = RD_RA;
            }
        }
        else
        {
            for (i = 0; i < 2; i++)
            {
                /* Scan CC[i] with threshold vRa and vRdUsb. */
                new_state.cc[i] = pd_typec_get_rp_rd_status(port, i, 0);
                if (new_state.cc[i] != RD_RA)
                {
                    /* Scan CC[i] again with vRdusb and vRd1.5A to determine correct Rp value. */
                    new_state.cc[i] = pd_typec_get_rp_rd_status(port, i, 1);
                }
            }
        }
    }

    return new_state;
}

void pd_typec_set_polarity (uint8_t port, bool polarity)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (polarity == 0)
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CC_1V2;
    }
    else
    {
        pd->cc_ctrl_0 |= PDSS_CC_CTRL_0_CC_1V2;
    }
}

ccg_status_t pd_vconn_enable(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    if (channel == CC_CHANNEL_1)
    {
        pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_SWITCH_CC1;
    }
    else
    {
        pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_SWITCH_CC2;
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_vconn_disable(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    if (channel == CC_CHANNEL_1)
    {
        pd->pfet300_ctrl &= ~PDSS_PFET300_CTRL_EN_SWITCH_CC1;
    }
    else
    {
        pd->pfet300_ctrl &= ~PDSS_PFET300_CTRL_EN_SWITCH_CC2;
    }

    return CCG_STAT_SUCCESS;
}

bool pd_is_vconn_present(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    if (channel == CC_CHANNEL_1)
    {
        if (pd->pfet300_ctrl & PDSS_PFET300_CTRL_EN_SWITCH_CC1)
        {
            return true;
        }
    }
    else
    {
        if (pd->pfet300_ctrl & PDSS_PFET300_CTRL_EN_SWITCH_CC2)
        {
            return true;
        }
    }

    return false;
}

ccg_status_t hpd_receive_init(uint8_t port, hpd_event_cbk_t cbk)
{
#if CCG_HPD_RX_ENABLE
    PPDSS_REGS_T pd;

    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    if ((hpd_transmit_enable[port]) || (hpd_receive_enable[port]))
    {
        return CCG_STAT_BUSY;
    }

    /* PD block should have been enabled. */
    pd = gl_pdss[port];
    if ((pd->ctrl & PDSS_CTRL_IP_ENABLED) == 0)
    {
        return CCG_STAT_NOT_READY;
    }

    /* Store the callback pointer. */
    hpd_cbks[port] = cbk;
    hpd_receive_enable[port] = true;

    /* Configure the relevant GPIO for HPD functionality. */
    if (port == 0)
    {
        gpio_set_drv_mode (HPD_P0_PORT_PIN, GPIO_DM_HIZ_DIGITAL);
        hsiom_set_config (HPD_P0_PORT_PIN, HPD_HSIOM_SETTING);
    }
    else
    {
        gpio_set_drv_mode (HPD_P1_PORT_PIN, GPIO_DM_HIZ_DIGITAL);
        hsiom_set_config (HPD_P1_PORT_PIN, HPD_HSIOM_SETTING);
    }

    /* Set the default values for the HPD config settings. */
    pd->hpd_ctrl1 = PDSS_HPD_CTRL1_DEFAULT_VALUE;
    pd->hpd_ctrl2 = PDSS_HPD_CTRL2_DEFAULT;
    pd->hpd_ctrl3 = PDSS_HPD_CTRL3_DEFAULT_VALUE;
    pd->hpd_ctrl4 = PDSS_HPD_CTRL4_DEFAULT;
    pd->hpd_ctrl5 = PDSS_HPD_CTRL5_DEFAULT;

    pd->hpd_queue = pd->hpd_queue;

    /* Enable the HPD queue interrupt. */
    pd->intr2 = PDSS_INTR2_MASK_HPD_QUEUE_MASK;
    pd->intr2_mask |= PDSS_INTR2_MASK_HPD_QUEUE_MASK;

    /*
     * CDT 245126 workaround.
     * Enable the HPDIN_CHANGE interrupt with both edge detection. As HPD module
     * can't detect HPD events across deep sleep, this interrupt is used to ensure
     * that device doesn't go back to deepsleep after HPD status changes.
     * HPD RX activity timer is started when this interrupt fires and device
     * doesn't go back to deep sleep until timer is running or a queue interrupt
     * fires.
     */
    pd->intr_1_cfg |= (3 << PDSS_INTR_1_CFG_HPDIN_CFG_POS);
    /* Disable HPD IN filter. */
    pd->intr_1_cfg &= ~(PDSS_INTR_1_CFG_HPDIN_FILT_EN);
    /* Clear and then enable the interrupt. */
    pd->intr1 = PDSS_INTR1_HPDIN_CHANGED;
    pd->intr1_mask |= PDSS_INTR1_HPDIN_CHANGED;

    /* Enable the HPD function and the bring the HPD receiver out of reset. */
    pd->ctrl = (pd->ctrl & ~(PDSS_CTRL_HPD_DIRECTION | PDSS_CTRL_HPDT_ENABLED)) |
        PDSS_CTRL_HPD_ENABLED;
    pd->hpd_ctrl1 &= ~(PDSS_HPD_CTRL1_RESET_HPD_STATE);
#endif /* CCG_HPD_RX_ENABLE */

    return CCG_STAT_SUCCESS;
}

bool hpd_receive_get_status(uint8_t port)
{
    bool ret = false;
#if CCG_HPD_RX_ENABLE
    PPDSS_REGS_T pd;

    /* If the HPD receive block is turned ON, get the current state of the signal. */
    if (hpd_receive_enable[port])
    {
        pd = gl_pdss[port];
        ret = ((pd->status & PDSS_STATUS_HPD_STATUS) != 0) ? true : false;
    }
#endif /* CCG_HPD_RX_ENABLE */

    return ret;
}

ccg_status_t hpd_transmit_init(uint8_t port, hpd_event_cbk_t cbk)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (hpd_transmit_enable[port])
    {
        return CCG_STAT_BUSY;
    }

    /* Store the callback pointer. */
    hpd_cbks[port] = cbk;
    hpd_transmit_enable[port] = true;

    /* Configure the relevant GPIO for HPD functionality. */
    if (port == 0)
    {
        gpio_set_drv_mode(HPD_P0_PORT_PIN, GPIO_DM_STRONG);
        hsiom_set_config(HPD_P0_PORT_PIN, HPD_HSIOM_SETTING);
    }
    else
    {
        gpio_set_drv_mode(HPD_P1_PORT_PIN, GPIO_DM_STRONG);
        hsiom_set_config(HPD_P1_PORT_PIN, HPD_HSIOM_SETTING);
    }

    /* Set the default values for the HPDT config settings. */
    pd->hpdt_ctrl1 = PDSS_HPDT_CTRL1_DEFAULT;
    pd->hpdt_ctrl2 = PDSS_HPDT_CTRL2_DEFAULT;

    /* Enable the HPD queue interrupt. */
    pd->intr2 = PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK;
    pd->intr2_mask |= PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK;

    /* Enable the HPDT function and the bring the HPD transmitter out of reset. */
    pd->ctrl = (pd->ctrl & ~(PDSS_CTRL_HPD_ENABLED)) |
        PDSS_CTRL_HPDT_ENABLED | PDSS_CTRL_HPD_DIRECTION;
    pd->hpdt_ctrl1 &= ~(PDSS_HPDT_CTRL1_RESET_HPDT_STATE);

    return CCG_STAT_SUCCESS;
}

void hpd_sleep_entry(uint8_t port)
{
    /* Configure the relevant pin for GPIO functionality. */
    if (hpd_transmit_enable[port])
    {
        if (port == 0)
        {
            hsiom_set_config(HPD_P0_PORT_PIN, HSIOM_MODE_GPIO);
        }
        else
        {
            hsiom_set_config(HPD_P1_PORT_PIN, HSIOM_MODE_GPIO);
        }
    }
}

void hpd_wakeup(uint8_t port, bool value)
{
    PPDSS_REGS_T pd;

    /* PD block should be turned on already. */
    pd = gl_pdss[port];

    /* Configure the relevant GPIO for HPD functionality. */
    if (hpd_transmit_enable[port])
    {
        /* Set the default values for the HPDT config settings. */
        if (value)
        {
            /* Start HPD off in the high state and queue a PLUG event. */
            pd->hpdt_ctrl1 = PDSS_HPDT_CTRL1_DEFAULT | PDSS_HPDT_CTRL1_DEFAULT_LEVEL;
            pd->hpdt_ctrl2 = PDSS_HPDT_CTRL2_DEFAULT;
        }
        else
        {
            /* Start HPD off in the low state. */
            pd->hpdt_ctrl1 = PDSS_HPDT_CTRL1_DEFAULT;
            pd->hpdt_ctrl2 = PDSS_HPDT_CTRL2_DEFAULT;
        }

        /*
         * Bring the HPDT block out of reset. We need a small delay here to
         * ensure that there is no glitch on the HPD line.
         */
        CyDelayUs(5);
        pd->hpdt_ctrl1 &= ~(PDSS_HPDT_CTRL1_RESET_HPDT_STATE);

        if (port == 0)
        {
            hsiom_set_config(HPD_P0_PORT_PIN, HPD_HSIOM_SETTING);
        }
        else
        {
            hsiom_set_config(HPD_P1_PORT_PIN, HPD_HSIOM_SETTING);
        }
    }
}

#if CCG_HPD_RX_ENABLE
/*
 * CDT 245126 workaround.
 * This routine implements the workaround before entering deep sleep.
 *
 * Details:
 * HPD RX module can't detect HPD High to Low and IRQ transitions across
 * deepsleep. Workaround is this:
 *
 * Before entering deepsleep, check if HPD is high. If yes, enable HPD
 * TX module with default value of HIGH. Enable HPD loopback. Configure HPD
 * RX to detect a high with minimum time possible (<50us). After this enter
 * deepsleep. Once device enters deep sleep, HPD RX module loses all memory.
 * Now, when HPD goes low, HPD IN wakeup interrupt will wake up the device.
 * HPD RX module starts with HPD RX status as LOW. Due to loopback enabled and HPD
 * TX driving high, HPD RX will see HPD Connect event. Then disable the loopback
 * and revert HPD RX settings. From this point, HPD RX will look at actual
 * HPD status and will capture HPD events correctly.
 *
 * FW also uses a HPD RX activity timer. Whenever HPD IN interrupt fires, start this
 * timer with maximum HPD event time period. This is currently set to 5ms and can be fine
 * tuned if required. This ensures that on any HPD activity, device remains active
 * till the HPD event is captured in HPD RX queue. Stop the timer once HPD Queue
 * interrupt fires.
 */

/*
 * This flag keeps track of HPD Connection status. It is used in HPD CHANGE wakeup
 * interrupt.
 */
bool gl_hpd_state[NO_OF_TYPEC_PORTS];

void hpd_rx_sleep_entry(uint8_t port, bool hpd_state)
{
    PPDSS_REGS_T pd;
    pd = gl_pdss[port];

    /* Store HPD Connection status. */
    gl_hpd_state[port] = hpd_state;

    if (hpd_receive_enable[port])
    {
        /* If HPD is connected, implement the CDT 245126 workaround. */
        if (hpd_state)
        {
            /* Ensure default level of HPD TX is high. */
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_DEFAULT_LEVEL;
            /* Enable HPD TX. */
            pd->ctrl |= PDSS_CTRL_HPDT_ENABLED;

            /* Enable HPD loop back. */
            pd->hpd_ctrl1 |= PDSS_HPD_CTRL1_LOOPBACK_EN;

            /* Update HPD RX Stable High minimum time to a very small value (<50us) */
            pd->hpd_ctrl3 = ((pd->hpd_ctrl3 & ~PDSS_HPD_CTRL3_STABLE_HIGH_MASK)
                    | (1 << PDSS_HPD_CTRL3_STABLE_HIGH_POS));
            pd->hpd_ctrl5 = ((pd->hpd_ctrl5 & ~PDSS_HPD_CTRL5_LONG_HIGH_MASK)
                    | (1 << PDSS_HPD_CTRL5_LONG_HIGH_POS));

            /*
             * Disable HPD Queue interrupt. There is no point in using Queue interrupt
             * for HPD high detection once device wakes up. We can poll for this.
             */
            pd->intr2 = PDSS_INTR2_HPD_QUEUE;
            pd->intr2_mask &= ~PDSS_INTR2_MASK_HPD_QUEUE_MASK;
        }
    }
}

/* CDT 245126 workaround: This routine implements the wakeup portion of workaround. */
void hpd_rx_wakeup(uint8_t port)
{
    PPDSS_REGS_T pd;
    pd = gl_pdss[port];
    uint8_t timeout = 0;

    if (hpd_receive_enable[port])
    {
        /* Revert the settings only if Loopback was enabled before entering deep sleep. */
        if (pd->hpd_ctrl1 & PDSS_HPD_CTRL1_LOOPBACK_EN)
        {
            /*
             * Wait for the HPD Queue connect event. This is the fake HPD Queue connect
             * event triggered due to CDT 245126 workaround. 20us wait is enough.
             */
            while ((HPD_GET_EVENT_0(pd->hpd_queue) != HPD_EVENT_PLUG) && (timeout < 20))
            {
                CyDelayUs (1);
                timeout++;
            }

            /* Ensure that HPD RX high time is reset back to default. */
            pd->hpd_ctrl3 = PDSS_HPD_CTRL3_DEFAULT_VALUE;
            pd->hpd_ctrl5 = PDSS_HPD_CTRL5_DEFAULT;
            /* Ensure that Loopback is disabled. */
            pd->hpd_ctrl1 &= ~PDSS_HPD_CTRL1_LOOPBACK_EN;
        }
        /* Enable the HPD Queue interrupt to capture true HPD interrupts from now on. */
        pd->intr2 = PDSS_INTR2_HPD_QUEUE;
        pd->intr2_mask |= PDSS_INTR2_MASK_HPD_QUEUE_MASK;
    }
}

bool is_hpd_rx_state_idle(uint8_t port)
{
    /* If timer is running, HPD RX module is busy. */
    return (!timer_is_running (port, HPD_RX_ACTIVITY_TIMER_ID));
}

#endif /* CCG_HPD_RX_ENABLE */

ccg_status_t hpd_deinit(uint8_t port)
{
    PPDSS_REGS_T pd;

    if ((!hpd_transmit_enable[port]) && (!hpd_receive_enable[port]))
    {
        return CCG_STAT_FAILURE;
    }

    if (hpd_transmit_enable[port])
    {
        /* Make sure that the HPD signal is driven to zero. */
        if (port == 0)
        {
            gpio_set_value(HPD_P0_PORT_PIN, 0);
        }
        else
        {
            gpio_set_value(HPD_P1_PORT_PIN, 0);
        }
    }

    /* Disable all HPD related interrupts. */
    pd = gl_pdss[port];
    pd->intr2 = PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK | PDSS_INTR2_MASK_HPD_QUEUE_MASK;
    pd->intr2_mask &= ~(PDSS_INTR2_MASK_HPD_QUEUE_MASK | PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK);

    pd->intr1 = PDSS_INTR1_HPDIN_CHANGED;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_HPDIN_CHANGED_MASK;

    /* Disable both HPD transmit and receive blocks. */
    pd->ctrl &= ~(PDSS_CTRL_HPDT_ENABLED | PDSS_CTRL_HPD_ENABLED);

    hpd_cbks[port] = NULL;
    hpd_transmit_enable[port] = false;
    hpd_receive_enable[port] = false;

    if (port == 0)
    {
        hsiom_set_config(HPD_P0_PORT_PIN, HSIOM_MODE_GPIO);
    }
    else
    {
        hsiom_set_config(HPD_P1_PORT_PIN, HSIOM_MODE_GPIO);
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t hpd_transmit_sendevt(uint8_t port, hpd_event_type_t evtype,
        bool wait)
{
    PPDSS_REGS_T pd;

    /* Wait is currently not supported. */
    (void)wait;

    pd = gl_pdss[port];
    if ((pd->hpdt_ctrl1 & PDSS_HPDT_CTRL1_COMMAND_START) != 0)
    {
        return CCG_STAT_BUSY;
    }

    /* Update HPD-out as required. */
    switch (evtype)
    {
        case HPD_EVENT_UNPLUG:
            pd->hpdt_ctrl1 = (pd->hpdt_ctrl1 & ~PDSS_HPDT_CTRL1_COMMAND_MASK);
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_COMMAND_START;
            break;

        case HPD_EVENT_PLUG:
            pd->hpdt_ctrl1 = (pd->hpdt_ctrl1 & ~PDSS_HPDT_CTRL1_COMMAND_MASK) | (1 << PDSS_HPDT_CTRL1_COMMAND_POS);
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_COMMAND_START;
            break;

        case HPD_EVENT_IRQ:
            pd->hpdt_ctrl1 = (pd->hpdt_ctrl1 & ~PDSS_HPDT_CTRL1_COMMAND_MASK) | (2 << PDSS_HPDT_CTRL1_COMMAND_POS);
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_COMMAND_START;
            break;

        default:
            return CCG_STAT_BAD_PARAM;
    }

#ifdef CCGX_CDT225123_WORKAROUND
    /* CDT 225123 Workaround: Wait for 5 us and clear the command start bit. */
    CyDelayUs(5);
    pd->hpdt_ctrl1 &= ~PDSS_HPDT_CTRL1_COMMAND_START;
#endif /* CCGX_CDT225123_WORKAROUND */

    return CCG_STAT_SUCCESS;
}

/******************** PD block ADC functionality *****************************/

uint8_t pd_adc_volt_to_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt)
{
    uint32_t threshold;

    threshold = (((uint32_t)volt * PD_ADC_NUM_LEVELS) / gl_pdss_status[port].adc_vddd_mv[adc_id]);

    if (threshold < PD_ADC_LEVEL_MIN_THRESHOLD)
    {
        threshold = PD_ADC_LEVEL_MIN_THRESHOLD;
    }
    if (threshold > PD_ADC_LEVEL_MAX_THRESHOLD)
    {
        threshold = PD_ADC_LEVEL_MAX_THRESHOLD;
    }

    return (uint8_t)threshold;
}

uint16_t pd_adc_level_to_volt(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level)
{
    uint32_t threshold;

    threshold = (((uint32_t)level * gl_pdss_status[port].adc_vddd_mv[adc_id]) / PD_ADC_NUM_LEVELS);

    return (uint16_t)threshold;
}

uint16_t pd_adc_get_vbus_voltage(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level)
{
    uint32_t result;

    result = (((uint32_t)level * gl_pdss_status[port].adc_vddd_mv[adc_id] * pd_vbus_mon_divider)/PD_ADC_NUM_LEVELS);

    return result;
}

uint8_t pd_get_vbus_adc_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt,
        int8_t per)
{
    int32_t threshold;

    threshold = volt;
    threshold = (threshold + ((threshold / 100) * per));

    /* Remove negative numbers. */
    if (threshold < 0)
    {
        threshold = 0;
    }

    /* Convert volts to ADC units. */
    return pd_adc_volt_to_level(port, adc_id, (threshold / pd_vbus_mon_divider));
}

ccg_status_t  pd_adc_free_run_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /*
Note: This typically only gets called with one ADC-ID.
Hence not using ADC register pointers to update the configuration.
     */
    if (adc_id == PD_ADC_ID_0)
    {
        /* Disable interrupts. */
        pd->intr1_mask &= ~PDSS_INTR1_CMP_OUT1_CHANGED;
        pd->intr1 = PDSS_INTR1_CMP_OUT1_CHANGED;

        pd->adc1_ctrl = level | PDSS_ADC1_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC1_CTRL_VSEL_POS) & PDSS_ADC1_CTRL_VSEL_MASK);

    }
    else if(adc_id == PD_ADC_ID_1)
    {
        /* Disable interrupts. */
        pd->intr1_mask &= ~PDSS_INTR1_CMP_OUT2_CHANGED;
        pd->intr1 = PDSS_INTR1_CMP_OUT2_CHANGED;

        pd->adc2_ctrl = level | PDSS_ADC2_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC2_CTRL_VSEL_POS) & PDSS_ADC2_CTRL_VSEL_MASK);
    }

    return CCG_STAT_SUCCESS;
}


void pd_adc_comparator_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level, PD_ADC_INT_T int_cfg, PD_ADC_CB_T cb)
{
    uint8_t state;
    uint32_t reg_val;
    bool out;
    PPDSS_REGS_T pd = gl_pdss[port];

    state = CyEnterCriticalSection();
    gl_pdss_status[port].adc_cb[adc_id] = cb;

    /*
Note: This typically only gets called with one ADC-ID.
Hence not using ADC register pointers to update the configuration.
     */
    if (cb != NULL)
    {
        if (adc_id == PD_ADC_ID_0)
        {
            /* Load the interrupt configuration. */
            reg_val = pd->intr_1_cfg & ~PDSS_INTR_1_CFG_ADC1_CFG_MASK;
            pd->intr_1_cfg = (int_cfg << PDSS_INTR_1_CFG_ADC1_CFG_POS) |
                reg_val;

            pd->adc1_ctrl = level | PDSS_ADC1_CTRL_ADC_ISO_N |
                ((input << PDSS_ADC1_CTRL_VSEL_POS) & PDSS_ADC1_CTRL_VSEL_MASK);

            /* Delay 10us for the input selection to stabilize. */
            CyDelayUs(10);

            /* Clear comparator interrupts. */
            pd->intr1 = PDSS_INTR1_CMP_OUT1_CHANGED;

            /* Enable comparator interrupts. */
            pd->intr1_mask |= PDSS_INTR1_CMP_OUT1_CHANGED;
            if (pd->adc1_ctrl & PDSS_ADC1_CTRL_CMP_OUT)
            {
                out = true;
            }
            else
            {
                out = false;
            }

            if (((int_cfg ==  PD_ADC_INT_FALLING) && (out == false)) ||
                    ((int_cfg ==  PD_ADC_INT_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr1_set |= PDSS_INTR1_CMP_OUT1_CHANGED;
            }
        }
        else if (adc_id == PD_ADC_ID_1)
        {
            /* Load the interrupt configuration. */
            reg_val = pd->intr_1_cfg & ~PDSS_INTR_1_CFG_ADC2_CFG_MASK;
            pd->intr_1_cfg = (int_cfg << PDSS_INTR_1_CFG_ADC2_CFG_POS) |
                reg_val;

            pd->adc2_ctrl = level | PDSS_ADC2_CTRL_ADC_ISO_N |
                ((input << PDSS_ADC2_CTRL_VSEL_POS) & PDSS_ADC2_CTRL_VSEL_MASK);

            /* Delay 10us for the input selection to stabilize. */
            CyDelayUs(10);

            /* Clear comparator interrupts. */
            pd->intr1 = PDSS_INTR1_CMP_OUT2_CHANGED;

            /* Enable comparator interrupts. */
            pd->intr1_mask |= PDSS_INTR1_CMP_OUT2_CHANGED;

            if (pd->adc2_ctrl & PDSS_ADC2_CTRL_CMP_OUT)
            {
                out = true;
            }
            else
            {
                out = false;
            }

            if (((int_cfg == PD_ADC_INT_FALLING) && (out == false)) ||
                    ((int_cfg == PD_ADC_INT_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr1_set |= PDSS_INTR1_CMP_OUT2_CHANGED;
            }
        }
    }
    else
    {
        if (adc_id == PD_ADC_ID_0)
        {
            /* Revert register configuration. */
            pd->adc1_ctrl = PDSS_ADC1_CTRL_ADC_ISO_N | PDSS_ADC1_CTRL_PD_LV;

            /* Disable interrupts. */
            pd->intr1_mask &= ~PDSS_INTR1_CMP_OUT1_CHANGED;
            pd->intr1 = PDSS_INTR1_CMP_OUT1_CHANGED;
        }
        else if (adc_id == PD_ADC_ID_1)
        {
            /* Revert register configuration. */
            pd->adc2_ctrl = PDSS_ADC2_CTRL_ADC_ISO_N | PDSS_ADC2_CTRL_PD_LV;

            /* Disable interrupts. */
            pd->intr1_mask &= ~PDSS_INTR1_CMP_OUT2_CHANGED;
            pd->intr1 = PDSS_INTR1_CMP_OUT2_CHANGED;
        }
    }

    CyExitCriticalSection(state);
}

bool pd_adc_comparator_sample(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level)
{
    uint8_t state;
    uint8_t comp_out = true;
    uint32_t reg_adc_ctrl;
    uint32_t rval;
    bool out;
    PPDSS_REGS_T pd = gl_pdss[port];

    if (adc_id == PD_ADC_ID_0)
    {
        /* Store previous configuration and disable interrupts. */
        state = CyEnterCriticalSection();
        reg_adc_ctrl = pd->adc1_ctrl;

        /* Configure the input and level. */
        pd->adc1_ctrl = level | PDSS_ADC1_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC1_CTRL_VSEL_POS) & PDSS_ADC1_CTRL_VSEL_MASK);
        /* Delay 10us for the input selection to stabilize. */
        CyDelayUs(10);

        if (pd->adc1_ctrl & PDSS_ADC1_CTRL_CMP_OUT)
        {
            comp_out = false;
        }

        /* Revert register configuration. */
        pd->adc1_ctrl = reg_adc_ctrl;
        CyDelayUs(10);
        pd->intr1 = PDSS_INTR1_CMP_OUT1_CHANGED;

        if (((pd->intr1_mask & PDSS_INTR1_CMP_OUT1_CHANGED) != 0) &&
                ((reg_adc_ctrl & PDSS_ADC1_CTRL_PD_LV) == 0))
        {
            rval = (pd->intr_1_cfg & PDSS_INTR_1_CFG_ADC1_CFG_MASK) >> PDSS_INTR_1_CFG_ADC1_CFG_POS;

            if (pd->adc1_ctrl & PDSS_ADC1_CTRL_CMP_OUT)
            {
                out = true;
            }
            else
            {
                out = false;
            }

            if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
                    ((rval ==  PD_ADC_INT_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr1_set |= PDSS_INTR1_CMP_OUT1_CHANGED;
            }
        }

        CyExitCriticalSection(state);
    }
    else
    {
        /* Store previous configuration and disable interrupts. */
        state = CyEnterCriticalSection();
        reg_adc_ctrl = pd->adc2_ctrl;

        /* Configure the input and level. */
        pd->adc2_ctrl = (level | PDSS_ADC2_CTRL_ADC_ISO_N |
                ((input << PDSS_ADC2_CTRL_VSEL_POS) & PDSS_ADC2_CTRL_VSEL_MASK));
        /* Delay 10us for the input selection to stabilize. */
        CyDelayUs(10);

        if (pd->adc2_ctrl & PDSS_ADC2_CTRL_CMP_OUT)
        {
            comp_out = false;
        }

        /* Revert register configuration. */
        pd->adc2_ctrl = reg_adc_ctrl;
        pd->intr1 = PDSS_INTR1_CMP_OUT2_CHANGED;
        CyDelayUs(10);

        if (((pd->intr1_mask & PDSS_INTR1_CMP_OUT2_CHANGED) != 0) &&
                ((reg_adc_ctrl & PDSS_ADC2_CTRL_PD_LV) == 0))
        {
            rval = (pd->intr_1_cfg & PDSS_INTR_1_CFG_ADC2_CFG_MASK) >> PDSS_INTR_1_CFG_ADC2_CFG_POS;

            if (pd->adc2_ctrl & PDSS_ADC2_CTRL_CMP_OUT)
            {
                out = true;
            }
            else
            {
                out = false;
            }

            if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
                    ((rval ==  PD_ADC_INT_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr1_set |= PDSS_INTR1_CMP_OUT2_CHANGED;
            }
        }

        CyExitCriticalSection(state);
    }

    return comp_out;
}

bool pd_adc_get_comparator_status(uint8_t port, PD_ADC_ID_T adc_id)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (adc_id == PD_ADC_ID_0)
    {
        if (pd->adc1_ctrl & PDSS_ADC1_CTRL_CMP_OUT)
        {
            return false;
        }
    }
    else if (adc_id == PD_ADC_ID_1)
    {
        if (pd->adc2_ctrl & PDSS_ADC2_CTRL_CMP_OUT)
        {
            return false;
        }
    }

    return true;
}

uint8_t pd_adc_sample(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input)
{
    uint8_t state;
    uint32_t volatile timeout = 0;
    uint8_t level = 0;
    uint32_t reg_adc_ctrl;
    uint32_t rval;
    bool out;
    PPDSS_REGS_T pd = gl_pdss[port];

#if 1
    /*
Note: Memory optimization to be tested.
This function could be called with both ADC IDs selected. Hence
access the registers through pointers to allow the use of common
code.
     */
    volatile uint32_t *adc_ctrl_reg_p;
    volatile uint32_t *adc_sar_ctrl_reg_p;
    uint32_t           adc_intr_mask;
    uint32_t           adc_intr_cfg;
    uint32_t           adc_intr_cfg_pos;

    if (adc_id == PD_ADC_ID_0)
    {
        adc_ctrl_reg_p     = &(pd->adc1_ctrl);
        adc_sar_ctrl_reg_p = &(pd->adc1_sar_ctrl);
        adc_intr_mask      = PDSS_INTR1_CMP_OUT1_CHANGED;
        adc_intr_cfg       = PDSS_INTR_1_CFG_ADC1_CFG_MASK;
        adc_intr_cfg_pos   = PDSS_INTR_1_CFG_ADC1_CFG_POS;
    }
    else
    {
        adc_ctrl_reg_p     = &(pd->adc2_ctrl);
        adc_sar_ctrl_reg_p = &(pd->adc2_sar_ctrl);
        adc_intr_mask      = PDSS_INTR1_CMP_OUT2_CHANGED;
        adc_intr_cfg       = PDSS_INTR_1_CFG_ADC2_CFG_MASK;
        adc_intr_cfg_pos   = PDSS_INTR_1_CFG_ADC2_CFG_POS;
    }

    /* Store previous configuration and disable interrupts. */
    state = CyEnterCriticalSection();
    reg_adc_ctrl = *adc_ctrl_reg_p;

    /* Configure the input. */
    *adc_ctrl_reg_p = PDSS_ADC1_CTRL_ADC_ISO_N |
        ((input << PDSS_ADC1_CTRL_VSEL_POS) & PDSS_ADC1_CTRL_VSEL_MASK);
    *adc_sar_ctrl_reg_p |= PDSS_ADC1_SAR_CTRL_SAR_EN;

    /* Wait for sampling to complete or timeout. */
    while (((*adc_sar_ctrl_reg_p & PDSS_ADC1_SAR_CTRL_SAR_EN) != 0) && (timeout < PD_ADC_TIMEOUT_COUNT))
    {
        timeout++;
    }

    /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
    CyDelayUs(2);

    level = ((*adc_sar_ctrl_reg_p & PDSS_ADC1_SAR_CTRL_SAR_OUT_MASK) >>
            PDSS_ADC1_SAR_CTRL_SAR_OUT_POS);

    /* Revert register configuration. */
    *adc_ctrl_reg_p = reg_adc_ctrl;
    CyDelayUs(10);
    pd->intr1 = adc_intr_mask;

    if( ((pd->intr1_mask & adc_intr_mask) != 0) && ((reg_adc_ctrl & PDSS_ADC1_CTRL_PD_LV) == 0))
    {
        rval = (pd->intr_1_cfg & adc_intr_cfg) >> adc_intr_cfg_pos;

        if (*adc_ctrl_reg_p & PDSS_ADC1_CTRL_CMP_OUT)
        {
            out = true;
        }
        else
        {
            out = false;
        }

        if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
                ((rval ==  PD_ADC_INT_RISING) && (out == true)))
        {
            /* Raise an interrupt. */
            pd->intr1_set |= adc_intr_mask;
        }
    }

    CyExitCriticalSection(state);
#else
    /* Store previous configuration and disable interrupts. */
    state = CyEnterCriticalSection();

    if (adc_id == PD_ADC_ID_0)
    {
        reg_adc_ctrl = pd->adc1_ctrl;

        /* Configure the input. */
        pd->adc1_ctrl = PDSS_ADC1_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC1_CTRL_VSEL_POS) & PDSS_ADC1_CTRL_VSEL_MASK);
        pd->adc1_sar_ctrl |= PDSS_ADC1_SAR_CTRL_SAR_EN;

        /* Wait for sampling to complete or timeout. */
        while (((pd->adc1_sar_ctrl & PDSS_ADC1_SAR_CTRL_SAR_EN) != 0) && (timeout < PD_ADC_TIMEOUT_COUNT))
        {
            timeout++;
        }

        /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
        CyDelayUs(2);

        level = ((pd->adc1_sar_ctrl & PDSS_ADC1_SAR_CTRL_SAR_OUT_MASK) >>
                PDSS_ADC1_SAR_CTRL_SAR_OUT_POS);

        /* Revert register configuration. */
        pd->adc1_ctrl = reg_adc_ctrl;
        CyDelayUs(10);
        pd->intr1 = PDSS_INTR1_CMP_OUT1_CHANGED;

        if( ((pd->intr1_mask & PDSS_INTR1_CMP_OUT1_CHANGED) != 0) && ((reg_adc_ctrl & PDSS_ADC1_CTRL_PD_LV) == 0))
        {
            rval = (pd->intr_1_cfg & PDSS_INTR_1_CFG_ADC1_CFG_MASK) >> PDSS_INTR_1_CFG_ADC1_CFG_POS;

            if (pd->adc1_ctrl & PDSS_ADC1_CTRL_CMP_OUT)
            {
                out = true;
            }
            else
            {
                out = false;
            }

            if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
                    ((rval ==  PD_ADC_INT_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr1_set |= PDSS_INTR1_CMP_OUT1_CHANGED;
            }
        }
    }
    else
    {
        reg_adc_ctrl = pd->adc2_ctrl;

        /* Configure the input. */
        pd->adc2_ctrl = PDSS_ADC2_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC2_CTRL_VSEL_POS) & PDSS_ADC2_CTRL_VSEL_MASK);
        pd->adc2_sar_ctrl |= PDSS_ADC2_SAR_CTRL_SAR_EN;

        /* Wait for sampling to complete ot timeout */
        while (((pd->adc2_sar_ctrl & PDSS_ADC2_SAR_CTRL_SAR_EN) != 0) && (timeout < PD_ADC_TIMEOUT_COUNT))
        {
            timeout++;
        }

        /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
        CyDelayUs(2);

        level = ((pd->adc2_sar_ctrl & PDSS_ADC2_SAR_CTRL_SAR_OUT_MASK) >>
                PDSS_ADC2_SAR_CTRL_SAR_OUT_POS);

        /* Revert register configuration. */
        pd->adc2_ctrl = reg_adc_ctrl;
        CyDelayUs(10);
        pd->intr1 = PDSS_INTR1_CMP_OUT2_CHANGED;

        if( ((pd->intr1_mask & PDSS_INTR1_CMP_OUT2_CHANGED) != 0) && ((reg_adc_ctrl & PDSS_ADC2_CTRL_PD_LV) == 0))
        {
            rval = (pd->intr_1_cfg & PDSS_INTR_1_CFG_ADC2_CFG_MASK) >> PDSS_INTR_1_CFG_ADC2_CFG_POS;

            if (pd->adc2_ctrl & PDSS_ADC2_CTRL_CMP_OUT)
            {
                out = true;
            }
            else
            {
                out = false;
            }
            if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
                    ((rval ==  PD_ADC_INT_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr1_set |= PDSS_INTR1_CMP_OUT2_CHANGED;
            }
        }
    }

    CyExitCriticalSection(state);
#endif

    return level;
}

uint16_t pd_adc_calibrate(uint8_t port, PD_ADC_ID_T adc_id)
{
    uint8_t level;
    uint32_t threshold;

    level = pd_adc_sample(port, adc_id, PD_ADC_INPUT_BANDGAP);

    /* Check for zero. If level came out as zero, then do not calculate. */
    if (level != 0)
    {
        threshold = ((PD_ADC_BAND_GAP_VOLT_MV * PD_ADC_NUM_LEVELS) / (uint32_t)level);
        gl_pdss_status[port].adc_vddd_mv[adc_id] = (uint16_t)threshold;
    }

    return gl_pdss_status[port].adc_vddd_mv[adc_id];
}

ccg_status_t pd_adc_init(uint8_t port, PD_ADC_ID_T adc_id)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (adc_id == PD_ADC_ID_0)
    {
        /* Enable the ADC and power it down. */
        pd->adc1_ctrl = PDSS_ADC1_CTRL_ADC_ISO_N | PDSS_ADC1_CTRL_PD_LV;

        /* Calibrate the ADC before starting. */
        pd_adc_calibrate (port, PD_ADC_ID_0);
    }
    else
    {
        /* Enable the ADC and power it down. */
        pd->adc2_ctrl = PDSS_ADC2_CTRL_ADC_ISO_N | PDSS_ADC2_CTRL_PD_LV;

        /* Calibrate the ADC before starting. */
        pd_adc_calibrate (port, PD_ADC_ID_1);
    }

    return CCG_STAT_SUCCESS;
}

bool pd_frs_rx_enable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
    uint8_t intr_state;
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Set the VBus detach comparator threshold to Vsafe5V */
    uint8_t level = pd_get_vbus_adc_level(port, pd_hal_get_vbus_detach_adc(),
            VSAFE_5V, VSAFE_5V_FRS_SWAP_RX_MARGIN);

    /* Enable VSAFE5V comparator */
    pd_adc_free_run_ctrl(port, pd_hal_get_vbus_detach_adc(), pd_hal_get_vbus_detach_input(),
            level);

    /* Set the sink fet OFF settings as per current HW */
#ifdef CCG3
    /* TODO: Need to cater for flipped FET usage or remove flipped FET completely. */
    if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
    {
        pd->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
    }
    else /* PD_FET_DR_N_JN_FET */
    {
        pd->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_EN_LV_OFF_VALUE_MASK;
        pd->ngdo_ctrl_c |= PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
    }
#else /* CCG4PD3 */
    if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
    {
        pd->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
    }
    else
    {
        pd->ngdo_ctrl_c |= PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
    }
#endif /* CCG */

    /* Enable auto consumer fet Off based on SWAP signal */
    pd->ngdo_ctrl_c |= (PDSS_NGDO_CTRL_C_AUTO_MODE | PDSS_NGDO_CTRL_C_SEL_SWAP_VBUS_LESS_5);

    /* Set the source fet ON settings as per current HW */
#ifdef CCG3
    /* TODO: Need to cater for flipped FET usage or remove flipped FET completely. */
    if(pd_pctrl_drive == PD_FET_DR_P_JN_FET)
    {
        pd->ngdo_ctrl_p |= PDSS_NGDO_CTRL_P_PULLDN_EN_LV_ON_VALUE_MASK;
    }
    else /* PD_FET_DR_N_JN_FET */
    {
        pd->ngdo_ctrl_p |= PDSS_NGDO_CTRL_P_EN_LV_ON_VALUE_MASK;
        pd->ngdo_ctrl_p &= ~PDSS_NGDO_CTRL_P_PULLDN_EN_LV_ON_VALUE_MASK;
    }
#else /* CCG4PD3 */
    if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
    {
        pd->ngdo_ctrl_p |= PDSS_NGDO_CTRL_P_PULLDN_EN_LV_ON_VALUE_MASK;
    }
    else
    {
        pd->ngdo_ctrl_p &= ~PDSS_NGDO_CTRL_P_PULLDN_EN_LV_ON_VALUE_MASK;
    }
#endif /* CCG */

    /* Enable auto provider fet ON based on SWAP signal */
    pd->ngdo_ctrl_p |= (PDSS_NGDO_CTRL_P_AUTO_MODE | PDSS_NGDO_CTRL_P_SEL_SWAP_VBUS_LESS_5);

    /* Configure CC line voltage thresholds to detect frs signal */
    pd->swap_cntrl_0 = SWAP_CNTRL0_DFLT_VAL;

    /* See if cc polarity need update */
    if(dpm_stat->polarity == CC_CHANNEL_2)
    {
        pd->swap_cntrl_0 |= PDSS_SWAP_CNTRL_0_CMP_FS_CC1V2;
    }

    /* Now configure the Swap controller */
    pd->swap_ctrl1 = FRS_RX_SWAP_CTRL1_DFLT_VAL;
    pd->swap_ctrl2 = FRS_RX_SWAP_CTRL2_DFLT_VAL;
    pd->swap_ctrl3 = FRS_RX_SWAP_CTRL3_DFLT_VAL;
    pd->swap_ctrl4 = FRS_RX_SWAP_CTRL4_DFLT_VAL;
    pd->swap_ctrl5 = FRS_RX_SWAP_CTRL5_DFLT_VAL;
    /* Let thresholds settle */
    CyDelayUs(10);

    /* Take swap controller out of reset */
    pd->swap_ctrl1 &= ~PDSS_SWAP_CTRL1_RESET_SWAP_STATE;

    /* Set the vsafe5v comp signal source */
    pd->swap_ctrl0 = pd_hal_get_vbus_detach_adc() << FRS_RX_SWAP_CTRL0_SWAPR_SOURCE_SEL_POS;

    /* Enable necessary interrupts */
    pd->intr2 = (PDSS_INTR2_SWAP_IRQ | PDSS_INTR2_SWAP_UNPLUGED | PDSS_INTR2_VSWAP_VBUS_LESS_5_DONE);
    pd->intr2_mask |= PDSS_INTR2_SWAP_IRQ | PDSS_INTR2_SWAP_UNPLUGED |PDSS_INTR2_VSWAP_VBUS_LESS_5_DONE;

    /* Enable TX discard on swap */
    pd->debug_cc_1 |= PDSS_TX_STOP_ON_SWAP_MASK;

    intr_state = CyEnterCriticalSection();

    /* Enable the swap controller */
    pd->swap_ctrl0 |= PDSS_SWAP_CTRL0_SWAP_ENABLED;
    /* Block is not ready before 60us, to be safe we use 100us delay */
    CyDelayUs(100);
    /* Keep polling for 40us if low then an FRS */
    /* This timeout is set as per 48MHz CPU clock */
    uint32_t volatile timeout = 100;
    while(timeout--)
    {
        if(pd->status & PDSS_STATUS_VCMP_FS)
        {
            CyExitCriticalSection(intr_state);
            return true;
        }
    }
    /* Create FRS intr */
    pd->intr2_set |= PDSS_INTR2_SWAP_IRQ;
    CyExitCriticalSection(intr_state);

#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

    return true;
}

bool pd_frs_rx_disable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Disable TX discard on swap */
    pd->debug_cc_1 &= ~PDSS_TX_STOP_ON_SWAP_MASK;

    /* Turn Off VSafe5V ADC */
    pd_adc_comparator_ctrl(port, pd_hal_get_vbus_detach_adc(), 0, 0, 0, NULL);

    /* Disable the swap controller */
    pd->swap_ctrl1 |= PDSS_SWAP_CTRL1_RESET_SWAP_STATE;
    pd->swap_ctrl0 &= ~PDSS_SWAP_CTRL0_SWAP_ENABLED;

    /* Disable frs receive interrupts */
    pd->intr2_mask &= ~(PDSS_INTR2_SWAP_IRQ | PDSS_INTR2_SWAP_UNPLUGED | PDSS_INTR2_VSWAP_VBUS_LESS_5_DONE);
    pd->intr2 = PDSS_INTR2_SWAP_IRQ | PDSS_INTR2_SWAP_UNPLUGED | PDSS_INTR2_VSWAP_VBUS_LESS_5_DONE;

#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

    return true;
}

bool pd_frs_tx_enable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Configure FRS TX source */
    if(port == TYPEC_PORT_0_IDX)
    {
        /* Configuring a GPIO for trigering FRS signal */
        gpio_hsiom_set_config(APP_FRS_TX_GPIO_PORT_PIN_P1, HSIOM_MODE_P0_SWAPT_IN,
                GPIO_DM_HIZ_DIGITAL, 0);
    }
#if CCG_PD_DUALPORT_ENABLE
    if (port == TYPEC_PORT_1_IDX)
    {
        /* Configuring a GPIO for trigering FRS signal */
        gpio_hsiom_set_config(APP_FRS_TX_GPIO_PORT_PIN_P2, HSIOM_MODE_P1_SWAPT_IN,
                GPIO_DM_HIZ_DIGITAL, 0);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Configure for Auto FRS signal transmitting */
    pd->debug_cc_1 &= ~(PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC1 |
            PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC2 |
            PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN | PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN);

    /* Enable TX discard on swap */
    pd->debug_cc_1 |= PDSS_TX_STOP_ON_SWAP_MASK;

    pd->swap_ctrl0 = PDSS_SWAP_CTRL0_SWAP_ENABLED;
    pd->swap_ctrl0 |= FRS_TX_SOURCE_GPIO;

    pd->swap_ctrl0 |= PDSS_SWAP_CTRL0_SWAPT_POLARITY;
    pd->swapt_ctrl1 = FRS_TX_SWAP_CTRL1_DFLT_VAL;

    /* Enable necessary interrupts */
    pd->intr2 = PDSS_INTR2_SWAP_COMMAND_DONE;
    pd->intr2_mask |= PDSS_INTR2_SWAP_COMMAND_DONE;

    /* This delay is needed otherwise swap TX indefinitely short the cc line */
    CyDelayUs(10);

    /* Enable the swap tx  */
    pd->swapt_ctrl1 &= ~PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE;

    /* Set cc polarity for pulldowns */
    if(dpm_stat->polarity == CC_CHANNEL_2)
    {
        pd->debug_cc_1 &= ~(PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN);
        pd->debug_cc_1 |= (PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN);
    }
    else
    {
        pd->debug_cc_1 &= ~(PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN);
        pd->debug_cc_1 |= (PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN);
    }
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */
    return true;
}

bool pd_frs_tx_disable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Disable the swap controller */
    pd->swapt_ctrl1 = PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE;
    pd->swap_ctrl0 &= ~PDSS_SWAP_CTRL0_SWAP_ENABLED;

    /* Disable frs receive interrupts */
    pd->intr2_mask &= ~PDSS_INTR2_SWAP_COMMAND_DONE;
    pd->intr2 = PDSS_INTR2_SWAP_COMMAND_DONE;

    /* Disable pulldown */
    pd->debug_cc_1 &= ~(1u << 24u);
    pd->debug_cc_1 &= ~(1u << 23u);
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */

    return true;
}

#if ((defined(CCG3)) && (VBUS_OCP_ENABLE))
static void ocp_handler_wrapper(uint8_t port, timer_id_t id)
{
    (void)port;
    (void)id;
    vbus_ocp_handler(port);
}
#endif /* (defined(CCG3) && VBUS_OCP_ENABLE) */

/*************************** INTR1/3 Deepsleep Wakeup *************************/
void pdss_intr1_handler(uint8_t port)
{
    bool comp_out = true;
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
    uint32_t intr1_cause = pd->intr1_masked;

    pd->intr1 = intr1_cause;

    /*
     * This routine expects all interrupts which are triggered to be disabled
     * once they are fired, otherwise it can cause problems.
     */
    if (intr1_cause)
    {
        if (intr1_cause & PDSS_INTR1_CMP_OUT1_CHANGED)
        {
            /* Disable the interrupt. */
            pd->intr1_mask &= ~PDSS_INTR1_CMP_OUT1_CHANGED;

            /* Check status. */
            if (pd->adc1_ctrl & PDSS_ADC1_CTRL_CMP_OUT)
            {
                comp_out = false;
            }

            /* Report status. */
            if (pdss_stat->adc_cb[PD_ADC_ID_0] != NULL)
            {
                pdss_stat->adc_cb[PD_ADC_ID_0](port, comp_out);
            }
        }

        if (intr1_cause & PDSS_INTR1_CMP_OUT2_CHANGED)
        {
            /* Disable the interrupt. */
            pd->intr1_mask &= ~PDSS_INTR1_CMP_OUT2_CHANGED;

            /* Check status. */
            if (pd->adc2_ctrl & PDSS_ADC2_CTRL_CMP_OUT)
            {
                comp_out = false;
            }

            /* Report status. */
            if (pdss_stat->adc_cb[PD_ADC_ID_1] != NULL)
            {
                pdss_stat->adc_cb[PD_ADC_ID_1](port, comp_out);
            }
        }

        if (pd->intr1_masked & PDSS_INTR1_VCMP_LA_CHANGED)
        {
            /* Disable the interrupt. */
            pd->intr1_mask &= ~PDSS_INTR1_VCMP_LA_CHANGED;
        }

        if (pd->intr1_masked & PDSS_INTR1_VCMP_UP_CHANGED)
        {
            /* Disable the interrupt. */
            pd->intr1_mask &= ~PDSS_INTR1_VCMP_UP_CHANGED;
        }

        if (pd->intr1_masked & PDSS_INTR1_VCMP_DN_CHANGED)
        {
            /* Disable the interrupt. */
            pd->intr1_mask &= ~PDSS_INTR1_VCMP_DN_CHANGED;
        }

        if (pd->intr1_masked & PDSS_INTR1_MASK_V5V_CHANGED_MASK)
        {
            if (gl_ccg_supply_changed_cb != NULL)
            {
                /* Check the current status of the V5V supply and make a callback to the app. layer. */
                if (pd->status & PDSS_STATUS_V5V_STATUS)
                {
                    gl_ccg_supply_changed_cb (port, CCG_SUPPLY_V5V, true);
                }
                else
                {
                    gl_ccg_supply_changed_cb (port, CCG_SUPPLY_V5V, false);
                }
            }
        }

        if (intr1_cause & PDSS_INTR1_HPDIN_CHANGED)
        {
#if CCG_HPD_RX_ENABLE
            /*
             * Start HPD ACTIVITY TIMER to prevent re-entry into deepsleep.
             * HPD RX hardware block can't detect HPD events if device enters
             * deepsleep while HPD state is changing (or if HPD is HIGH).
             * If HPD is not connected, timer period shall be 100ms
             * (this is the default minimum HPD Connect debounce time).
             * Otherwise, timer period is 5ms (this is large enough
             * to capture HPD LOW and IRQ events.
             */
            if (gl_hpd_state[port] == false)
            {
                /* Start a timer of 100ms to prevent deep sleep entry. */
                timer_start_wocb (port, HPD_RX_ACTIVITY_TIMER_ID,
                        HPD_RX_ACTIVITY_TIMER_PERIOD_MAX);
            }
            else
            {
                timer_start_wocb (port, HPD_RX_ACTIVITY_TIMER_ID,
                        HPD_RX_ACTIVITY_TIMER_PERIOD_MIN);
            }
#endif /* CCG_HPD_RX_ENABLE */
        }
#if (VCONN_OCP_ENABLE) && (CCG4_DOCK)
        if (intr1_cause & PDSS_INTR1_MASKED_V5V_CHANGED_MASKED)
        {
            /* Disable the interrupt. */
            pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;
            app_vconn_ocp_cbk(port, false);
        }
#endif /* VCONN_OCP_ENABLE && CCG4_DOCK */
    }

#ifdef CCG3
    if (pd->intr3_masked != 0)
    {
#if VBUS_OVP_ENABLE
        if (pd->intr3_masked & PDSS_INTR3_POS_OV_CHANGED)
        {
            /* Disable and clear UVOV interrupts. */
            pd->intr3_mask &= ~PDSS_INTR3_POS_OV_CHANGED;
            pd->intr3 = PDSS_INTR3_POS_OV_CHANGED;

            /* Report an OVP trip event. */
            if (gl_ccg3_ovp_cb != NULL)
            {
                gl_ccg3_ovp_cb(port, comp_out);
            }
        }
#endif /* VBUS_OVP_ENABLE */

#if VBUS_OCP_ENABLE
        if (pd->intr3_masked & PDSS_INTR3_POS_CSA_CHANGED)
        {
            /* Disable and clear CSA interrupts. */
            pd->intr3_mask &= ~PDSS_INTR3_POS_CSA_CHANGED;
            pd->intr3 = PDSS_INTR3_POS_CSA_CHANGED;

            if (gl_vbus_ocp_mode == VBUS_OCP_MODE_INT ||
                    gl_vbus_ocp_mode == VBUS_OCP_MODE_INT_AUTOCTRL)
            {
                vbus_ocp_handler(port);
            }
            else if (gl_vbus_ocp_mode == VBUS_OCP_MODE_INT_SW_DB)
            {
                /* Negative edge interrupt enable. */
                pd->intr3_mask |= PDSS_INTR3_NEG_CSA_CHANGED;

                /* Start the debounce timer. */
                timer_start(0, PD_OCP_DEBOUNCE_TIMER, gl_ocp_sw_db_ms, ocp_handler_wrapper);
            }
        }

        if (gl_vbus_ocp_mode == VBUS_OCP_MODE_INT_SW_DB)
        {
            if (pd->intr3_masked & PDSS_INTR3_NEG_CSA_CHANGED)
            {
                /* Stop the debounce timer. */
                timer_stop(0, PD_OCP_DEBOUNCE_TIMER);

                /* Enable positive edge interrupt. */
                pd->intr3_mask |= PDSS_INTR3_POS_CSA_CHANGED;

                /* Disable and clear the negative edge interrupt. */
                pd->intr3_mask &= ~PDSS_INTR3_NEG_CSA_CHANGED;
                pd->intr3 = PDSS_INTR3_NEG_CSA_CHANGED;
            }
        }
#endif /* VBUS_OCP_ENABLE */

#if BATTERY_CHARGING_ENABLE
        if (pd->intr3_masked & PDSS_INTR3_CHGDET_CHANGED)
        {
            /* Disable and clear UVOV interrupts. */
            pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;
            pd->intr3_cfg_0 &= ~PDSS_INTR3_CFG_0_CHGDET_CFG_MASK;
            pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;

            /* Report an OVP trip event. */
            if (pdss_stat->bc_phy_cbk != NULL)
            {
                pdss_stat->bc_phy_cbk(port, BC_EVT_CMP1_FIRE);
            }
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }
#endif /* CCG3 */
}

#if ((defined(CCG3)) || (defined(CCG4PD3)))
void pd_internal_pfet_on(uint8_t port, bool turn_on_seq)
{
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

    regval = pd->ngdo_ctrl_0;

    if (!turn_on_seq)
    {
#ifdef CCG3
        if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval |= VBUS_P_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_P_PLDN_EN_LV_0;
            regval |= VBUS_P_NGDO_EN_LV_0;
        }
#else /* CCG4PD3 */
        if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval |= VBUS_P_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval &= ~VBUS_P_PLDN_EN_LV_0;
        }
#endif /* CCG */
    }
    else
    {
#ifdef CCG3
        if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval |= VBUS_P_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_P_PLDN_EN_LV_1;
            regval |= VBUS_P_NGDO_EN_LV_1;
        }
#else /* CCG4PD3 */
        if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval |= VBUS_P_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval &= ~VBUS_P_PLDN_EN_LV_1;
        }
#endif /* CCG */
    }

    pd->ngdo_ctrl_0 = regval;

    /* Now control the second FET if enabled. */
    if (pd_dual_fet)
    {
        CyDelayUs(pd_ngdo_spacing * LF_CLK_CYCLE_US);

        if (!turn_on_seq)
        {
#ifdef CCG3
            if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval |= VBUS_P_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_P_PLDN_EN_LV_1;
                regval |= VBUS_P_NGDO_EN_LV_1;
            }
#else /* CCG4PD3 */
            if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval |= VBUS_P_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval &= ~VBUS_P_PLDN_EN_LV_1;
            }
#endif /* CCG */
        }
        else
        {
#ifdef CCG3
            if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval |= VBUS_P_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_P_PLDN_EN_LV_0;
                regval |= VBUS_P_NGDO_EN_LV_0;
            }
#else /* CCG4PD3 */
            if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval |= VBUS_P_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval &= ~VBUS_P_PLDN_EN_LV_0;
            }
#endif /* CCG */
        }

        pd->ngdo_ctrl_0 = regval;
    }

    /* Reset the edge detector to work around a silicon defect. */
    pd->ngdo_ctrl_p |= PDSS_NGDO_CTRL_P_RST_EDGE_DET;
    pd->ngdo_ctrl_p &= ~PDSS_NGDO_CTRL_P_RST_EDGE_DET;

}

void pd_internal_pfet_off(uint8_t port, bool turn_off_seq)
{
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

    regval = pd->ngdo_ctrl_0;

    if (!turn_off_seq)
    {
#ifdef CCG3
        if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval &= ~VBUS_P_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_P_NGDO_EN_LV_0;
            regval |= VBUS_P_PLDN_EN_LV_0;
        }
#else /* CCG4PD3 */
        if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval &= ~VBUS_P_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval |= VBUS_P_PLDN_EN_LV_0;
        }
#endif /* CCG */
    }
    else
    {
#ifdef CCG3
        if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval &= ~VBUS_P_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_P_NGDO_EN_LV_1;
            regval |= VBUS_P_PLDN_EN_LV_1;
        }
#else /* CCG4PD3 */
        if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval &= ~VBUS_P_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval |= VBUS_P_PLDN_EN_LV_1;
        }
#endif /* CCG */
    }

    pd->ngdo_ctrl_0 = regval;

    /* Now control the second FET if enabled. */
    if (pd_dual_fet)
    {
        CyDelayUs(pd_ngdo_spacing * LF_CLK_CYCLE_US);

        if (!turn_off_seq)
        {
#ifdef CCG3
            if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~VBUS_P_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_P_NGDO_EN_LV_1;
                regval |= VBUS_P_PLDN_EN_LV_1;
            }
#else /* CCG4PD3 */
            if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval &= ~VBUS_P_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval |= VBUS_P_PLDN_EN_LV_1;
            }
#endif /* CCG */
        }
        else
        {
#ifdef CCG3
            if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~VBUS_P_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_P_NGDO_EN_LV_0;
                regval |= VBUS_P_PLDN_EN_LV_0;
            }
#else /* CCG4PD3 */
            if (pd_pctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval &= ~VBUS_P_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval |= VBUS_P_PLDN_EN_LV_0;
            }
#endif /* CCG */
        }

        pd->ngdo_ctrl_0 = regval;
    }

    /* Reset the edge detector to work around a silicon defect. */
    pd->ngdo_ctrl_p &= ~(PDSS_NGDO_CTRL_P_AUTO_MODE |
            PDSS_NGDO_CTRL_P_SEL_SWAP_VBUS_LESS_5);
    pd->ngdo_ctrl_p |= PDSS_NGDO_CTRL_P_RST_EDGE_DET;
    pd->ngdo_ctrl_p &= ~PDSS_NGDO_CTRL_P_RST_EDGE_DET;

}

void pd_internal_cfet_on(uint8_t port, bool turn_on_seq)
{
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

    regval = pd->ngdo_ctrl_0;

    if (!turn_on_seq)
    {
#ifdef CCG3
        if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval |= VBUS_C_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_C_PLDN_EN_LV_0;
            regval |= VBUS_C_NGDO_EN_LV_0;
        }
#else /* CCG4PD3 */
        if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval |= VBUS_C_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval &= ~VBUS_C_PLDN_EN_LV_0;
        }
#endif /* CCG */
    }
    else
    {
#ifdef CCG3
        if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval |= VBUS_C_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_C_PLDN_EN_LV_1;
            regval |= VBUS_C_NGDO_EN_LV_1;
        }
#else /* CCG4PD3 */
        if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval |= VBUS_C_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval &= ~VBUS_C_PLDN_EN_LV_1;
        }
#endif /* CCG */
    }

    pd->ngdo_ctrl_0 = regval;

    /* Now control the second FET if enabled. */
    if (pd_dual_fet)
    {
        CyDelayUs(pd_ngdo_spacing * LF_CLK_CYCLE_US);

        if (!turn_on_seq)
        {
#ifdef CCG3
            if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval |= VBUS_C_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_C_PLDN_EN_LV_1;
                regval |= VBUS_C_NGDO_EN_LV_1;
            }
#else /* CCG4PD3 */
            if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval |= VBUS_C_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval &= ~VBUS_C_PLDN_EN_LV_1;
            }
#endif /* CCG */
        }
        else
        {
#ifdef CCG3
            if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval |= VBUS_C_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_C_PLDN_EN_LV_0;
                regval |= VBUS_C_NGDO_EN_LV_0;
            }
#else /* CCG4PD3 */
            if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval |= VBUS_C_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval &= ~VBUS_C_PLDN_EN_LV_0;
            }
#endif /* CCG */
        }

        pd->ngdo_ctrl_0 = regval;
    }

    /* Reset the edge detector to work around a silicon defect. */
    pd->ngdo_ctrl_c |= PDSS_NGDO_CTRL_C_RST_EDGE_DET;
    pd->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_RST_EDGE_DET;

}

void pd_internal_cfet_off(uint8_t port, bool turn_off_seq)
{
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

    regval = pd->ngdo_ctrl_0;

    if (!turn_off_seq)
    {
#ifdef CCG3
        if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval &= ~VBUS_C_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_C_NGDO_EN_LV_0;
            regval |= VBUS_C_PLDN_EN_LV_0;
        }
#else /* CCG4PD3 */
        if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval &= ~VBUS_C_PLDN_EN_LV_0;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval |= VBUS_C_PLDN_EN_LV_0;
        }
#endif /* CCG */
    }
    else
    {
#ifdef CCG3
        if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
        {
            regval &= ~VBUS_C_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_N_JN_FET */
        {
            regval &= ~VBUS_C_NGDO_EN_LV_1;
            regval |= VBUS_C_PLDN_EN_LV_1;
        }
#else /* CCG4PD3 */
        if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
        {
            regval &= ~VBUS_C_PLDN_EN_LV_1;
        }
        else /* PD_FET_DR_ACTIVE_LOW */
        {
            regval |= VBUS_C_PLDN_EN_LV_1;
        }
#endif /* CCG */
    }

    pd->ngdo_ctrl_0 = regval;

    /* Now control the second FET if enabled. */
    if (pd_dual_fet)
    {
        CyDelayUs(pd_ngdo_spacing * LF_CLK_CYCLE_US);

        if (!turn_off_seq)
        {
#ifdef CCG3
            if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~VBUS_C_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_C_NGDO_EN_LV_1;
                regval |= VBUS_C_PLDN_EN_LV_1;
            }
#else /* CCG4PD3 */
            if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval &= ~VBUS_C_PLDN_EN_LV_1;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval |= VBUS_C_PLDN_EN_LV_1;
            }
#endif /* CCG */
        }
        else
        {
#ifdef CCG3
            if (pd_cctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~VBUS_C_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval &= ~VBUS_C_NGDO_EN_LV_0;
                regval |= VBUS_C_PLDN_EN_LV_0;
            }
#else /* CCG4PD3 */
            if (pd_cctrl_drive == PD_FET_DR_ACTIVE_HIGH)
            {
                regval &= ~VBUS_C_PLDN_EN_LV_0;
            }
            else /* PD_FET_DR_ACTIVE_LOW */
            {
                regval |= VBUS_C_PLDN_EN_LV_0;
            }
#endif /* CCG */
        }

        pd->ngdo_ctrl_0 = regval;
    }

    /* Reset the edge detector to work around a silicon defect. */
    pd->ngdo_ctrl_c &= ~(PDSS_NGDO_CTRL_C_AUTO_MODE | PDSS_NGDO_CTRL_C_SEL_SWAP_VBUS_LESS_5);

    pd->ngdo_ctrl_c |= PDSS_NGDO_CTRL_C_RST_EDGE_DET;
    pd->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_RST_EDGE_DET;
}
#endif /* ((defined(CCG3)) || (defined(CCG4PD3))) */

#ifdef CCG3
void pd_internal_vbus_discharge_on(uint8_t port)
{
    (void)port;
    PDSS->vbus_ctrl |= PDSS_VBUS_CTRL_DISCHG_EN;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    (void)port;
    PDSS->vbus_ctrl &= ~PDSS_VBUS_CTRL_DISCHG_EN;
}

#if VBUS_OCP_ENABLE
void pd_internal_vbus_ocp_en(uint8_t port, uint8_t av_bw, uint8_t vref_sel, bool pctrl,
        uint8_t mode, uint8_t debounce_ms)
{
    (void)port;
    PPDSS_REGS_T pd = gl_pdss[0];
    uint8_t state;
    uint32_t regval;

    state = CyEnterCriticalSection();

    gl_vbus_ocp_mode = mode;
    gl_ocp_sw_db_ms = debounce_ms;

    regval = pd->csa_ctrl;

    /* Power up the CSA block. */
    regval &= ~PDSS_CSA_CTRL_PD_CSA;
    /* Default operational settings. */
    regval |= PDSS_CSA_CTRL_SEL_OUT_D | PDSS_CSA_CTRL_CSA_ISO_N;
    /* Clear the gain, bw and vref settings. */
    regval &= ~(PDSS_CSA_CTRL_AV1_MASK | PDSS_CSA_CTRL_CSA_VREF_SEL_MASK | PDSS_CSA_CTRL_BW);
    pd->debug_cc_0 &= ~PDSS_DEBUG_CC_0_CSA_BW_1;

    /* Set CSA gain. */
    regval |=  (av_bw >> 2) << PDSS_CSA_CTRL_AV1_POS;
    /* Set Vref trim select. */
    regval |= vref_sel << PDSS_CSA_CTRL_CSA_VREF_SEL_POS;
    /* Set analog bandwidth, BW[0]. */
    regval ^= (-((bool)(av_bw & 1)) ^ regval) & PDSS_CSA_CTRL_BW;

    /* Write out CSA_CTRL. */
    pd->csa_ctrl = regval;
    /* Set analog bandwidth, BW[0]. */
    pd->debug_cc_0 ^= (-((bool)(av_bw & 2)) ^ pd->debug_cc_0) & PDSS_DEBUG_CC_0_CSA_BW_1;

    if (gl_vbus_ocp_mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        if (pctrl)
        {
            /*
             * Configure NGDO_CTRL_P to set auto shutoff for OCP.
             */
            regval = PDSS->ngdo_ctrl_p;

            /* Auto mode on. */
            regval |= PDSS_NGDO_CTRL_P_AUTO_MODE;
            /* Overcurrent detection to turn off the NGDO.*/
            regval |= PDSS_NGDO_CTRL_P_SEL_OC;
            /* Off value used by hardware in auto mode to turn off the NGDO. */
            if(pd_pctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~PDSS_NGDO_CTRL_P_PULLDN_EN_LV_OFF_VALUE_MASK;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval |= PDSS_NGDO_CTRL_P_PULLDN_EN_LV_OFF_VALUE_MASK;
                regval &= ~PDSS_NGDO_CTRL_P_EN_LV_OFF_VALUE_MASK;
            }

            PDSS->ngdo_ctrl_p = regval;
        }
        else
        {
            /*
             * Configure NGDO_CTRL_C to set auto shutoff for OCP.
             */
            regval = PDSS->ngdo_ctrl_c;

            /* Auto mode on. */
            regval |= PDSS_NGDO_CTRL_C_AUTO_MODE;
            /* Overcurrent detection to turn off the NGDO.*/
            regval |= PDSS_NGDO_CTRL_C_SEL_OC;
            /* Off value used by hardware in auto mode to turn off the NGDO. */
            if(pd_cctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval |= PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
                regval &= ~PDSS_NGDO_CTRL_C_EN_LV_OFF_VALUE_MASK;
            }

            PDSS->ngdo_ctrl_c = regval;
        }
    }

    /* Enable CSA positive edge filtering for 4 clk_lf cycles. */
    pd->intr3_cfg_0 = (pd->intr3_cfg_0 & ~PDSS_INTR3_CFG_0_CSA_POS_FILT_SEL_MASK) |
        (PDSS_INTR3_FILT_SEL_4_CLK_LF << PDSS_INTR3_CFG_0_CSA_POS_FILT_SEL_POS) |
        PDSS_INTR3_CFG_0_CSA_POS_FILT_EN;

    /* Clear interrupts. */
    pd->intr3 = PDSS_INTR3_POS_CSA_CHANGED | PDSS_INTR3_NEG_CSA_CHANGED;

    /* If the OCP_DET output is already high, flag it. */
    if (pd->ncell_status & PDSS_NCELL_STATUS_CSA_STATUS)
    {
        pd->intr3_set |= PDSS_INTR3_POS_CSA_CHANGED;
    }

    /* Enable interrupts. */
    pd->intr3_mask |= PDSS_INTR3_POS_CSA_CHANGED;

    CyExitCriticalSection(state);
}

void pd_internal_vbus_ocp_dis(uint8_t port, bool pctrl)
{
    (void)port;
    PPDSS_REGS_T pd = gl_pdss[0];
    uint8_t state;
    uint32_t regval;

    state = CyEnterCriticalSection();

    /* Default settings and power down. */
    pd->csa_ctrl = PDSS_CSA_CTRL_SEL_OUT_D | PDSS_CSA_CTRL_PD_CSA;

    /* Disable and clear interrupts. */
    pd->intr3_mask &= ~(PDSS_INTR3_POS_CSA_CHANGED | PDSS_INTR3_NEG_CSA_CHANGED);
    pd->intr3 = PDSS_INTR3_POS_CSA_CHANGED | PDSS_INTR3_NEG_CSA_CHANGED;

    if (gl_vbus_ocp_mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        /* No auto mode shutoff on overcurrent detect. */
        if (pctrl)
        {
            regval = pd->ngdo_ctrl_p;
            regval &= ~PDSS_NGDO_CTRL_P_SEL_OC;
            if (!(regval & (PDSS_NGDO_CTRL_P_SEL_OV | PDSS_NGDO_CTRL_P_SEL_UV)))
            {
                regval &= ~PDSS_NGDO_CTRL_P_AUTO_MODE;
            }
            pd->ngdo_ctrl_p = regval;
        }
        else
        {
            regval = pd->ngdo_ctrl_c;
            regval &= ~PDSS_NGDO_CTRL_C_SEL_OC;
            if (!(regval & (PDSS_NGDO_CTRL_C_SEL_OV | PDSS_NGDO_CTRL_C_SEL_UV)))
            {
                regval &= ~PDSS_NGDO_CTRL_C_AUTO_MODE;
            }
            pd->ngdo_ctrl_c = regval;
        }
    }

    CyExitCriticalSection(state);
}

#endif /* VBUS_OCP_ENABLE */

#if VBUS_OVP_ENABLE
void pd_internal_vbus_ovp_en(uint8_t port, uint16_t volt, int8_t per, PD_ADC_CB_T cb, bool pctrl,
        vbus_ovp_mode_t mode, uint8_t filter_sel)
{
    (void)port;
    (void)filter_sel;

    uint8_t level;
    int32_t threshold;
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[0];

    gl_vbus_ovp_mode = mode;

    /* Get threshold voltage level in mV. */
    threshold = volt;
    threshold = (threshold + (threshold * per) / 100);

    /* Remove negative numbers. */
    if (threshold < 0)
    {
        threshold = 0;
    }

    /*
     * Calculate OVP comparator threshold setting for CCG3 per Table 26,
     * s8usbpd_ver2 HardIP, *A.
     */
    if (threshold < UVOV_LADDER_BOT)
    {
        level = UVOV_CODE_BOT;
    }
    else if (threshold > UVOV_LADDER_TOP)
    {
        level = UVOV_CODE_TOP;
    }
    else if (threshold <= UVOV_LADDER_MID)
    {
        level = (uint8_t)((threshold - UVOV_LADDER_BOT) / UVOV_LO_STEP_SZ);
    }
    else
    {
        level = (uint8_t)(((threshold - UVOV_LADDER_MID) / UVOV_HI_STEP_SZ) + UVOV_CODE_MID);
    }

    /* Set up the OVP callback. */
    gl_ccg3_ovp_cb = cb;

    /* Clear OVP positive edge notification. */
    pd->intr3 = PDSS_INTR3_POS_OV_CHANGED;

    /* Configure the UVOV block. */
    regval = pd->uvov_ctrl & ~(PDSS_UVOV_CTRL_OV_IN_MASK | PDSS_UVOV_CTRL_PD_UVOV);
    regval |= PDSS_UVOV_CTRL_UVOV_ISO_N;
    pd->uvov_ctrl = (level << PDSS_UVOV_CTRL_OV_IN_POS) | regval;

    if (gl_vbus_ovp_mode == VBUS_OVP_MODE_UVOV_AUTOCTRL)
    {
        if (pctrl)
        {
            /*
             * Configure NGDO_CTRL_P to set auto shutoff for OVP.
             */
            regval = PDSS->ngdo_ctrl_p;

            /* Auto mode on. */
            regval |= PDSS_NGDO_CTRL_P_AUTO_MODE;

            /* Over-voltage detection to turn off the NGDO.*/
            regval |= PDSS_NGDO_CTRL_P_SEL_OV;

            /* Off value used by hardware in auto mode to turn off the NGDO. */
            if(pd_pctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~PDSS_NGDO_CTRL_P_PULLDN_EN_LV_OFF_VALUE_MASK;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval |= PDSS_NGDO_CTRL_P_PULLDN_EN_LV_OFF_VALUE_MASK;
                regval &= ~PDSS_NGDO_CTRL_P_EN_LV_OFF_VALUE_MASK;
            }

            PDSS->ngdo_ctrl_p = regval;
        }
        else
        {
            /*
             * Configure NGDO_CTRL_C to set auto shutoff for OVP.
             */
            regval = PDSS->ngdo_ctrl_c;

            /* Auto mode on. */
            regval |= PDSS_NGDO_CTRL_C_AUTO_MODE;

            /* Over-voltage detection to turn off the NGDO.*/
            regval |= PDSS_NGDO_CTRL_C_SEL_OV;

            /* Off value used by hardware in auto mode to turn off the NGDO. */
            if(pd_cctrl_drive == PD_FET_DR_P_JN_FET)
            {
                regval &= ~PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
            }
            else /* PD_FET_DR_N_JN_FET */
            {
                regval |= PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK;
                regval &= ~PDSS_NGDO_CTRL_C_EN_LV_OFF_VALUE_MASK;
            }

            PDSS->ngdo_ctrl_c = regval;
        }
    }

    /* Enable OV positive edge filtering for three CLK-LF cycles. */
    pd->intr3_cfg_0 = (pd->intr3_cfg_0 & ~PDSS_INTR3_CFG_0_OV_POS_FILT_SEL_MASK) |
        (0x03 << PDSS_INTR3_CFG_0_OV_POS_FILT_SEL_POS) |
        PDSS_INTR3_CFG_0_OV_POS_FILT_EN;

    /* Delay 10us for the input selection to stabilize. */
    CyDelayUs(10);

    /* If the OV_DET output is already high, flag it. */
    if (pd->ncell_status & PDSS_NCELL_STATUS_OV_STATUS)
    {
        pd->intr3_set |= PDSS_INTR3_POS_OV_CHANGED;
    }

    /* Enable OVP positive edge detection. */
    pd->intr3_mask |= PDSS_INTR3_POS_OV_CHANGED;
}

void pd_internal_vbus_ovp_dis(uint8_t port, bool pctrl)
{
    (void)port;
    PPDSS_REGS_T pd = gl_pdss[0];
    uint32_t regval;

    /* Clear AUTO MODE OVP detect. */
    if (gl_vbus_ovp_mode == VBUS_OVP_MODE_UVOV_AUTOCTRL)
    {
        regval = pd->ngdo_ctrl_p;
        regval &= ~PDSS_NGDO_CTRL_P_SEL_OV;
        if (!(regval & (PDSS_NGDO_CTRL_P_SEL_OC | PDSS_NGDO_CTRL_P_SEL_UV)))
        {
            regval &= ~PDSS_NGDO_CTRL_P_AUTO_MODE;
        }
        pd->ngdo_ctrl_p = regval;

        regval = pd->ngdo_ctrl_c;
        regval &= ~PDSS_NGDO_CTRL_C_SEL_OV;
        if (!(regval & (PDSS_NGDO_CTRL_C_SEL_OC | PDSS_NGDO_CTRL_C_SEL_UV)))
        {
            regval &= ~PDSS_NGDO_CTRL_C_AUTO_MODE;
        }
        pd->ngdo_ctrl_c = regval;
    }

    /* Disable the UVOV block, and set the OV threshold to 6V by default. */
    regval = pd->uvov_ctrl;
    regval = ((regval & ~PDSS_UVOV_CTRL_OV_IN_MASK) | (UVOV_CODE_6V0 << PDSS_UVOV_CTRL_OV_IN_POS));
    if (!pd_vbus_mon_internal)
    {
        /* Power down the block and isolate. */
        regval |= PDSS_UVOV_CTRL_PD_UVOV;
        regval &= ~PDSS_UVOV_CTRL_UVOV_ISO_N;
    }
    pd->uvov_ctrl = regval;

    /* Disable and clear UVOV interrupts. */
    pd->intr3_mask &= ~PDSS_INTR3_POS_OV_CHANGED;
    pd->intr3 = PDSS_INTR3_POS_OV_CHANGED;

    gl_ccg3_ovp_cb = NULL;
}

#endif /* VBUS_OVP_ENABLE */

ccg_status_t sbu_switch_configure(uint8_t port, sbu_switch_state_t sbu1_state, sbu_switch_state_t sbu2_state)
{
    PPDSS_REGS_T pd = gl_pdss[0];
    uint32_t regval = pd->sbu_ctrl;

    (void)port;

    /* Check that state values are withing allowed range. */
    if ((sbu1_state > SBU_CONNECT_AUX2) || (sbu2_state > SBU_CONNECT_AUX2))
    {
        /* Don't service the request. */
        return CCG_STAT_INVALID_ARGUMENT;
    }

    /* Configure SBU1 switch. */
    if (sbu1_state == SBU_NOT_CONNECTED)
    {
        /* SBU1 shall not be connected to AUX1/2. */
        regval &= ~(PDSS_SBU_CTRL_IN1_OUT1_EN | PDSS_SBU_CTRL_IN1_OUT2_EN);
    }
    else if (sbu1_state == SBU_CONNECT_AUX1)
    {
        /* SBU1 shall be connected to AUX1 and not to AUX2. */
        regval |= PDSS_SBU_CTRL_IN1_OUT1_EN;
        regval &= ~PDSS_SBU_CTRL_IN1_OUT2_EN;
    }
    else
    {
        /* SBU1 shall be connected to AUX2 and not to AUX1. */
        regval |= PDSS_SBU_CTRL_IN1_OUT2_EN;
        regval &= ~PDSS_SBU_CTRL_IN1_OUT1_EN;
    }

    /* Configure SBU2 switch. */
    if (sbu2_state == SBU_NOT_CONNECTED)
    {
        /* SBU2 shall not be connected to AUX1/2. */
        regval &= ~(PDSS_SBU_CTRL_IN2_OUT1_EN | PDSS_SBU_CTRL_IN2_OUT2_EN);
    }
    else if (sbu2_state == SBU_CONNECT_AUX1)
    {
        /* SBU2 shall be connected to AUX1 and not to AUX2. */
        regval |= PDSS_SBU_CTRL_IN2_OUT1_EN;
        regval &= ~PDSS_SBU_CTRL_IN2_OUT2_EN;
    }
    else
    {
        /* SBU2 shall be connected to AUX2 and not to AUX1. */
        regval |= PDSS_SBU_CTRL_IN2_OUT2_EN;
        regval &= ~PDSS_SBU_CTRL_IN2_OUT1_EN;
    }

    /* Store SBU1 and SBU2 states. */
    gl_sbu1_state = sbu1_state;
    gl_sbu2_state = sbu2_state;

    /* Update the switch register. */
    pd->sbu_ctrl = regval;
    return CCG_STAT_SUCCESS;
}

sbu_switch_state_t get_sbu1_switch_state(uint8_t port)
{
    (void)port;
    return gl_sbu1_state;
}

sbu_switch_state_t get_sbu2_switch_state(uint8_t port)
{
    (void)port;
    return gl_sbu2_state;
}

void aux_resistor_configure(uint8_t port, aux_resistor_config_t aux1_config,
        aux_resistor_config_t aux2_config)
{
    PPDSS_REGS_T pd = gl_pdss[0];
    uint32_t regval = pd->sbu_ctrl;

    (void)port;

    /* Chcek if resistor values passed are correct. */
    if (((aux1_config < AUX_NO_RESISTOR) || (aux1_config > AUX_1_470K_PD_RESISTOR)) ||
            (((aux2_config < AUX_2_100K_PU_RESISTOR) && (aux2_config != AUX_NO_RESISTOR))
             || (aux2_config > AUX_MAX_RESISTOR_CONFIG)))
    {
        /* Wrong configuration. */
        return;
    }

    /* Check if no resistor configuration for AUX1. */
    if (aux1_config == AUX_NO_RESISTOR)
    {
        regval &= ~(PDSS_SBU_CTRL_OUT1_1MEG_EN_PU | PDSS_SBU_CTRL_OUT1_100K_EN_PD
                | PDSS_SBU_CTRL_OUT1_470K_EN_PD);
    }
    else
    {
        /* Enable the requested resistor. */
        regval |= (1 << aux1_config);
    }

    /* Check if no resistor configuration for AUX2. */
    if (aux2_config == AUX_NO_RESISTOR)
    {
        regval &= ~(PDSS_SBU_CTRL_OUT2_100K_EN_PU | PDSS_SBU_CTRL_OUT2_4P7MEG_EN_PD
                | PDSS_SBU_CTRL_OUT2_1MEG_EN_PD);
    }
    else
    {
        /* Enable the requested resistor. */
        regval |= (1 << aux2_config);
    }

    /* Store the configuration. */
    gl_aux1_config = aux1_config;
    gl_aux2_config = aux2_config;

    /* Update the resistor. */
    pd->sbu_ctrl = regval;
    return;
}

aux_resistor_config_t get_aux1_resistor_config(uint8_t port)
{
    (void)port;
    return gl_aux1_config;
}

aux_resistor_config_t get_aux2_resistor_config(uint8_t port)
{
    (void)port;
    return gl_aux2_config;
}

void pd_enable_vconn_comp(void)
{
    /* Enable VCONN Comparator for VCONN monitoring. */
    PPDSS_REGS_T pd = gl_pdss[0];
    pd->vconn_ctrl |= PDSS_VCONN_CTRL_EN_COMP1;

    /* This delay is needed for VCONN comparator output to stabalize. */
    CyDelayUs (100);
}

bool pd_get_vconn_status(void)
{
    /* Return the status of VCONN. */
    PPDSS_REGS_T pd = gl_pdss[0];

    return (pd->status & PDSS_STATUS_VCONN1_STATUS);
}

void pd_disconnect_ra(void)
{
    uint32_t tmp;
    PPDSS_REGS_T pd = gl_pdss[0];
    tmp = pd->vconn_ctrl;

    /* Enable the pump, hence disconnecting RA. */
    tmp |= PDSS_VCONN_CTRL_PUMP_EN;
    /* Turn on both leakers and set to 200uA */
    tmp &= ~(PDSS_VCONN_CTRL_LEAKER_CONFIG1_MASK);
    tmp |= PDSS_VCONN_LEAKER_SEL_200_UA_VAL << PDSS_VCONN_CTRL_LEAKER_CONFIG1_POS;
    pd->vconn_ctrl = tmp;
}
#endif /* CCG3 */

#if BATTERY_CHARGING_ENABLE
ccg_status_t chgb_init(uint8_t cport, bc_phy_cbk_t cbk)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];

    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    pdss_stat->bc_phy_cbk = cbk;

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_enable(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    /* Enable Charger detect block */
    pd->chgdet_ctrl_0 = PDSS_CHGDET_CTRL_0_EN_CHGDET;
    pd->chgdet_ctrl_1 = PDSS_CHGDET_CTRL_1_CHGDET_ISO_N;
    CyDelayUs(50);

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_disable(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_1 = 0;
    pd->chgdet_ctrl_0 = PDSS_CHGDET_CTRL_0_PD;
    pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;
    pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;
    chgb_qc_src_cont_mode_stop(cport);

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_apply_src_term(uint8_t cport, chgb_src_term_t charger_term)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    /* Remove existing terms */
    chgb_remove_term(cport);

    switch(charger_term)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->chgdet_ctrl_1 |= ( 1u << PDSS_CHGDET_CTRL_1_CHGDET_APPLE_MODE_DP_POS)|
                ( 2u << PDSS_CHGDET_CTRL_1_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->chgdet_ctrl_1 |= ( 2u << PDSS_CHGDET_CTRL_1_CHGDET_APPLE_MODE_DP_POS) |
                ( 1u << PDSS_CHGDET_CTRL_1_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->chgdet_ctrl_1 |= ( 2u << PDSS_CHGDET_CTRL_1_CHGDET_APPLE_MODE_DP_POS) |
                ( 2u << PDSS_CHGDET_CTRL_1_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_QC:
        case CHGB_SRC_TERM_AFC:
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDM_PD_EN;
            break;
        case CHGB_SRC_TERM_DCP:
            pd->chgdet_ctrl_0 |= (PDSS_CHGDET_CTRL_0_DCP_EN |
                    PDSS_CHGDET_CTRL_0_RDAT_LKG_DP_EN|
                    PDSS_CHGDET_CTRL_0_RDAT_LKG_DM_EN);
            break;
        default:
            break;
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_apply_dp_pd(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDP_PD_EN;

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_remove_dp_pd(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    pd->chgdet_ctrl_0 &= ~PDSS_CHGDET_CTRL_0_RDP_PD_EN;

    return CCG_STAT_SUCCESS;
}

void chgb_apply_rdat_lkg_dp(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDAT_LKG_DP_EN;
}

void chgb_apply_rdat_lkg_dm(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDAT_LKG_DM_EN;
}

void chgb_remove_rdat_lkg_dp(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_0 &= ~PDSS_CHGDET_CTRL_0_RDAT_LKG_DP_EN;
}

void chgb_remove_rdat_lkg_dm(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_0 &= ~PDSS_CHGDET_CTRL_0_RDAT_LKG_DM_EN;
}

ccg_status_t chgb_apply_sink_term(uint8_t cport, chgb_snk_term_t charger_term)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    switch(charger_term)
    {
        case CHGB_SINK_TERM_SPD:
            /* Pull up on D+ */
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDP_PU_EN;
            break;
        case CHGB_SINK_TERM_PCD:
            /* Remove other termiantions. */
            pd->chgdet_ctrl_0 &= ~(PDSS_CHGDET_CTRL_0_VDM_SRC_EN |
                    PDSS_CHGDET_CTRL_0_IDP_SNK_EN);
            /* Connect VDP_SRC and IDM_SINK. */
            pd->chgdet_ctrl_0 |= (PDSS_CHGDET_CTRL_0_VDP_SRC_EN |
                    PDSS_CHGDET_CTRL_0_IDM_SNK_EN);
            break;
        case CHGB_SINK_TERM_SCD:
            /* Remove PCD termiantions. */
            pd->chgdet_ctrl_0 &= ~(PDSS_CHGDET_CTRL_0_VDP_SRC_EN |
                    PDSS_CHGDET_CTRL_0_IDM_SNK_EN);
            /* Connect VDM_SRC and IDP_SINK. */
            pd->chgdet_ctrl_0 |= (PDSS_CHGDET_CTRL_0_VDM_SRC_EN |
                    PDSS_CHGDET_CTRL_0_IDP_SNK_EN);
            break;
        case CHGB_SINK_TERM_QC_5V:
        case CHGB_SINK_TERM_AFC:
            /* 0.6 on D+, D- HiZ */
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_VDP_SRC_EN;
            /* Remove other terms */
            pd->chgdet_ctrl_0 &= ~ (PDSS_CHGDET_CTRL_0_RDP_PU_EN |
                    PDSS_CHGDET_CTRL_0_RDM_PU_EN |
                    PDSS_CHGDET_CTRL_0_VDM_SRC_EN |
                    PDSS_CHGDET_CTRL_0_IDM_SNK_EN);
            break;
        case CHGB_SINK_TERM_QC_9V:
            /* 3.3V on D+, 0.6 on D- */
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDP_PU_EN;
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_VDM_SRC_EN;

            /* Remove other terms */
            pd->chgdet_ctrl_0 &= ~ ( PDSS_CHGDET_CTRL_0_RDM_PU_EN |
                    PDSS_CHGDET_CTRL_0_VDP_SRC_EN);
            break;
        case CHGB_SINK_TERM_QC_12V:
            /* 0.6 on D+, 0.6 on D- */
            pd->chgdet_ctrl_0 |= (PDSS_CHGDET_CTRL_0_VDP_SRC_EN | PDSS_CHGDET_CTRL_0_VDM_SRC_EN);

            /* Remove other terms */
            pd->chgdet_ctrl_0 &= ~ (PDSS_CHGDET_CTRL_0_RDP_PU_EN |
                    PDSS_CHGDET_CTRL_0_RDM_PU_EN);
            break;
        case CHGB_SINK_TERM_QC_20V:
            /* 3.3V on D+, 3.3V on D- */
            pd->chgdet_ctrl_0 |= (PDSS_CHGDET_CTRL_0_RDP_PU_EN | PDSS_CHGDET_CTRL_0_RDM_PU_EN);

            /* Remove other terms */
            pd->chgdet_ctrl_0 &= ~ ( PDSS_CHGDET_CTRL_0_VDM_SRC_EN |
                    PDSS_CHGDET_CTRL_0_VDP_SRC_EN);
            break;
        case CHGB_SINK_TERM_QC_CONT:
            /* 0.6 on D+, 3.3 on D- */
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_VDP_SRC_EN;
            pd->chgdet_ctrl_0 |= PDSS_CHGDET_CTRL_0_RDM_PU_EN;

            /* Remove other terms */
            pd->chgdet_ctrl_0 &= ~ ( PDSS_CHGDET_CTRL_0_RDP_PU_EN |
                    PDSS_CHGDET_CTRL_0_VDM_SRC_EN);
            break;
        case CHGB_SINK_TERM_APPLE:
            chgb_remove_term(cport);
            break;
        default:
            break;
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_remove_term(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->chgdet_ctrl_0 &= ~(PDSS_CHGDET_CTRL_0_IDP_SNK_EN |
            PDSS_CHGDET_CTRL_0_IDM_SNK_EN |
            PDSS_CHGDET_CTRL_0_VDP_SRC_EN |
            PDSS_CHGDET_CTRL_0_VDM_SRC_EN |
            PDSS_CHGDET_CTRL_0_IDP_SRC_EN |
            PDSS_CHGDET_CTRL_0_DCP_EN |
            PDSS_CHGDET_CTRL_0_RDM_PD_EN |
            PDSS_CHGDET_CTRL_0_RDM_PU_EN |
            PDSS_CHGDET_CTRL_0_RDP_PD_EN |
            PDSS_CHGDET_CTRL_0_RDP_PU_EN |
            PDSS_CHGDET_CTRL_0_RDAT_LKG_DP_EN |
            PDSS_CHGDET_CTRL_0_RDAT_LKG_DM_EN );
    pd->chgdet_ctrl_1 = PDSS_CHGDET_CTRL_1_CHGDET_ISO_N;

    return CCG_STAT_SUCCESS;
}

bool chgb_set_comp(uint8_t cport, uint8_t comp_idx, chgb_comp_pinput_t p_input,
        chgb_comp_ninput_t n_input, chgb_vref_t vref, chgb_comp_edge_t edge)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    uint32_t regVal;
    uint32_t temp_chgdet_ctrl_0;
    uint32_t temp_intr3_cfg_0;
    uint32_t temp_intr3_mask;
    bool out = false;
    bool result = false;
    uint8_t intr_state;

    intr_state = CyEnterCriticalSection();
    if(comp_idx == 0)
    {
        temp_chgdet_ctrl_0 = pd->chgdet_ctrl_0;
        temp_intr3_cfg_0 = pd->intr3_cfg_0;
        temp_intr3_mask =  pd->intr3_mask;

        pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;

        regVal = pd->chgdet_ctrl_0;
        regVal &= ~(PDSS_CHGDET_CTRL_0_CMP_INN_SEL_MASK |
                PDSS_CHGDET_CTRL_0_CMP_INP_SEL_MASK |
                PDSS_CHGDET_CTRL_0_VREF_SEL_MASK);
        regVal |= ((n_input << PDSS_CHGDET_CTRL_0_CMP_INN_SEL_POS) |
                (p_input << PDSS_CHGDET_CTRL_0_CMP_INP_SEL_POS) |
                (vref << PDSS_CHGDET_CTRL_0_VREF_SEL_POS)|
                PDSS_CHGDET_CTRL_0_EN_COMP_CHGDET );


        /* Enable comparator */
        pd->chgdet_ctrl_0 = regVal;

        CyDelayUs(10);

        if (pd->ncell_status & PDSS_NCELL_STATUS_CHGDET_STATUS)
        {
            result = true;
        }

        /* Enable Interrupt and check if condition already exists */
        if(edge == CHGB_COMP_NO_INTR)
        {
            pd->chgdet_ctrl_0 = temp_chgdet_ctrl_0;
            pd->intr3_cfg_0 = temp_intr3_cfg_0;
            pd->intr3_mask = temp_intr3_mask;
            CyDelayUs(10);
        }
        else
        {
            pd->intr3_cfg_0 &= ~(PDSS_INTR3_CFG_0_CHGDET_CFG_MASK | PDSS_INTR3_CFG_0_CHGDET_FILT_EN);
            pd->intr3_cfg_0 |= (edge << PDSS_INTR3_CFG_0_CHGDET_CFG_POS);

            /* Enable comparator interrupts. */
            pd->intr3_mask |= PDSS_INTR3_CHGDET_CHANGED;

        }

        /* Clear comparator interrupt. */
        pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;

        if(pd->intr3_mask & PDSS_INTR3_CHGDET_CHANGED)
        {
            if (pd->ncell_status & PDSS_NCELL_STATUS_CHGDET_STATUS)
            {
                out = true;
            }
            regVal = (pd->intr3_cfg_0 & PDSS_INTR3_CFG_0_CHGDET_CFG_MASK) >> PDSS_INTR3_CFG_0_CHGDET_CFG_POS;


            if (((regVal ==  CHGB_COMP_EDGE_FALLING) && (out == false)) ||
                    ((regVal ==  CHGB_COMP_EDGE_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr3_set |= PDSS_INTR3_CHGDET_CHANGED;
            }
        }

    }

    CyExitCriticalSection(intr_state);

    return result;
}

ccg_status_t chgb_stop_comp(uint8_t cport, uint8_t comp_idx)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    if(comp_idx == 0)
    {
        pd->chgdet_ctrl_0 &= ~PDSS_CHGDET_CTRL_0_EN_COMP_CHGDET;
        pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;
        pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;
    }

    return CCG_STAT_SUCCESS;
}

bool chgb_get_comp_result(uint8_t cport, uint8_t comp_idx)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    if(comp_idx == 0)
    {
        if(pd->ncell_status & PDSS_NCELL_STATUS_CHGDET_STATUS)
        {
            return true;
        }
    }
    else
    {
        /*TODO: For second comparator */

    }
    return false;
}

ccg_status_t chgb_qc_src_init(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_qc_src_stop(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}

#ifdef CCG3

void chgb_dp_isr_handler(void)
{
    if (gpio_get_intr(DP_C_PORT_PIN) == true)
    {
        if(gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk != NULL)
        {
            CyDelayUs(100);
            if(gpio_read_value(DP_C_PORT_PIN) == true)
            {
                gl_pdss_status[BC_PORT_0_IDX].bc_qc_pulse_count++ ;
                gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk(BC_PORT_0_IDX, BC_EVT_QC_CONT);
            }
        }
    }
    /* Clear the interrupt. */
    gpio_clear_intr(DP_C_PORT_PIN);
}

void chgb_dm_isr_handler(void)
{
    if (gpio_get_intr(DM_C_PORT_PIN) == true)
    {
        if(gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk != NULL)
        {
            CyDelayUs(100);
            if(gpio_read_value(DM_C_PORT_PIN) == false)
            {
                gl_pdss_status[BC_PORT_0_IDX].bc_qc_pulse_count-- ;
                gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk(BC_PORT_0_IDX, BC_EVT_QC_CONT);
            }
        }
    }
    /* Clear the interrupt. */
    gpio_clear_intr(DM_C_PORT_PIN);
}
#endif /* CCG3 */

int chgb_get_qc_pulse_count(uint8_t cport)
{
    return gl_pdss_status[cport].bc_qc_pulse_count;
}

void chgb_update_qc_pulse_count(uint8_t cport, int new_count)
{
    uint8_t intr_state = CyEnterCriticalSection();
    gl_pdss_status[cport].bc_qc_pulse_count = gl_pdss_status[cport].bc_qc_pulse_count - new_count;
    CyExitCriticalSection(intr_state);
}



ccg_status_t chgb_qc_src_cont_mode_start(uint8_t cport)
{
#ifdef CCG3
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];

    if(cport == BC_PORT_0_IDX)
    {
        pdss_stat->bc_qc_pulse_count = 0;
        /* Register Dp interrupt handler. */
        CyIntDisable(GPIO_PORT2_INTR_NO);
        (void)CyIntSetVector(GPIO_PORT2_INTR_NO, &chgb_dp_isr_handler);
        CyIntEnable(GPIO_PORT2_INTR_NO);

        gpio_int_set_config( DP_C_PORT_PIN, GPIO_INTR_RISING);

        /* Register Dm interrupt handler. */
        CyIntDisable(GPIO_PORT1_INTR_NO);
        (void)CyIntSetVector(GPIO_PORT1_INTR_NO, &chgb_dm_isr_handler);
        CyIntEnable(GPIO_PORT1_INTR_NO);

        gpio_int_set_config( DM_C_PORT_PIN, GPIO_INTR_FALLING);
    }
#endif /* CCG3 */

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_qc_src_cont_mode_stop(uint8_t cport)
{
#ifdef CCG3
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    if(cport == BC_PORT_0_IDX)
    {
        gpio_int_set_config(DP_C_PORT_PIN, GPIO_INTR_DISABLE);
        gpio_int_set_config(DM_C_PORT_PIN, GPIO_INTR_DISABLE);
        pdss_stat->bc_qc_pulse_count = 0;
    }
#endif /* CCG3 */
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_qc_sink_init(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_afc_src_init(uint8_t cport)
{
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_afc_src_stop(uint8_t cport)
{
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_afc_sink_init(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}

#endif /* BATTERY_CHARGING_ENABLE */

/* End of file */

