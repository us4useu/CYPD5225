/**
 * @file hal_ccgx.c
 *
 * @brief @{PD and Type-C HAL layer for CCGx device family.@}
 */

/*
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

#include "config.h"
#include "hal_ccgx.h"
#include "ccgx_regs.h"
#include "pd.h"
#include "gpio.h"
#include "pdss_hal.h"
#include "timer.h"
#include "system.h"

#if VBUS_SCP_ENABLE
/*SCP callbacks. */
vbus_ocp_cbk_t vbus_scp_cbk[NO_OF_TYPEC_PORTS];
#endif /* VBUS_SCP_ENABLE */

#if VBUS_OCP_ENABLE

/* OCP callbacks. */
vbus_ocp_cbk_t vbus_ocp_cbk[NO_OF_TYPEC_PORTS];

#if ((VBUS_OCP_MODE != VBUS_OCP_MODE_EXT) && (defined(CCG3) || defined(CCG5)))

/*
 * Note: The high side CSA block used for Over-Current detection in CCG3 and CCG5 devices has
 * a non-linear measurement error. The error is as high as 26% at low sense voltages and reduces
 * to the order of 4% at higher sense voltages.
 *
 * The following table has been updated to prevent false Over-Current detection due to a worst
 * case positive measurement error. This means that the nominal Over-Current detection threshold
 * is set much higher than the expected value for low sense voltages.
 */
const uint8_t csa_tab[] =
/* Vsense (mv) (for 10mOhm Rsense), av1 (:3)-bw (:2), vref_sel code (6) */
{                  /* Nominal gain, Vref */
    /* 10, */  0x1F,  5,        /* 100, 1.35  */
    /* 11, */  0x1F, 19,        /* 100, 1.49  */
    /* 12, */  0x1F, 32,        /* 100, 1.62  */
    /* 13, */  0x1F, 46,        /* 100, 1.76  */
    /* 14, */  0x1F, 59,        /* 100, 1.89  */
    /* 15, */  0x1B, 12,        /* 70,  1.42  */
    /* 16, */  0x1B, 21,        /* 70,  1.51  */
    /* 17, */  0x1B, 31,        /* 70,  1.61  */
    /* 18, */  0x1B, 40,        /* 70,  1.70  */
    /* 19, */  0x1B, 50,        /* 70,  1.80  */
    /* 20, */  0x1B, 39,        /* 70,  1.69  */
    /* 21, */  0x1B, 47,        /* 70,  1.77  */
    /* 22, */  0x1B, 55,        /* 70,  1.85  */
    /* 23, */  0x1B, 63,        /* 70,  1.93  */
    /* 24, */  0x16, 15,        /* 50,  1.45  */
    /* 25, */  0x16, 15,        /* 50,  1.45  */
    /* 26, */  0x16, 21,        /* 50,  1.51  */
    /* 27, */  0x16, 27,        /* 50,  1.57  */
    /* 28, */  0x16, 33,        /* 50,  1.63  */
    /* 29, */  0x16, 39,        /* 50,  1.69  */
    /* 30, */  0x16, 44,        /* 50,  1.74  */
    /* 31, */  0x16, 50,        /* 50,  1.80  */
    /* 32, */  0x16, 56,        /* 50,  1.86  */
    /* 33, */  0x16, 62,        /* 50,  1.92  */
    /* 34, */  0x12, 28,        /* 40,  1.58  */
    /* 35, */  0x12, 33,        /* 40,  1.63  */
    /* 36, */  0x12, 30,        /* 40,  1.60  */
    /* 37, */  0x12, 34,        /* 40,  1.64  */
    /* 38, */  0x12, 39,        /* 40,  1.69  */
    /* 39, */  0x12, 43,        /* 40,  1.73  */
    /* 40, */  0x12, 48,        /* 40,  1.78  */
    /* 41, */  0x12, 52,        /* 40,  1.82  */
    /* 42, */  0x12, 57,        /* 40,  1.87  */
    /* 43, */  0x12, 61,        /* 40,  1.91  */
    /* 44, */  0x0D, 17,        /* 30,  1.47  */
    /* 45, */  0x0D, 20,        /* 30,  1.50  */
    /* 46, */  0x0D, 23,        /* 30,  1.53  */
    /* 47, */  0x0D, 27,        /* 30,  1.57  */
    /* 48, */  0x0D, 30,        /* 30,  1.60  */
    /* 49, */  0x0D, 33,        /* 30,  1.63  */
    /* 50, */  0x0D, 37,        /* 30,  1.67  */
    /* 51, */  0x0D, 40,        /* 30,  1.70  */
    /* 52, */  0x0D, 43,        /* 30,  1.73  */
    /* 53, */  0x0D, 47,        /* 30,  1.77  */
    /* 54, */  0x0D, 44,        /* 30,  1.74  */
    /* 55, */  0x0D, 47,        /* 30,  1.77  */
    /* 56, */  0x0D, 51,        /* 30,  1.81  */
    /* 57, */  0x0D, 54,        /* 30,  1.84  */
    /* 58, */  0x0D, 57,        /* 30,  1.87  */
    /* 59, */  0x0D, 60,        /* 30,  1.90  */
    /* 60, */  0x0D, 63,        /* 30,  1.93  */
    /* 61, */  0x09,  1,        /* 20,  1.31  */
    /* 62, */  0x09,  3,        /* 20,  1.33  */
    /* 63, */  0x09,  5,        /* 20,  1.35  */
    /* 64, */  0x09,  8,        /* 20,  1.38  */
    /* 65, */  0x09, 10,        /* 20,  1.40  */
    /* 66, */  0x09, 12,        /* 20,  1.42  */
    /* 67, */  0x09, 14,        /* 20,  1.44  */
    /* 68, */  0x09, 16,        /* 20,  1.46  */
    /* 69, */  0x09, 18,        /* 20,  1.48  */
    /* 70, */  0x09, 21,        /* 20,  1.51  */
    /* 71, */  0x09, 23,        /* 20,  1.53  */
    /* 72, */  0x09, 25         /* 20,  1.55  */

/*
 * For an Rsense of 10 mOhm this is enough.
 * The rest of the table follows.
 */
#if 0
    /* 73,  */ 0x09, 27,        /* 20,  1.57  */
    /* 74,  */ 0x09, 29,        /* 20,  1.59  */
    /* 75,  */ 0x09, 31,        /* 20,  1.61  */
    /* 76,  */ 0x09, 33,        /* 20,  1.63  */
    /* 77,  */ 0x09, 36,        /* 20,  1.66  */
    /* 78,  */ 0x09, 38,        /* 20,  1.68  */
    /* 79,  */ 0x09, 40,        /* 20,  1.70  */
    /* 80,  */ 0x09, 42,        /* 20,  1.72  */
    /* 81,  */ 0x09, 44,        /* 20,  1.74  */
    /* 82,  */ 0x09, 46,        /* 20,  1.76  */
    /* 83,  */ 0x09, 48,        /* 20,  1.78  */
    /* 84,  */ 0x09, 51,        /* 20,  1.81  */
    /* 85,  */ 0x09, 53,        /* 20,  1.83  */
    /* 86,  */ 0x09, 55,        /* 20,  1.85  */
    /* 87,  */ 0x09, 57,        /* 20,  1.87  */
    /* 88,  */ 0x09, 59,        /* 20,  1.89  */
    /* 89,  */ 0x09, 61,        /* 20,  1.91  */
    /* 90,  */ 0x04, 15,        /* 15,  1.45  */
    /* 91,  */ 0x04, 17,        /* 15,  1.47  */
    /* 92,  */ 0x04, 18,        /* 15,  1.48  */
    /* 93,  */ 0x04, 20,        /* 15,  1.50  */
    /* 94,  */ 0x04, 22,        /* 15,  1.52  */
    /* 95,  */ 0x04, 23,        /* 15,  1.53  */
    /* 96,  */ 0x04, 25,        /* 15,  1.55  */
    /* 97,  */ 0x04, 26,        /* 15,  1.56  */
    /* 98,  */ 0x04, 28,        /* 15,  1.58  */
    /* 99,  */ 0x04, 30,        /* 15,  1.60  */
    /* 100, */ 0x04, 31,        /* 15,  1.61  */
    /* 101, */ 0x04, 33,        /* 15,  1.63  */
    /* 102, */ 0x04, 35,        /* 15,  1.65  */
    /* 103, */ 0x04, 36,        /* 15,  1.66  */
    /* 104, */ 0x04, 38,        /* 15,  1.68  */
    /* 105, */ 0x04, 39,        /* 15,  1.69  */
    /* 106, */ 0x04, 41,        /* 15,  1.71  */
    /* 107, */ 0x04, 43,        /* 15,  1.73  */
    /* 108, */ 0x04, 44,        /* 15,  1.74  */
    /* 109, */ 0x04, 46,        /* 15,  1.76  */
    /* 110, */ 0x04, 47,        /* 15,  1.77  */
    /* 111, */ 0x04, 49,        /* 15,  1.79  */
    /* 112, */ 0x04, 51,        /* 15,  1.81  */
    /* 113, */ 0x04, 52,        /* 15,  1.82  */
    /* 114, */ 0x04, 54,        /* 15,  1.84  */
    /* 115, */ 0x04, 55,        /* 15,  1.85  */
    /* 116, */ 0x04, 57,        /* 15,  1.87  */
    /* 117, */ 0x04, 59,        /* 15,  1.89  */
    /* 118, */ 0x04, 60,        /* 15,  1.90  */
    /* 119, */ 0x04, 62,        /* 15,  1.92  */
    /* 120, */ 0x04, 63,        /* 15,  1.93  */
    /* 121, */ 0x00,  0,        /* 10,  1.30  */
    /* 122, */ 0x00,  1,        /* 10,  1.31  */
    /* 123, */ 0x00,  2,        /* 10,  1.32  */
    /* 124, */ 0x00,  3,        /* 10,  1.33  */
    /* 125, */ 0x00,  4,        /* 10,  1.34  */
    /* 126, */ 0x00,  5,        /* 10,  1.35  */
    /* 127, */ 0x00,  7,        /* 10,  1.37  */
    /* 128, */ 0x00,  8,        /* 10,  1.38  */
    /* 129, */ 0x00,  9,        /* 10,  1.39  */
    /* 130, */ 0x00, 10,        /* 10,  1.40  */
    /* 131, */ 0x00, 11,        /* 10,  1.41  */
    /* 132, */ 0x00, 12,        /* 10,  1.42  */
    /* 133, */ 0x00, 13,        /* 10,  1.43  */
    /* 134, */ 0x00, 14,        /* 10,  1.44  */
    /* 135, */ 0x00, 15,        /* 10,  1.45  */
    /* 136, */ 0x00, 16,        /* 10,  1.46  */
    /* 137, */ 0x00, 17,        /* 10,  1.47  */
    /* 138, */ 0x00, 18,        /* 10,  1.48  */
    /* 139, */ 0x00, 19,        /* 10,  1.49  */
    /* 140, */ 0x00, 21,        /* 10,  1.51  */
    /* 141, */ 0x00, 22,        /* 10,  1.52  */
    /* 142, */ 0x00, 23,        /* 10,  1.53  */
    /* 143, */ 0x00, 24,        /* 10,  1.54  */
    /* 144, */ 0x00, 25,        /* 10,  1.55  */
    /* 145, */ 0x00, 26,        /* 10,  1.56  */
    /* 146, */ 0x00, 27,        /* 10,  1.57  */
    /* 147, */ 0x00, 28,        /* 10,  1.58  */
    /* 148, */ 0x00, 29,        /* 10,  1.59  */
    /* 149, */ 0x00, 30,        /* 10,  1.60  */
    /* 150, */ 0x00, 31,        /* 10,  1.61  */
    /* 151, */ 0x00, 32,        /* 10,  1.62  */
    /* 152, */ 0x00, 33,        /* 10,  1.63  */
    /* 153, */ 0x00, 35,        /* 10,  1.65  */
    /* 154, */ 0x00, 36,        /* 10,  1.66  */
    /* 155, */ 0x00, 37,        /* 10,  1.67  */
    /* 156, */ 0x00, 38,        /* 10,  1.68  */
    /* 157, */ 0x00, 39,        /* 10,  1.69  */
    /* 158, */ 0x00, 40,        /* 10,  1.70  */
    /* 159, */ 0x00, 41,        /* 10,  1.71  */
    /* 160, */ 0x00, 42,        /* 10,  1.72  */
    /* 161, */ 0x00, 43,        /* 10,  1.73  */
    /* 162, */ 0x00, 44,        /* 10,  1.74  */
    /* 163, */ 0x00, 45,        /* 10,  1.75  */
    /* 164, */ 0x00, 46,        /* 10,  1.76  */
    /* 165, */ 0x00, 47,        /* 10,  1.77  */
    /* 166, */ 0x00, 48,        /* 10,  1.78  */
    /* 167, */ 0x00, 50,        /* 10,  1.80  */
    /* 168, */ 0x00, 51,        /* 10,  1.81  */
    /* 169, */ 0x00, 52,        /* 10,  1.82  */
    /* 170, */ 0x00, 53,        /* 10,  1.83  */
    /* 171, */ 0x00, 54,        /* 10,  1.84  */
    /* 172, */ 0x00, 55,        /* 10,  1.85  */
    /* 173, */ 0x00, 56,        /* 10,  1.86  */
    /* 174, */ 0x00, 57,        /* 10,  1.87  */
    /* 175, */ 0x00, 58,        /* 10,  1.88  */
    /* 176, */ 0x00, 59,        /* 10,  1.89  */
    /* 177, */ 0x00, 60,        /* 10,  1.90  */
    /* 178, */ 0x00, 61,        /* 10,  1.91  */
    /* 179, */ 0x00, 62,        /* 10,  1.92  */
    /* 180, */ 0x00, 63         /* 10,  1.93  */
#endif
};
#endif /* ((VBUS_OCP_MODE != VBUS_OCP_MODE_EXT) && (defined(CCG3) || defined(CCG5))) */

#if (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT)

static const gpio_port_pin_t hal_ocp_fault_pin[] = {
    APP_VBUS_OCP_PORT_PIN_P1
#if CCG_PD_DUALPORT_ENABLE
    ,
    APP_VBUS_OCP_PORT_PIN_P2
#endif
};

static uint8_t hal_ocp_debounce_ms[NO_OF_TYPEC_PORTS];

static void ocp_handler_wrapper(uint8_t port, timer_id_t id)
{
    vbus_ocp_handler (port);
}

CY_ISR(vbus_ocp_port0_handler)
{
    /* Clear the interrupt. */
    gpio_clear_intr(hal_ocp_fault_pin[TYPEC_PORT_0_IDX]);

    if (hal_ocp_debounce_ms[TYPEC_PORT_0_IDX] == 0)
    {
        vbus_ocp_handler(TYPEC_PORT_0_IDX);
    }
    else
    {
        /* Stop looking for interrupts again and start the debounce timer. */
        gpio_int_set_config (hal_ocp_fault_pin[TYPEC_PORT_0_IDX], GPIO_INTR_DISABLE);
        timer_start(TYPEC_PORT_0_IDX, PD_OCP_DEBOUNCE_TIMER,
                hal_ocp_debounce_ms[TYPEC_PORT_0_IDX], ocp_handler_wrapper);
    }
}

#if CCG_PD_DUALPORT_ENABLE

CY_ISR(vbus_ocp_port1_handler)
{
    /* Clear the interrupt. */
    gpio_clear_intr(hal_ocp_fault_pin[TYPEC_PORT_1_IDX]);

    if (hal_ocp_debounce_ms[TYPEC_PORT_1_IDX] == 0)
    {
        vbus_ocp_handler(TYPEC_PORT_1_IDX);
    }
    else
    {
        /* Stop looking for interrupts again and start the debounce timer. */
        gpio_int_set_config (hal_ocp_fault_pin[TYPEC_PORT_1_IDX], GPIO_INTR_DISABLE);
        timer_start(TYPEC_PORT_1_IDX, PD_OCP_DEBOUNCE_TIMER,
                hal_ocp_debounce_ms[TYPEC_PORT_1_IDX], ocp_handler_wrapper);
    }
}

#endif /* CCG_PD_DUALPORT_ENABLE */

#endif /* (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT) */

#endif /* VBUS_OCP_ENABLE */

/** Variable used to identify current CCG silicon revision. */
uint8_t ccg_si_revision = 0;

/** Function that configures CCG device GPIOs that are used for:
    1. VBus monitoring in cases where it is done through external resistor divider.
    2. OCP fault indication GPIO where they are used.
    3. OVP trip GPIO where used.
 */
static void ccg_pwr_gpio_init(void)
{
#if (VBUS_MON_INTERNAL == 0)
    /* Connect port 0 VBUS_MON to corresponding AMUX. */
    hsiom_set_config(APP_VBUS_MON_PORT_PIN_P1, APP_VBUS_MON_AMUX_INPUT_P1);
#endif /* VBUS_MON_INTERNAL */

#if ((VBUS_OCP_ENABLE) && (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT))
    uint8_t intr_no;

    /* OCP_FAULT GPIO for PD Port-0 */
    intr_no = GPIO_PORT0_INTR_NO + APP_VBUS_OCP_FAULT_PORT_NO_P1;
    /* Register sync interrupt handler. */
    CyIntDisable(intr_no);
    (void)CyIntSetVector(intr_no, &vbus_ocp_port0_handler);
    CyIntSetPriority(intr_no, 0);
    CyIntEnable(intr_no);
#endif /* ((VBUS_OCP_ENABLE) && (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT)) */

#ifdef CCG4
    /* Route OVP comparator output to OVP trip GPIO. */
    system_connect_ovp_trip(TYPEC_PORT_0_IDX);
#endif /* CCG4 */

#if CCG_PD_DUALPORT_ENABLE

#if (VBUS_MON_INTERNAL == 0)
    /* Connect port 1 VBUS_MON to corresponding AMUX. */
    hsiom_set_config(APP_VBUS_MON_PORT_PIN_P2, APP_VBUS_MON_AMUX_INPUT_P2);
#endif /* VBUS_MON_INTERNAL */

#if ((VBUS_OCP_ENABLE) && (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT))
    /* OCP_FAULT GPIO for PD Port-1 */
    intr_no = GPIO_PORT0_INTR_NO + APP_VBUS_OCP_FAULT_PORT_NO_P2;
    /* Register sync interrupt handler. */
    CyIntDisable(intr_no);
    (void)CyIntSetVector(intr_no, &vbus_ocp_port1_handler);
    CyIntSetPriority(intr_no, 0);
    CyIntEnable(intr_no);
#endif /* ((VBUS_OCP_ENABLE) && (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT)) */

#ifdef CCG4
    /* Route OVP comparator output to OVP trip GPIO. */
    system_connect_ovp_trip(TYPEC_PORT_1_IDX);
#endif /* CCG4 */

#endif /* CCG_PD_DUALPORT_ENABLE */
}

void system_init(void)
{
    /* Configure clocks for the PDSS IP block. */
#if (NO_OF_TYPEC_PORTS == 1)
    PERI->pclk_ctl[PDSS_PORT0_PCLK_RX_IDX]  = PDSS_PORT0_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_TX_IDX]  = PDSS_PORT0_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* CCG3PA specific clocks. */
    PERI->pclk_ctl[PDSS_PORT0_PCLK_REFGEN_IDX] = PDSS_PORTX_REFGEN_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_BCH_IDX]    = PDSS_PORT0_BCH_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_FILT1_IDX]  = PDSS_PORT0_FILT_CLK_SEL_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_ISINK_IDX]  = PDSS_PORT0_ISINK_CLK_DIV_ID;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined(CCG5))
    /* CCG5 specific clocks. */
    PERI->pclk_ctl[PDSS_PORT0_PCLK_REFGEN_IDX] = PDSS_PORTX_REFGEN_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_FILT1_IDX]  = PDSS_PORT0_FILT1_CLK_DIV_ID;
#endif /* (defined(CCG5)) */

#if ((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)))
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SWAP_IDX] = PDSS_PORT0_SWAP_CLK_DIV_ID;
#endif /*((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)) */

#elif (NO_OF_TYPEC_PORTS == 2)
    PERI->pclk_ctl[PDSS_PORT0_PCLK_RX_IDX]  = PDSS_PORT0_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_TX_IDX]  = PDSS_PORT0_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;

#ifdef CCG5
    /* CCG5 specific clocks. */
    PERI->pclk_ctl[PDSS_PORT0_PCLK_REFGEN_IDX] = PDSS_PORTX_REFGEN_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_FILT1_IDX]  = PDSS_PORT0_FILT1_CLK_DIV_ID;

    /* On CCG5, we use the same clocks for both PD ports. */
    PERI->pclk_ctl[PDSS_PORT1_PCLK_RX_IDX]    = PDSS_PORT0_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_TX_IDX]    = PDSS_PORT0_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_SAR_IDX]   = PDSS_PORT0_SAR_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_FILT1_IDX] = PDSS_PORT0_FILT1_CLK_DIV_ID;

#else /* !CCG5 */
    /* Using separate clocks for Ports 0 and 1 on CCG4. */
    PERI->pclk_ctl[PDSS_PORT1_PCLK_RX_IDX]  = PDSS_PORT1_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_TX_IDX]  = PDSS_PORT1_TX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT1_PCLK_SAR_IDX] = PDSS_PORT1_SAR_CLK_DIV_ID;
#endif /* CCG5 */

#if ((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)))
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SWAP_IDX] = PDSS_PORT0_SWAP_CLK_DIV_ID;

#ifdef CCG5
    PERI->pclk_ctl[PDSS_PORT1_PCLK_SWAP_IDX] = PDSS_PORT0_SWAP_CLK_DIV_ID;
#else /* !CCG5 */
    PERI->pclk_ctl[PDSS_PORT1_PCLK_SWAP_IDX] = PDSS_PORT1_SWAP_CLK_DIV_ID;
#endif /* CCG5 */

#endif /*((CCG_PD_REV3_ENABLE) && ((CCG_FRS_RX_ENABLE) || (CCG_FRS_TX_ENABLE)) */

#endif /* NO_OF_TYPEC_PORTS */

    /* Update configuration of various GPIOs used in power management. */
    ccg_pwr_gpio_init();

    /* Store the CCG silicon revision. */
    ccg_si_revision = get_silicon_revision() - CCG_SILICON_REV00_VALUE;
#ifdef CCG5
    if (ccg_si_revision > 0)
    {
        /* For Rev *A and later silicon, we need to enable crude OVP detection on CC lines at start-up. */
        PDSS0->vconn20_ctrl |= (CCG5_CC_OVP_DET_ENABLE_VAL << PDSS_VCONN20_CTRL_T_VCONN_POS);
#if CCG_PD_DUALPORT_ENABLE
        PDSS1->vconn20_ctrl |= (CCG5_CC_OVP_DET_ENABLE_VAL << PDSS_VCONN20_CTRL_T_VCONN_POS);
#endif /* CCG_PD_DUALPORT_ENABLE */
    }
#endif /* CCG5 */
}

uint8_t ccg_get_si_revision(void)
{
    return (ccg_si_revision);
}

#if VBUS_OCP_ENABLE

#if (VBUS_OCP_MODE == VBUS_OCP_MODE_POLLING)
extern void sln_set_ocp_params(uint8_t port, bool enable, uint32_t max_cur, uint8_t debounce);
#endif /* (VBUS_OCP_MODE == VBUS_OCP_MODE_POLLING) */

uint8_t system_vbus_ocp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk)
{
    if (cbk == NULL)
    {
        return false;
    }
    vbus_ocp_cbk[port] = cbk;

    /* Enable VBUS OCP protection. */
#if (VBUS_OCP_MODE == VBUS_OCP_MODE_POLLING)
    cur = cur + ((cur * pd_get_ptr_ocp_tbl(port)->threshold) / 100);
    sln_set_ocp_params (port, true, cur, pd_get_ptr_ocp_tbl(port)->debounce);
#elif (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT)
    hal_ocp_debounce_ms[port] = pd_get_ptr_ocp_tbl(port)->debounce;

  #if (VBUS_OCP_FAULT_GPIO_POLARITY == VBUS_OCP_GPIO_ACTIVE_HIGH)
    gpio_int_set_config (hal_ocp_fault_pin[port], GPIO_INTR_RISING);
  #else /* VBUS_OCP_FAULT_GPIO_POLARITY */
    gpio_int_set_config (hal_ocp_fault_pin[port], GPIO_INTR_FALLING);
  #endif /* (VBUS_OCP_FAULT_GPIO_POLARITY == VBUS_OCP_GPIO_ACTIVE_HIGH) */
#else /* (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT) */

  #if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5))
    uint32_t vsense;
    uint8_t vref_sel;
    uint8_t gain_sel;

    /* Apply the OCP threshold which is mentioned in percentage of rated current. */
    vsense = (cur + (cur * pd_get_ptr_ocp_tbl(port)->threshold) / 100);

    #if (defined(CCG3) || defined(CCG5))
    {
        unsigned int i = 0;

        /* Convert to mV, accounting for the unit of current which is 10mA. */
        vsense = (vsense * pd_get_ptr_ocp_tbl(port)->sense_res) / 100;
        vsense = GET_MIN (vsense, VSENSE_MAX);
        if (vsense < VSENSE_MIN)
        {
            i = 0;
            vsense = VSENSE_MIN;
        }
        else
        {
            i = ((vsense - VSENSE_MIN) << 1);
        }

        gain_sel = csa_tab[i];
        vref_sel = csa_tab[i + 1];
    }
    #else /* (defined(CCG3PA) || defined(CCG3PA2)) */
    {
        /* vsense contains the current setting required in 10mA units. */
        pd_lscsa_calc_cfg(vsense, &gain_sel, &vref_sel);
    }
    #endif /* CCGx */

    pd_internal_vbus_ocp_en(port, gain_sel, vref_sel, CCG_SRC_FET, VBUS_OCP_MODE,
            pd_get_ptr_ocp_tbl(port)->debounce);
  #endif /* (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5)) */
#endif /* (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT) */

    return true;
}

uint8_t system_vbus_ocp_dis(uint8_t port)
{
#if (VBUS_OCP_MODE == VBUS_OCP_MODE_POLLING)
    sln_set_ocp_params(port, false, 0, 0);
#elif (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT)
    hal_ocp_debounce_ms[port] = 0;
    gpio_int_set_config(hal_ocp_fault_pin[port], GPIO_INTR_DISABLE);
#else /* VBUS_OCP_MODE */
  #if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5))
    pd_internal_vbus_ocp_dis(port, CCG_SRC_FET);
  #endif /* (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5)) */
#endif /* (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT) */

    vbus_ocp_cbk[port] = NULL;
    return true;
}

#endif /* VBUS_OCP_ENABLE */

void vbus_ocp_handler(uint8_t port)
{
#if VBUS_OCP_ENABLE

#if (VBUS_OCP_MODE == VBUS_OCP_MODE_POLLING)
    if (vbus_ocp_cbk[port] != NULL)
    {
        vbus_ocp_cbk[port](port);
    }
#elif (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT)
    gpio_port_pin_t pin = hal_ocp_fault_pin[port];

    /* Check if the pin is still indicating fault. */
  #if (VBUS_OCP_FAULT_GPIO_POLARITY == VBUS_OCP_GPIO_ACTIVE_HIGH)
    if (gpio_read_value(pin) != false)
    {
        if (vbus_ocp_cbk[port] != NULL)
        {
            vbus_ocp_cbk[port](port);
        }
    }
    else
    {
        /* Re-enable the interrupt on the pin. */
        gpio_int_set_config(pin, GPIO_INTR_RISING);
    }
  #else /* (VBUS_OCP_FAULT_GPIO_POLARITY == VBUS_OCP_GPIO_ACTIVE_HIGH) */
    if (gpio_read_value(pin) == false)
    {
        if (vbus_ocp_cbk[port] != NULL)
        {
            vbus_ocp_cbk[port](port);
        }
    }
    else
    {
        /* Re-enable the interrupt on the pin. */
        gpio_int_set_config(pin, GPIO_INTR_FALLING);
    }
  #endif /* (VBUS_OCP_FAULT_GPIO_POLARITY == VBUS_OCP_GPIO_ACTIVE_HIGH) */
#else /* VBUS_OCP_MODE */

  #if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5))
    if (vbus_ocp_cbk[port] != NULL)
    {
        vbus_ocp_cbk[port](port);
    }
  #endif /* (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5)) */

#endif /* (VBUS_OCP_MODE == VBUS_OCP_MODE_EXT) */
#endif /* VBUS_OCP_ENABLE */
}

#if VBUS_SCP_ENABLE
uint8_t system_vbus_scp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk)
{
    uint32_t vsense;
    uint8_t debounce;

    debounce = pd_get_ptr_scp_tbl(port)->debounce;
    /* Assumming 500KHz clock */
    debounce = (debounce > 0x40) ? 0x20 : ((debounce+1) >> 1);

    if (cbk == NULL)
    {
        return false;
    }

    vbus_scp_cbk[port] = cbk;
    
    /*
     * SCP Current threshold shall be a fixed value instead of a function of
     * negotiated current. Setting it to 7.5 A for now.
     */
    cur = 1000; // 150% of 10A is 15A

    /* Get the current threshold for SCP. */
    vsense = (cur + (cur * pd_get_ptr_scp_tbl(port)->threshold) / 100);

    /* Convert into mV. */
    vsense = (vsense * VBUS_CSA_RSENSE) / 1000;

    pd_internal_vbus_scp_en (port, vsense, debounce, CCG_SRC_FET, VBUS_SCP_MODE);
    return true;
}

uint8_t system_vbus_scp_dis(uint8_t port)
{
    pd_internal_vbus_scp_dis (port, CCG_SRC_FET);

    vbus_scp_cbk[port] = NULL;
    return true;
}

void vbus_scp_handler(uint8_t port)
{
    if (vbus_scp_cbk[port] != NULL)
    {
        vbus_scp_cbk[port](port);
    }
}
#endif /* VBUS_SCP_ENABLE */

void system_disconnect_ovp_trip(uint8_t port)
{
#ifdef CCG4
#if ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0))
    uint8_t intr_state = CyEnterCriticalSection();
    /*
     * The VBUS OVP trip pin for port1 on the CCG4 EVK can only connect to
     * comparator 0. Configure the HSIOM appropriately. Also certain revision
     * of CCG4 EVK has OVP trip circuit polarity reversed and ovp trip won't work
     * on these boards.
     */
    if(port == TYPEC_PORT_0_IDX)
    {
        /* Drive GPIO, Actual comparator HW output is opposite of  pd_adc_get_comparator_status() return value*/
        gpio_set_value (APP_VBUS_OVP_TRIP_PORT_PIN_P1, (pd_adc_get_comparator_status (port, APP_OVP_ADC_ID) ^ 0x1));

        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P1, HSIOM_MODE_GPIO);

    }

#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        /* Drive GPIO, Actual comparator HW output is opposite of  pd_adc_get_comparator_status() return value*/
        gpio_set_value (APP_VBUS_OVP_TRIP_PORT_PIN_P2, (pd_adc_get_comparator_status (port, APP_OVP_ADC_ID) ^ 0x1));

        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P2, HSIOM_MODE_GPIO);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    CyExitCriticalSection(intr_state);
#endif /* ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0)) */
#endif /* CCG4*/

}

void system_connect_ovp_trip(uint8_t port)
{
#ifdef CCG4
#if ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0))
    uint8_t intr_state = CyEnterCriticalSection();
    /*
     * The VBUS OVP trip pin for port1 on the CCG4 EVK can only connect to
     * comparator 0. Configure the HSIOM appropriately. Also certain revision
     * of CCG4 EVK has OVP trip circuit polarity reversed and ovp trip won't work
     * on these boards.
     */
    if(port == TYPEC_PORT_0_IDX)
    {
        /* Set Drive Mode of GPIO. */
        gpio_set_drv_mode (APP_VBUS_OVP_TRIP_PORT_PIN_P1, GPIO_DM_STRONG);
        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P1, APP_VBUS_OVP_TRIP_HSIOM_P1);

    }

#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        /* Set Drive Mode of GPIO. */
        gpio_set_drv_mode (APP_VBUS_OVP_TRIP_PORT_PIN_P2, GPIO_DM_STRONG);
        /* Set HSIOM Configuration. */
        hsiom_set_config (APP_VBUS_OVP_TRIP_PORT_PIN_P2, APP_VBUS_OVP_TRIP_HSIOM_P2);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    CyExitCriticalSection(intr_state);
#endif /* ((VBUS_OVP_ENABLE != 0) && (VBUS_OVP_TRIP_ENABLE != 0)) */
#endif /* CCG4*/
}

/* End of file */
