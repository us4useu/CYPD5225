/**
 * @file config.h
 *
 * @brief @{Header file that enables/disables various CCG firmware features.
 *
 * This file also provides mapping to the implementation for hardware dependent
 * functions like FET control, voltage selection etc.
 *
 * This current implementation matches the CCG5 EVK from Cypress. This can be
 * updated by users to match their hardware implementation.@}
 *
 *******************************************************************************
 *
 * Copyright (2016-2018), Cypress Semiconductor Corporation or a subsidiary of
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
#ifndef _CONFIG_H_
#define _CONFIG_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>
#include <stack_params.h>
#include <NCP81239_regs.h>	

/*******************************************************************************
 * Application and boot priority configuration.
 ******************************************************************************/

/*
 * Disable Pseudo-metadata handling in flashing sequence.
 * This definition should be left enabled for CCG5 solutions.
 */
#define CCG_PSEUDO_METADATA_DISABLE             (1u)

/* Always prioritize booting of the primary (FW2) binary. */
#define CCG_PRIORITIZE_FW2                      (1u)

/* This is firmware with bootloader linkage. */
#define CCG_FIRMWARE_APP_ONLY                   (0u)

/*******************************************************************************
 * HPI Module Configuration
 ******************************************************************************/

/*
 * Index of SCB used for HPI interface. This should be set based on
 * the pin selection in the project schematic.
 */
#define HPI_SCB_INDEX                               (0u)

/* HPI interface enable. */
#define CCG_HPI_ENABLE                              (1u)

/* Enable PD support in HPI. */
#define CCG_HPI_PD_ENABLE                           (1u)

/* Enable PD command support in HPI library. */
#define CCG_HPI_PD_CMD_DISABLE                      (0u)

/* Enable VDM response query through HPI. */
#define HPI_VDM_QUERY_SUPPORTED                     (1u)

/*******************************************************************************
 * PD Stack Configuration
 ******************************************************************************/

/* PD Revision 3.0 support is enabled. */
#define CCG_PD_REV3_ENABLE                          (1u)

/* Enable Fast Role Swap from Sink to Source. */
#define CCG_FRS_RX_ENABLE                           (1u)

/* Disable Fast Role Swap from Source to Sink. */
#define CCG_FRS_TX_ENABLE                           (0u)

/*******************************************************************************
 * App level Configuration
 ******************************************************************************/

/* Consumer FET controls. */
#define APP_VBUS_SNK_FET_OFF_P1()                   pd_internal_cfet_off(0, 0)
#define APP_VBUS_SNK_FET_ON_P1()                    pd_internal_cfet_on(0, 0)

/* Provider FET controls. */
#define APP_VBUS_SRC_FET_OFF_P1()                   pd_internal_pfet_off(0, 0)
#define APP_VBUS_SRC_FET_ON_P1()                    pd_internal_pfet_on(0, 0)

/* Discharge path controls. */
#define APP_DISCHARGE_FET_OFF_P1()                  pd_internal_vbus_discharge_off(0)
#define APP_DISCHARGE_FET_ON_P1()                   pd_internal_vbus_discharge_on(0)

/* Source voltage selection. Nothing to do on PSVP. */
#define APP_VBUS_SET_5V_P1()                        __asm("NOP\n")
#define APP_VBUS_SET_9V_P1()                        __asm("NOP\n")
#define APP_VBUS_SET_12V_P1()                       __asm("NOP\n")
#define APP_VBUS_SET_13V_P1()                       __asm("NOP\n")
#define APP_VBUS_SET_15V_P1()                       __asm("NOP\n")
#define APP_VBUS_SET_19V_P1()                       __asm("NOP\n")
#define APP_VBUS_SET_20V_P1()                       __asm("NOP\n")

/* Function/Macro to set P1 source voltage to contract value. */
#define APP_VBUS_SET_VOLT_P1(mV)                    \
{                                                   \
    set_pd_ctrl_voltage(TYPEC_PORT_0_IDX, mV);      \
}

/*******************************************************************************
 * Alternate mode configuration
 ******************************************************************************/

/* Firmware shall save only SVIDs supported by the application. */
#define SAVE_SUPP_SVID_ONLY                         (1u)

/* Enable saving of discovery VDMs received during alternate mode discovery. */
#define VDM_RESP_QUERY_SUPPORTED                    (1u)

/* Enable alternate mode support when device is DFP. */
#define DFP_ALT_MODE_SUPP                           (1u)

/* Enable DisplayPort source support. */
#define DP_DFP_SUPP                                 (1u)

/* Enable Thunderbolt support as DFP. */
#define TBT_DFP_SUPP                                (1u)

/* Enable alternate mode support when device is UFP. */
#define UFP_ALT_MODE_SUPP                           (1u)

/* Enable Thunderbolt support as UFP. */
#define TBT_UFP_SUPP                                (1u)

/*******************************************************************************
 * Feature selection and configuration.
 ******************************************************************************/

#define CCG_PROG_SOURCE_ENABLE                      (1u)
#define PROG_SOURCE_MIN_VOLT                        (VSAFE_0V)

#define NCP81239_EN_P1()                             NCP81239_EN_P1_Write(1)
#define NCP81239_DIS_P1()                            NCP81239_EN_P1_Write(0)

/* BC 1.2 Source support is enabled. */
#define BC_1_2_SRC_ENABLE                           (1u)

/* Disable CDP/DCP support on PD contract completion. */
#define CCG_BC_12_IN_PD_ENABLE                      (0u)

/* Interval (in seconds) for which CCG5 waits for a CDP handshake. Set to 0 for indefinite wait. */
#define CCG5_CDP_WAIT_DURATION                      (0u)

/* Enable deep sleep for power saving. */
#define SYS_DEEPSLEEP_ENABLE                        (1u)

/* Enable hardware based DRP toggle for additional power saving. */
#define CCG_HW_DRP_TOGGLE_ENABLE                    (1u)

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                           (1u)

/*
 * Select CCG5 GPIO to be used as Activity Indication. This should be set to a
 * valid value if APP_FW_LED_ENABLE is non-zero.
 */
#define FW_LED_GPIO_PORT_PIN                        (GPIO_PORT_1_PIN_6)

/*
 * Timer ID allocation for various solution soft timers.
 */

/*
 * Activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                                (0xC0)

/*
 * The LED toggle period.
 */
#define LED_TIMER_PERIOD                            (1000)

/* Timer used to ensure I2C transfers to the MUX complete on time. */
#define MUX_I2C_TIMER                               (0xC1)

/* The MUX transfer timeout is set to 10 ms timeout period. */
#define MUX_I2C_TIMER_PERIOD                        (10u)

/*
 * Delay to be applied to allow USB connections through the MUX to get enabled.
 * Using 50 ms as PI3DPX1205A datasheet specifies 3 ms delay.
 */
#define MUX_INIT_DELAY_MS                           (50u)

/***********************************************************************************/

/* Disable CCG device reset on error (watchdog expiry or hard fault). */
#define RESET_ON_ERROR_ENABLE                       (1u)

/* Enable reset reporting through HPI. */
#define HPI_WATCHDOG_RESET_ENABLE                   (1u)

/* Watchdog reset timer id. */
#define WATCHDOG_TIMER_ID                           (0xD0u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                    (750u)

/* Disable tracking of maximum stack usage. */
#define STACK_USAGE_CHECK_ENABLE                    (0u)

/***********************************************************************************/

/*********************************** FET Types *************************************/

#define CCG_SRC_FET                                 (1)
#define CCG_SNK_FET                                 (0)

/*********************************** OVP *******************************************/

/* VBus OVP enable setting. */
#define VBUS_OVP_ENABLE                             (1u)

/*
 * OVP mode selection
 * 0 - OVP using ADC comparator.
 * 1 - OVP using dedicated comparator. Firmware detects trip interrupt and turns off the FETs.
 * 2 - OVP using dedicated comparator. Hardware detects trip interrupt and turns off the FETs.
 *
 * We recommend setting this to 2 for all CCG5 designs.
 */
#define VBUS_OVP_MODE                               (2u)

/************************************ OCP *******************************************/

/*
 * VBus OCP feature enable.
 */
#define VBUS_OCP_ENABLE                             (1u)

/*
 * Software OCP mode.
 * 0 - External OCP hardware.
 * 1 - Internal OCP with neither software debounce nor automatic FET control.
 * 2 - Internal OCP with automatic FET control by hardware when an OCP event is
 *     detected.
 * 3 - Internal OCP with software debounce using delay in milliseconds from the
 *     config table.
 *
 * We recommend setting this to 3 for all CCG5 designs.
 */
#define VBUS_OCP_MODE                               (3u)

/*
 * VConn OCP feature enable.
 */
#define VCONN_OCP_ENABLE                            (1u)

#endif /* _CONFIG_H_ */

/* End of file */
