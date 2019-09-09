/**
 * @file hal_ccgx.h
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

#ifndef _HAL_CCGX_H_
#define _HAL_CCGX_H_

#include "config.h"
#include "pd.h"
#include "stdint.h"
#include "stdbool.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/*
 * The following definitions are clock source IDs for various sub-blocks. These
 * are defined in hardware and should not be modified.
 */

#if defined CCG3 || defined CCG4
#define PDSS_PORT0_PCLK_RX_IDX          (8u)
#define PDSS_PORT0_PCLK_TX_IDX          (9u)
#define PDSS_PORT0_PCLK_SAR_IDX         (10u)

#ifdef CCG3
#define PDSS_PORT0_PCLK_SWAP_IDX        (11u)
#else
#define PDSS_PORT0_PCLK_SWAP_IDX        (6u)
#endif /* CCG3 */

#define PDSS_PORT1_PCLK_RX_IDX          (11u)
#define PDSS_PORT1_PCLK_TX_IDX          (12u)
#define PDSS_PORT1_PCLK_SAR_IDX         (13u)
#define PDSS_PORT1_PCLK_SWAP_IDX        (7u)

#endif /* defined CCG3 || defined CCG4 */

#if (defined(CCG3PA) || defined(CCG3PA2))
#define PDSS_PORT0_PCLK_RX_IDX          (6u)
#define PDSS_PORT0_PCLK_TX_IDX          (7u)
#define PDSS_PORT0_PCLK_SAR_IDX         (8u)
#define PDSS_PORT0_PCLK_REFGEN_IDX      (10u)
#define PDSS_PORT0_PCLK_BCH_IDX         (11u)
#define PDSS_PORT0_PCLK_FILT1_IDX       (12u)
#define PDSS_PORT0_PCLK_FILT2_IDX       (13u)
#define PDSS_PORT0_PCLK_ISINK_IDX       (14u)
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#ifdef CCG5

/* Clock divider ids for PD blocks on the CCG5 device. */
#define PDSS_PORT0_PCLK_RX_IDX          (6u)
#define PDSS_PORT0_PCLK_TX_IDX          (7u)
#define PDSS_PORT0_PCLK_SAR_IDX         (8u)
#define PDSS_PORT0_PCLK_SWAP_IDX        (9u)
#define PDSS_PORT0_PCLK_FILT1_IDX       (10u)
#define PDSS_PORT0_PCLK_FILT2_IDX       (11u)
#define PDSS_PORT0_PCLK_REFGEN_IDX      (12u)

#define PDSS_PORT1_PCLK_RX_IDX          (13u)
#define PDSS_PORT1_PCLK_TX_IDX          (14u)
#define PDSS_PORT1_PCLK_SAR_IDX         (15u)
#define PDSS_PORT1_PCLK_SWAP_IDX        (16u)
#define PDSS_PORT1_PCLK_FILT1_IDX       (17u)
#define PDSS_PORT1_PCLK_FILT2_IDX       (18u)

#endif /* CCG5 */

/* Macros for OCP table. */
#if defined(CCG3) || defined(CCG5)
#define MA_PER_UNIT                     (10u)
#define VSENSE_MIN                      (10)
#define VSENSE_MAX                      (72)
#endif /* CCGx */

/**< Revision id corresponding to Rev ** version of any CCG silicon. */
#define CCG_SILICON_REV00_VALUE         (0x11)

#ifdef CCG5

/**< T_VCONN value corresponding to CC1/CC2 OVP detection enable. */
#define CCG5_CC_OVP_DET_ENABLE_VAL      (0x04)

#endif /* CCG5 */

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @typedef vbus_ocp_cbk_t
 * @brief VBus over current protection callback function.
 */
typedef void (*vbus_ocp_cbk_t)(
        uint8_t port     /**< USB-PD port on which over-current event occurred. */
        );

extern const uint8_t csa_tab[];

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief USB-PD system initialization.
 *
 * This function initializes the PD block clocks by setting the divider values
 * for PERI registers and enabling the corresponding control registers.
 */
void system_init(void);

/**
 * @brief Enables VBus OCP checks on the specified port.
 *
 * @param port USB-PD port on which to enable VBus OCP checks.
 * @param cbk Function to be called when OCP event is detected.
 *
 * @return Returns 1 if params are ok else return 0
 */
uint8_t system_vbus_ocp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk);

/**
 * @brief Disables VBus OCP checks on the specified port.
 *
 * @param port USB-PD port on which to disable VBus OCP checks.
 *
 * @return Returns 1 if params are ok else return 0
 */
uint8_t system_vbus_ocp_dis(uint8_t port);

/**
 * @brief Vbus OCP interrupt handler.
 *
 * @param port USB-PD port on which the OCP interrupt was triggered.
 *
 * @return None
 */
void vbus_ocp_handler(uint8_t port);

/**
 * @brief This function disconnects the OVP comparator output from OVP Trip GPIO
 * Before disconnecting this function also make sure OVP Trip GPIO is driving
 * strong the last output of comparator. This is useful in using the same comparator
 * for other level comparision.
 * @param port USB-PD port
 *
 * @return None
 */
void system_disconnect_ovp_trip(uint8_t port);

/**
 * @brief This function connects the OVP comparator output to OVP Trip GPIO
 * @param port USB-PD port
 *
 * @return None
 */
void system_connect_ovp_trip(uint8_t port);

/**
 * @brief Enables VBus SCP checks on the specified port.
 *
 * @param port USB-PD port on which to enable VBus SCP checks.
 * @cur Negotiated current in 10mA units.
 * @param cbk Function to be called when SCP event is detected.
 *
 * @return Returns 1 if params are ok else return 0
 */
uint8_t system_vbus_scp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk);

/**
 * @brief Disables VBus SCP checks on the specified port.
 *
 * @param port USB-PD port on which to disable VBus SCP checks.
 *
 * @return Returns 1 if params are ok else return 0
 */
uint8_t system_vbus_scp_dis(uint8_t port);

/**
 * @brief Vbus SCP interrupt handler.
 *
 * @param port USB-PD port on which the SCP interrupt was triggered.
 *
 * @return None
 */
void vbus_scp_handler(uint8_t port);

#endif /* _HAL_CCGX_H_ */

/**
 * @brief Function to check CCG silicon revision.
 * @return Returns the silicon revision of the CCG part. 0 for Rev ** and non-zero for others.
 */
uint8_t ccg_get_si_revision(void);

/* End of file */

