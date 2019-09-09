/**
 * @file stack_params.h
 *
 * @brief @{Header file that defines parameters to configure the CCGx Firmware
 * Stack. The current definitions for these values are optimized for the CCG5
 * Notebook Port Controller implementation and should not be changed.
 *
 * Please contact Cypress for details of possible customizations in these
 * settings.@}
 *******************************************************************************
 *
 * Copyright (2017-2018), Cypress Semiconductor Corporation or a subsidiary of
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
#ifndef _STACK_PARAMS_H_
#define _STACK_PARAMS_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>

/*******************************************************************************
 * CCG Device Selection.
 ******************************************************************************/

/*
 * Select target device family. This definition is used to implement the PD
 * block driver.
 */
#define CCG5

/* Select target silicon ID for CYPD5225-96BZXI. */
#define CCG_DEV_SILICON_ID                      (0x2100)
#define CCG_DEV_FAMILY_ID                       (0x11B1)

/* No. of USB-PD ports supported. CYPD5225-96BZXI supports one port. */
#define NO_OF_TYPEC_PORTS                       (2u)

#define TYPEC_PORT_0_IDX                        (0u)
#define TYPEC_PORT_1_IDX                        (1u)

#if (NO_OF_TYPEC_PORTS >= 2)
    /* Set this flag to enable the second PD port. */
    #define CCG_PD_DUALPORT_ENABLE              (1u)
#else
    #define CCG_PD_DUALPORT_ENABLE              (0u)
#endif

/*******************************************************************************
 * Timer Module Configuration
 ******************************************************************************/

/*
 * The timer module provides software timer support. It provides multiple
 * timers running off a single hardware timer and has a general accuracy of 5%.
 * This module can generate callbacks at a granularity of 1ms. It provides
 * various backend implementations selectable via the following compile time
 * options. The various options can be selected by selecting the required value
 * for implemenation macro TIMER_TYPE.
 */

/*
 * SYS_TICK based timer backend, interrupting the system every 1ms.
 * This implementation requires the SYS_TICK timer to be reserved for the use
 * of this module. This implementation shall not function in DEEP_SLEEP mode
 * and the user should ensure that the timer is shut-off before entering
 * DEEP_SLEEP. To shut off the timer, just ensure that all soft-timers are
 * stopped or have expired.
 */
#define TIMER_TYPE_SYSTICK                      (1)

/*
 * WDT based timer backend.
 * This requires user to reserve both WDT and SYS_TICK timers for the use of this.
 * The WDT timer runs off ILO which is highly inaccurate. The SYS_TICK timer is
 * used to calibrate the WDT to match IMO accuracy. The WDT based
 * implementation works across DEEP_SLEEP.
 */
#define TIMER_TYPE_WDT                          (2)

/*
 * Timer implementation selection.
 * TIMER_TYPE_WDT should be used if deep sleep entry for power saving is
 * being used.
 */
#define TIMER_TYPE                              (TIMER_TYPE_WDT)

/*
 * In addition to the hardware timer options, the module also provides a
 * TICKLESS timer implementation. The TICKLESS implementation is currently
 * available only for WDT based timer backend. The TICKLESS timer interrupts
 * the system only at the timer expiry (instead of every 1ms). Since this
 * involves a more complex algorithm, it requires more code space (about 200
 * bytes more). This implementation allows the same timer to be used in ACTIVE
 * as well as DEEP_SLEEP modes, due to the WDT backend. It also gives maximum
 * power savings as well as faster execution due to less number of interrupts.
 */
#define TIMER_TICKLESS_ENABLE                   (1)

/*
 * Timer module supports multiple software instances based on a single hardware
 * timer. The number of instances is defined based on the PD port count.
 */
#define TIMER_NUM_INSTANCES                     (NO_OF_TYPEC_PORTS)

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EN_TIMER_PERIOD             (250u)

/* Period (in ms) of VBus validity checks after enabling the power source. */
#define APP_PSOURCE_EN_MONITOR_TIMER_PERIOD     (1u)

/* Time (in ms) between VBus valid and triggering of PS_RDY. */
#define APP_PSOURCE_EN_HYS_TIMER_PERIOD         (5u)

/* Time (in ms) for which the VBus_Discharge path will be enabled when turning power source OFF. */
#define APP_PSOURCE_DIS_TIMER_PERIOD            (600u)

/* Period (in ms) of VBus drop to VSAFE0 checks after power source is turned OFF. */
#define APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD    (1u)

/* VBus Monitoring is done using internal resistor divider. */
#define VBUS_MON_INTERNAL                       (1u)

/* Allowed VBus valid margin as percentage of expected voltage. */
#define VBUS_TURN_ON_MARGIN                     (-20)

/* Allowed margin over expected voltage (as percentage) for negative VBus voltage transitions. */
#define VBUS_DISCHARGE_MARGIN                   (10u)

/* Allowed margin over 5V before the provider FET is turned OFF when discharging to VSAFE0. */
#define VBUS_DISCHARGE_TO_5V_MARGIN             (10u)

/*******************************************************************************
 * ADC selection for various functions.
 ******************************************************************************/

/* CCG5 has only one ADC (ADC_ID_0) per port. */
#define APP_VBUS_POLL_ADC_ID                    (PD_ADC_ID_0)

/* Internal VBus divider is connected to AMUX_B input of the ADC. */
#define APP_VBUS_POLL_ADC_INPUT                 (PD_ADC_INPUT_AMUX_B)

#endif /* _STACK_PARAMS_H_ */

/* End of file */

