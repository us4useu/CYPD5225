/**
 * @file config.h
 *
 * @brief @{Header file that enables/disables various CCG bootloader features.@}
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
#ifndef __CONFIG_H__
#define __CONFIG_H__

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <project.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/* Select target device family. */
#define CCG5

/* Enable all boot loader specific code. */
#define CCG_BOOT                     (1u) 

/* Select target silicon ID for CYPD5125-40LQXI. */
#define CCG_DEV_SILICON_ID           (0x2101)
#define CCG_DEV_FAMILY_ID            (0x11B1)

/*System Macros*/
#define NO_OF_TYPEC_PORTS            (1u)
#define TYPEC_PORT_0_IDX             (0u)
#define TYPEC_PORT_1_IDX             (1u)

/*
 * Index of SCB used for HPI interface. This should be set based on
 * the pin selection in the project schematic.
 */
#define HPI_SCB_INDEX                       (0u)

/* Disable I2C Address configuration based on SWD_CLK. */
/* Uncomment this to disable I2C Address configuration.*/
#define DISABLE_I2C_ADDR_CONFIG             (1u)

/* Enable HPI support. */
#define CCG_HPI_ENABLE                      (1u)

/* Enable PD policy registers in HPI. */
#define CCG_HPI_PD_ENABLE                   (0u)

/* Disable image selection based on APP Priority Field. */
#define APP_PRIORITY_FEATURE_ENABLE         (0u)

/* Disable Secure Boot. */
#define SECURE_FW_UPDATE                    (0u)

/* Enabling flashing of the device via HPI interface. */
#define FLASHING_MODE_HPI_ENABLE            (1u)

/* Whether dual app bootloading is disabled. */
#define CCG_DUALAPP_DISABLE                 (1u)

#if CCG_DUALAPP_DISABLE
    /* Stub definitions in case dual-app bootloading is disabled. */
    #define Bootloader_1_MD_BTLDB_ACTIVE_1      (0x01u)
    #define Bootloader_1_MD_BTLDB_ACTIVE_NONE   (0x02u)

    extern volatile uint8_t Bootloader_1_activeApp;
#endif /* CCG_DUALAPP_DISABLE */

/*******************************************************************************
 * Timer Module Configuration
 ******************************************************************************/

/*
 * The timer module provides software timer support. The module provides
 * multiple timers running off a single hardware timer and has a general
 * accuracy of 5%. The timer module can be used to generate callbacks at a
 * ganularity of 1ms. The module provides various implementations selectable
 * via compile time options. The various options can be selected by selecting
 * the required value for implemenation macro TIMER_TYPE.
 */

/*
 * SYS_TICK based timer interrupting system every 1ms.
 * This implementation requires the SYS_TICK timer to be reserved for the use
 * of this module. This implementation shall not function in DEEP_SLEEP mode
 * and user should ensure that the timer is shut-off before entering
 * DEEP_SLEEP. To shut off timer, just ensure that all soft-timers are stopped
 * or has expired.
 */
#define TIMER_TYPE_SYSTICK                      (1)

/*
 * WDT based implementation: This requires user to reserve both WDT and
 * SYS_TICK timers for internal use. The WDT timer runs off ILO which is highly
 * inaccurate. The SYS_TICK timer is used to calibrate the WDT to match IMO
 * accuracy. The WDT based implementation works across DEEP_SLEEP.
 */
#define TIMER_TYPE_WDT                          (2)

/*
 * Choose the timer implementation to be used.
 * TIMER_TYPE_WDT should be used if deep sleep entry for power saving is
 * being used.
 */
#define TIMER_TYPE                              (TIMER_TYPE_WDT)

/*
 * In addition to the hardware timer options, the module also provides TICKLESS
 * implementation. The TICKLESS implementation is currently available only for
 * WDT based timer implementation. The TICKLESS timer interrupts the system
 * only at the timer expiry (instead of every 1ms). Since this involves more
 * complex algorithm, it requires more code space (about 200 bytes more). This
 * implementation allows the same timer to be used in ACTIVE as well as
 * DEEP_SLEEP modes with maximum power savings as well as faster execution due
 * to less number of interrupts.
 */
#define TIMER_TICKLESS_ENABLE                   (1)

/*
 * Timer module supports multiple software instances based on a single hardware
 * timer. The number of instances is defined based on the PD port count.
 */
#define TIMER_NUM_INSTANCES                     (NO_OF_TYPEC_PORTS)

/*
 * Timer ID allocation for various solution soft timers.
 */

/*
 * Boot-loader application timer instance to be used.
 */
#define BL_TIMER_INSTANCE                       (0)

/* Enable boot-wait window. */
#define BOOTWAIT_ENABLE                         (0)

/*
 * Boot-wait window timer. This timer allows the device to wait in boot-loader
 * mode before jumping to a valid firmware image. This feature and delay value
 * is configured via meta-data row information.
 */
#define BL_BOOT_WAIT_TIMER_ID                   (0xC0)

#endif /* __CONFIG_H__ */

/* End of File */
