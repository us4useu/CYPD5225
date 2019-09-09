/**
 * @file ridge_slave.h
 *
 * @brief @{Alpine/Titan Ridge I2C slave interface header file.@}
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
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWAR/TRE
 * IS PROVIDED AS-IS, WITH NO WAR/TRRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WAR/TRRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PAR/TRTICULAR/TR PURPOSE. Cypress reserves the
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

#ifndef _RIDGE_SLAVE_H_
#define _RIDGE_SLAVE_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "status.h"
#include "i2c.h"

/**
 * @brief Default SCB index used for Alpine/Titan Ridge Slave interface.
 */
#define RIDGE_SLAVE_SCB_INDEX                      (0x1)

/**
 * @brief AR/TR Slave interface clock frequency can be upto 1 MHz.
 */
#define RIDGE_SLAVE_SCB_CLOCK_FREQ                 (I2C_SCB_CLOCK_FREQ_1_MHZ)

/**
 * @brief Minimum AR/TR slave write size: Slave address + Register Address.
 */
#define RIDGE_SLAVE_MIN_WRITE_SIZE                 (2)

/**
 * @brief Maximum AR/TR slave write size: We should never get writes longer than 32 bytes.
 */
#define RIDGE_SLAVE_MAX_WRITE_SIZE                 (16)

/**
 * @brief Maximum AR/TR slave read size.
 */
#define RIDGE_SLAVE_MAX_READ_SIZE                  (16)

/**
 * @brief Slave address associated with USB-PD port number 0. This is defined by Intel.
 */
#define RIDGE_SLAVE_ADDR_P0                        (0x38)

/**
 * @brief Slave address associated with USB-PD port number 1. This is defined by Intel.
 */
#define RIDGE_SLAVE_ADDR_P1                        (0x3F)

/**
 * @brief I2C slave address mask to be applied on the incoming slave address.
 *
 * Since a single I2C block is used to handle transaction on two different slave addresses,
 * a mask needs to be applied while checking the incoming preamble for an address match.
 * This mask allows comparison of only the address bits that do not change across the two
 * addresses.
 */
#define RIDGE_SLAVE_ADDR_MASK                      (0xF0)

/**
 * @brief Alpine/Titan Ridge command value that requests a CCG device reset.
 */
#define RIDGE_CMD_CCG_RESET                        (0x02)

/**
 * @brief Alpine/Titan Ridge command value to clear the interrupt from CCG.
 */
#define RIDGE_CMD_INT_CLEAR                        (0x04)

    /**
 * @brief Alpine/Titan Ridge command IRQ ACK PD Controller to Titan Ridge.
 */

#define RIDGE_IRQ_ACK                              (0x2000)
/**
 * @brief List of Alpine/Titan Ridge slave interface registers.
 *
 * The Thunderbolt Alternate Mode specification defines the following set of registers
 * that should be implemented by a USB-PD port controller in Thunderbolt enabled systems.
 */
typedef enum
{
    RIDGE_REG_CCG_COMMAND = 0x50,                  /**< CCG command register. */
    RIDGE_REG_CCG_STATUS  = 0x5F                   /**< CCG status register. */
} ridge_slave_reg_addr_t;

/**
 * @brief Initialize the Alpine/Titan Ridge slave interface module.
 *
 * This function initializes the Alpine/Titan Ridge slave interface module and configures it
 * to use the specified SCB block. The SCB will be configured as an I2C slave block,
 * and the interrupt output will also be initialized to a de-asserted state.
 *
 * Since only two registers are to be implemented, and the commands to be implemented are
 * simple, the complete module is implemented using the I2C command callbacks.
 *
 * @param ar_scbnum SCB index to be used for the Alpine/Titan Ridge slave interface.
 * @param portsel Alpine/Titan Ridge slave port selection. Only applicable for CCG3 designs.
 *
 * @return None
 */
void ridge_slave_init(uint8_t ar_scbnum, uint8_t portsel);

/**
 * @brief Update the AR/TR status register and send an event to the Alpine/Titan Ridge.
 *
 * This function is used by the application layer to update the content of the Alpine/Titan
 * Ridge status register. If the content of the register is changing, CCG asserts
 * the corresponding interrupt to notify Alpine/Titan Ridge about the status change.
 *
 * @param port USB-PD port index corresponding to the status update.
 * @param status Value to be written into the status register.
 * @param rewrite Flag to enable updating of the status register even it remains the same.
 * This feature are using for HPD status updating.
 *
 * @return None
 */
void ridge_slave_status_update(uint8_t port, uint32_t status, bool rewrite);

/**
 * @brief Handler for pending Ridge Slave interface tasks.
 * @return None
 */
void ridge_slave_task(void);

/**
 * @brief Check whether the AR/TR slave interface is idle so that device can be placed into sleep.
 *
 * This function should be called prior to placing the CCG device in deep sleep. Deep sleep
 * entry is only allowed if this function returns true.
 *
 * @return true if the interface is idle, false otherwise.
 */
bool ridge_slave_sleep(void);

/**
 * @brief Resets Ridge related registers.
 * @param port USB-PD port index corresponding to the register reset.
 * @return None
 */
void ridge_reg_reset(uint8_t port);

/**
 * @brief Check whether a host is connected to the specified PD port.
 * @param port Index of the PD port.
 * @return true if a host is connected, false otherwise.
 */
bool ridge_slave_is_host_connected(uint8_t port);

/**
 * @brief Update the status register with Over-Current status of the port.
 * @param port PD port index.
 * @param status Whether the port currently has an over-current condition.
 * @return None
 */
void ridge_slave_update_ocp_status(uint8_t port, bool status);

#endif /* _RIDGE_SLAVE_H_ */

/* [] END OF FILE */
