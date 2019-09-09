/**
 * @file solution.h
 *
 * @brief @{Header file for one port CCG5 NON-TBT solution layer.@}
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

#ifndef _SOLUTION_H_
#define _SOLUTION_H_

#include <stdbool.h>
#include <pd.h>

/*******************************************************************************
 * MACROS
 ******************************************************************************/

/* NCP Regulator I2C slave addresses for each PD port. */
#define NCP_PORT_1_SLAVE_ADDR                       (0x74u)
#define NCP_PORT_2_SLAVE_ADDR                       (0x75u)

/* Resolution of voltage changes (in mV) supported by the NCP regulator. */
#define NCP_REG_VOLT_RESOLUTION                     (100u)

/*
 * Excess voltage to be configured to make up for in-system drops. The unit used
 * is the resolution supported by the regulator itself.
 */
#define NCP_REG_EXCESS_VOLTAGE                      (1u)

/* Pericom MUX I2C slave addresses for each PD port. */
#define MUX_PORT_1_SLAVE_ADDR                       (0x50u)
#define MUX_PORT_2_SLAVE_ADDR                       (0x57u)

/* CCG controlled switch for DisplayPort and USB lines. */
#define DP_MUX                                      (0u)

/* This firmware supports only CCG controlled DP/USB switch operation. */
#define MUX_TYPE                                    DP_MUX

/* Size of writes (in bytes) to configure the MUX. */
#define MUX_BUFF_SIZE                               (4u)

/* Offset to the actual MUX configuration byte in the write data. */
#define MUX_CONFIG_IDX                              (3u)

/* Enables the WatchDog Timer */
#define MUX_WDT_EN(status)                          (status |= 0x02u)

/* MUX operation modes macros */
#define MUX_OP_MODE_SEL_SHIFT                       (5u)   /* [6:5]   Operation Mode Selection. */
#define MUX_SAFE_STATE                              (0x00u << MUX_OP_MODE_SEL_SHIFT)
#define MUX_USB_SS                                  (0x02u << MUX_OP_MODE_SEL_SHIFT)
#define MUX_DP_4_LANE                               (0x01u << MUX_OP_MODE_SEL_SHIFT)
#define MUX_DP_2_LANE                               (0x03u << MUX_OP_MODE_SEL_SHIFT)

/* Set MUX polarity macroses */
#define MUX_POLARITY_SHIFT                          (4u)   /* [4]   Polarity Selection. */
#define MUX_SET_POLARITY(status, polarity)          (status |= (polarity << MUX_POLARITY_SHIFT))

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief Configure the I2C interface
 * @return None.
 */
void i2cm_init(void);

/**
 * @brief Send data to I2C Slave device
 * @param addr Slave address
 * @param buffer Data to be send to I2C Slave
 * @param count Data size
 * @return Returns true if the operation is successful, false otherwise
 */
bool I2C_Write(uint8_t addr, uint8_t *buffer, uint32_t count);

/**
 * @brief This function enables voltage sourcing.
 *
 * This function invokes the defualt source enable routine used by PD stack.
 * Then it enables NCP81239 controller.
 *
 * @param port Port index the function is performed for.
 * @param pwr_ready_handler Application handler callback function.
 *
 * @return None.
 */
void sln_psrc_enable(uint8_t port, pwr_ready_cbk_t pwr_ready_handler);

/**
 * @brief This function disables voltage sourcing.
 *
 * This function invokes the defualt source disable routine used by PD stack.
 * Then it disables NCP81239 controller.
 *
 * @param port Port index the function is performed for.
 * @param pwr_ready_handler Application handler callback function.
 *
 * @return None.
 */
void sln_psrc_disable(uint8_t port, pwr_ready_cbk_t pwr_ready_handler);

#endif /* _SOLUTION_H_ */

/* [] END OF FILE */

