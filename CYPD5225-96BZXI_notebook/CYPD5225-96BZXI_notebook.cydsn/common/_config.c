/**
 * @file config.c
 *
 * @brief Contains the CCG device configuration information. Please refer to
 * the firmware documentation and the EZ-PD Configuration Utility for details
 * of the configuration table.
 *
 *******************************************************************************
 * @copyright
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

#include <project.h>
#include <pd.h>

/*
   The config table should be placed in the "configSection" so that the update
   tools can correctly retrieve the data.
 */
const unsigned char __attribute__ ((section (".configSection"), used)) gl_config_table[0x0400]=
{
    0x59,0x43,0x04,0x00,0x00,0x20,0x00,0x04,0xDB,0x3C,0xB4,0x04,0x01,0x00,0x02,0x00,
    0x00,0x02,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x0C,0x02,0x18,0x00,0x24,0x02,0x0C,0x00,0x30,0x02,0x0C,0x00,0x3C,0x02,0x18,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE8,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
    0x02,0x01,0x02,0x00,0x00,0x00,0x01,0x00,0x01,0x04,0x0F,0x02,0x03,0x07,0x01,0x01,
    0x01,0x01,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x2C,0x91,0x01,0x27,0x2C,0xD1,0x02,0x00,0x2C,0xB1,0x04,0x00,0x2C,0x41,0x06,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0x90,0x01,0x26,
    0x5A,0x30,0x42,0x9A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0x00,0x5A,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x00,0x54,0x02,0x58,0x02,0x60,0x02,
    0x64,0x02,0x68,0x02,0x6C,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x78,0x02,0x18,0x00,0x90,0x02,0x0C,0x00,0x9C,0x02,0x0C,0x00,0xA8,0x02,0x18,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE8,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
    0x02,0x01,0x02,0x00,0x00,0x00,0x01,0x00,0x01,0x04,0x0F,0x02,0x03,0x07,0x01,0x01,
    0x01,0x01,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x2C,0x91,0x01,0x27,0x2C,0xD1,0x02,0x00,0x2C,0xB1,0x04,0x00,0x2C,0x41,0x06,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0x90,0x01,0x26,
    0x5A,0x30,0x42,0x9A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0x00,0x5A,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x00,0xC0,0x02,0xC4,0x02,0xCC,0x02,
    0xD0,0x02,0xD4,0x02,0xD8,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xB4,0x04,0xE0,0xF6,0x43,0x79,0x70,0x72,0x65,0x73,0x73,0x00,0x18,0x00,0x00,0x00,
    0x41,0x80,0x00,0xFF,0xB4,0x04,0x00,0xAE,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xF6,
    0x0A,0x00,0x00,0x00,0x0C,0x00,0x00,0x00,0x42,0x80,0x00,0xFF,0x00,0x00,0x87,0x80,
    0x0C,0x00,0x00,0x00,0x43,0x80,0x87,0x80,0x01,0x00,0x00,0x00,0xB4,0x04,0xE0,0xF6,
    0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x01,0x00,0x3C,0x04,0x14,0x0A,0x02,0x08,0x14,0x0A,0x02,0x32,0x01,0x0A,0x00,
    0x04,0x46,0x01,0x00,0x04,0x32,0x01,0x00,0x04,0x1E,0x01,0x00,0x0C,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x41,0x80,0x00,0xFF,
    0xB4,0x04,0x00,0xAE,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xF6,0x0A,0x00,0x00,0x00,
    0x0C,0x00,0x00,0x00,0x42,0x80,0x00,0xFF,0x00,0x00,0x87,0x80,0x0C,0x00,0x00,0x00,
    0x43,0x80,0x87,0x80,0x01,0x00,0x00,0x00,0xB4,0x04,0xE0,0xF6,0x00,0x00,0x00,0x00,
    0x01,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x3C,
    0x04,0x14,0x0A,0x02,0x08,0x14,0x0A,0x02,0x32,0x01,0x0A,0x00,0x04,0x46,0x01,0x00,
    0x04,0x32,0x01,0x00,0x04,0x1E,0x01,0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

const pd_config_t * get_pd_config(void)
{
    return (pd_config_t *)&gl_config_table;
}

const pd_port_config_t * get_pd_port_config(uint8_t port)
{
    return (pd_port_config_t *)&(((pd_config_t *)gl_config_table)->port_conf[port]);
}

/* End of File */
