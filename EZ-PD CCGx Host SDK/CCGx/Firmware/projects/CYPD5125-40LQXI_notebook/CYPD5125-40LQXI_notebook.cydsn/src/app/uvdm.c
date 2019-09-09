/**
 * @file uvdm.c
 *
 * @brief @{Unstructured VDM handler source file.@}
 *
 *******************************************************************************
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

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "pd.h"
#include "flash_config.h"
#include "status.h"
#include "system.h"
#include "utils.h"
#include "flash.h"
#include "boot.h"
#include "uvdm.h"

#if (SECURE_FW_UPDATE == 1)
#include <crypto_hal.h>
#include <secure_boot.h>
#endif /* SECURE_FW_UPDATE */

#if QC_4_0_ENABLE
#include "dpm.h"
#include "app.h"
#endif /* QC_4_0_ENABLE */

/*
 * Following macros are used for data memory write and read operations.
 * Note that CCG3 has 128 bytes flash row and CCG4 has 256 bytes flash row
 * size. Hence, Macros are defined based on size of flash row.
 */

/* Index of first section of Data Buffer. */
#define UVDM_DATA_BUF_FIRST_SEC_INDEX                           (0x01)

/* Size of first five sections of Data Buffer in bytes. */
#define UVDM_DATA_BUFFER_SECTION_SIZE_24                        (24)

#if (CCG_FLASH_ROW_SIZE == 128u)
/* Index of last section of Data Buffer. */
#define UVDM_DATA_BUF_LAST_SEC_INDEX                            (0x06)

/* Number of Data Memory Sections. */
#define UVDM_NUM_DATA_MEMORY_SECTIONS                           (0x06)

/* Size of last section of Data Buffer in bytes. */
#define UVDM_DATA_BUFFER_LAST_SECTION_SIZE                      (0x08)

/*
 * Mask to determine if all sections are written to before FLASH_WRITE
 * command. One bit represents one section in the data buffer.
 */
#define UVDM_SEND_DATA_SECTION_UPDATE_MASK                      (0x003F)

#elif (CCG_FLASH_ROW_SIZE == 256u)
/* Index of last section of Data Buffer. */
#define UVDM_DATA_BUF_LAST_SEC_INDEX                            (0x0B)

/* Number of Data Memory Sections. */
#define UVDM_NUM_DATA_MEMORY_SECTIONS                           (0x0B)

/* Size of last section of Data Buffer in bytes. */
#define UVDM_DATA_BUFFER_LAST_SECTION_SIZE                      (0x10)

/*
 * Mask to determine if all sections are written to before FLASH_WRITE
 * command. One bit represents one section in the data buffer.
 */
#define UVDM_SEND_DATA_SECTION_UPDATE_MASK                      (0x07FF)
#endif /* CCG_FLASH_ROW_SIZE */

/* Array of VDOs to hold response VDM. */
pd_do_t gl_response_vdo[UVDM_RESPONSE_MAX_NO_OF_VDO];

/*
 * Data buffer (of size one Flash row) to store data related
 * with flashing commands. Align it at Flash row size bytes boundary.
 */
typedef struct
{
    uint8_t srom_api_param[SROM_API_PARAM_SIZE];
    uint8_t data_buf[CCG_FLASH_ROW_SIZE];
} flash_mem_data_t;

flash_mem_data_t gl_data_buf __attribute__((aligned(CCG_FLASH_ROW_SIZE)));

/* Current write pointer in the data buffer. */
uint8_t gl_write_data_pointer = 0;

/* Current read pointer in the data buffer. */
uint8_t gl_read_data_pointer = 0;

/*
 * In Non-Blocking Flash update case, this flag ensures that
 * system handles only one FLASH WRITE request at a time.
 */
static volatile bool gl_flash_write_cmd_pending = false;

#if (CCG_BOOT == 0)
/*
 * This is the variable which is used to keep track of SEND_DATA VDMs.
 * Basically, expectation is that all sections of data buffer are updated
 * through SEND_DATA UVDM Command before a FLASH_WRITE Command.
 * Usage: Each bit in this variable corresponds to one section of data buffer.
 * Clear the correspondig bit when that section is updated. If this variable is
 * zero when FLASH_WRITE request comes in, it ensures that all sections were
 * Restore, the value back after FLASH_WRITE command completes.
 */
static uint16_t gl_send_data_section_tracker = UVDM_SEND_DATA_SECTION_UPDATE_MASK;

/* Current Non blocking VDM Command. */
uvdm_cmd_opcode_t gl_cur_nb_cmd = 0;

uvdm_cmd_opcode_t uvdm_get_cur_nb_cmd(void)
{
    return gl_cur_nb_cmd;
}

void uvdm_reset_nb_cmd_state(void)
{
    /*
     * This marks the end of FLASH WRITE Command (Successful or Abort).
     * CCG can handle the next FLASH WRITE command from now.
     */
    gl_flash_write_cmd_pending = false;

    if (gl_cur_nb_cmd == UVDM_CMD_FLASH_WRITE_OPCODE)
    {
        /* Revert back SEND_DATA UVDM tracker. */
        gl_send_data_section_tracker = UVDM_SEND_DATA_SECTION_UPDATE_MASK;
    }
}
#endif /* (CCG_BOOT == 0) */

static bool uvdm_exit_flashing_mode(void)
{
#if (CCG_BOOT == 0)
    /* Check if CY Alternate mode was used to enable Flashing mode. If yes, exit Flashing mode. */
    if (flash_access_get_status (1 << FLASH_IF_UVDM))
#endif /* (CCG_BOOT == 0) */
    {
        flash_enter_mode (false, FLASH_IF_UVDM, true);
#if (FLASH_ENABLE_NB_MODE == 1)
        /* Check if a non-blocking flash write is active. If yes, start an abort operation. */
        if (gl_flash_write_cmd_pending)
        {
            /* Signal Flash module to abort the current flash write operation. */
            flash_non_blocking_write_abort ();
        }
#endif /* FLASH_ENABLE_NB_MODE */
        /* Valid request. */
        return true;
    }
#if (CCG_BOOT == 0)
    else
    {
        /* This is an invalid request. */
        return false;
    }
#endif /* (CCG_BOOT == 0) */
}

#if (CCG_BOOT == 0)
/* Flag to indicate CY Flashing mode entry. */
volatile bool gl_cy_flashing_mode = false;

void uvdm_enter_cy_alt_mode(void)
{
    gl_cy_flashing_mode = true;
}

void uvdm_exit_cy_alt_mode(void)
{
    /* Exit CY Alternate Mode. */
    gl_cy_flashing_mode = false;
    uvdm_exit_flashing_mode ();
}

bool uvdm_get_flashing_mode(void)
{
    /* Return the state of CY flashing mode. */
    return gl_cy_flashing_mode;
}
#endif /* CCG_BOOT */

/*
 * Handles SEND_DATA command. Copies data into the data buffer.
 * Sequnce number determines which section of data buffer to update.
 *
 * @param data Pointer to data array
 * @param size Size of data in bytes
 * @param seq_num Sequnce number of data
 * @return bool
 */
static bool uvdm_update_data_memory(uint8_t* data, uint8_t size, uint8_t seq_num)
{
    /* Ensure Sequence number is valid. */
    if ((seq_num < UVDM_DATA_BUF_FIRST_SEC_INDEX) ||
            (seq_num > UVDM_DATA_BUF_LAST_SEC_INDEX))
    {
        return false;
    }

    /* Calculate the write data pointer based on sequence number. */
    gl_write_data_pointer = (seq_num - UVDM_DATA_BUF_FIRST_SEC_INDEX)
                *UVDM_DATA_BUFFER_SECTION_SIZE_24;

    /* Ensure the size of data for section 1 to 5 is 24 bytes. */
    if (((seq_num < UVDM_DATA_BUF_LAST_SEC_INDEX) && (size != UVDM_DATA_BUFFER_SECTION_SIZE_24))
        || ((seq_num == UVDM_DATA_BUF_LAST_SEC_INDEX) && (size != UVDM_DATA_BUFFER_LAST_SECTION_SIZE)))
    {
        return false;
    }

    /* Copy data. */
    memcpy (&gl_data_buf.data_buf[gl_write_data_pointer], data, size);

#if (CCG_BOOT == 0)
    /* Updated SEND_DATA UVDM tracker. */
    gl_send_data_section_tracker &= ~(1 << (seq_num-1));
#endif /* (CCG_BOOT == 0) */
    return true;
}

#if (SECURE_FW_UPDATE == 0)
/*
 * Handles READ_DATA command. Reads data from the data buffer based on
 * sequence number.
 *
 * @param data Pointer to data array
 * @param size Size of data in bytes
 * @return NONE
 */
static bool uvdm_read_data_memory(uint8_t* data, uint8_t seq_num)
{
    uint8_t size = 0;

    /* Ensure Sequence number is valid. */
    if ((seq_num < UVDM_DATA_BUF_FIRST_SEC_INDEX) ||
            (seq_num > UVDM_DATA_BUF_LAST_SEC_INDEX))
    {
        return false;
    }

    /* Calculate the read data pointer based on sequence number. */
    gl_read_data_pointer = (seq_num - UVDM_DATA_BUF_FIRST_SEC_INDEX)*
                UVDM_DATA_BUFFER_SECTION_SIZE_24;

    /* Get the size of data to read based on sequence number. */
    if (seq_num < UVDM_DATA_BUF_LAST_SEC_INDEX)
    {
        size = UVDM_DATA_BUFFER_SECTION_SIZE_24;
    }
    else
    {
        size = UVDM_DATA_BUFFER_LAST_SECTION_SIZE;
    }

    /* Copy data. */
    memcpy (data, &gl_data_buf.data_buf[gl_read_data_pointer], size);
    return true;
}
#endif /* SECURE_FW_UPDATE */

uvdm_response_state_t uvdm_handle_cmd(uint32_t *rx_pkt, pd_do_t **vdm_rspn_pkt,
    uint8_t *vdo_count, flash_cbk_t flash_cb)
{
    ccg_status_t response_code = CCG_STAT_NO_RESPONSE;
    uvdm_response_state_t response_state = UVDM_HANDLED_RESPONSE_READY;
    uint8_t *cmd_param;
    uvdm_cmd_opcode_t cmd_opcode;
    uint8_t cmd_length, seq_num;
    uint16_t row_num;
    pd_do_t* d_obj = (pd_do_t *)(&(rx_pkt[1]));

#if (CCG_BOOT == 0)
    /* Check if SVID is Flashing VID and CY Flashing Alternate mode is active. */
    if ((d_obj->ustd_vdm_hdr.svid != get_pd_config()->flashing_vid) ||
        (uvdm_get_flashing_mode() == false))
#else
    /* In boot mode, Check if SVID is Cypress VID. */
    if (d_obj->ustd_vdm_hdr.svid != CY_VID)
#endif /* CCG_BOOT */
    {
        /* Can't handle the VDM. */
        return UVDM_NOT_HANDLED;
    }

    /* Set up parameters. */
    cmd_param = (uint8_t *)&rx_pkt[2];
    cmd_opcode = d_obj->ustd_vdm_hdr.cmd;
    cmd_length = (GET_PD_HDR_CNT(rx_pkt[0]) - 1) << 2;
    seq_num = d_obj->ustd_vdm_hdr.seq_num;

    /*
     * Atleast one VDO needs to be sent back as response. Increment
     * this number as required by response.
     */
    uint8_t no_of_vdo = 1;

    /* Fill the header VDO with Cypress VID/Flashing VID and Responder ACK. */
#if (CCG_BOOT == 0)
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.svid = get_pd_config()->flashing_vid;
#else
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.svid = CY_VID;
#endif /* CCG_BOOT */
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
    /* Ensure sequence number field is updated in response header. */
    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.seq_num = d_obj->ustd_vdm_hdr.seq_num;

    /* Handle command. */
    switch (cmd_opcode)
    {
        case UVDM_CMD_GET_DEVICE_MODE_OPCODE:
            /*
             * Make sure cmd_length is 0 in this case, as no command specific
             * Data object is expected.
             */
            if (!cmd_length)
            {
                *((uint32 *)&gl_response_vdo[UVDM_DEVICE_MODE_VDO_INDEX])
                    =  sys_get_device_mode ();
                no_of_vdo++;
                /*
                 * If DEVICE MODE is bootloader, append
                 * bootloader last flash row in the second
                 * VDO.
                 */
                if (gl_response_vdo[UVDM_DEVICE_MODE_VDO_INDEX].val
                        == SYS_FW_MODE_BOOTLOADER)
                {
                    gl_response_vdo[UVDM_BOOT_LAST_ROW_VDO_INDEX].val
                        = CCG_BOOT_LOADER_LAST_ROW;
                    no_of_vdo++;
                }
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type =
                    CMD_TYPE_RESP_NAK;
            }
            break;

            /* Return the version of all three images. */
        case UVDM_CMD_GET_DEVICE_VERSION_OPCODE:
            /*
             * Make sure cmd_length is 0 in this case, as no command specific
             * Data object is expected.
             */
            if (!cmd_length)
            {
                /* Get boot loader version. */
                MEM_COPY ((uint8_t *)(&gl_response_vdo[UVDM_BOOT_VERSION_VDO_INDEX]),
                        sys_get_boot_version(), UVDM_VERSION_NUM_SIZE_BYTES);
                MEM_COPY ((uint8_t *)(&gl_response_vdo[UVDM_IMG1_VERSION_VDO_INDEX]),
                        sys_get_img1_fw_version(), UVDM_VERSION_NUM_SIZE_BYTES);
#if (!CCG_DUALAPP_DISABLE)
                MEM_COPY ((uint8_t *)(&gl_response_vdo[UVDM_IMG2_VERSION_VDO_INDEX]),
                        sys_get_img2_fw_version(), UVDM_VERSION_NUM_SIZE_BYTES);
#else
                gl_response_vdo[UVDM_IMG2_VERSION_VDO_INDEX].val = 0;
                gl_response_vdo[UVDM_IMG2_VERSION_VDO_INDEX + 1].val = 0;
#endif
                no_of_vdo += UVDM_GET_VERSION_U_VDM_NO_OF_VDO;
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

        case UVDM_CMD_GET_SILICON_ID_OPCODE:
            /* Validate command length and command signature. */
            if ((cmd_length == UVDM_GET_SILICON_ID_CMD_SIZE) &&
                    (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                     UVDM_GET_SILICON_ID_CMD_SIG))
            {
                /* Read Silicon ID. */
                sys_get_silicon_id ((uint32_t *)
                        &gl_response_vdo[UVDM_SILICON_ID_VDO_INDEX]);
                /* Read UUID. */
                CyGetUniqueId ((uint32 *)&gl_response_vdo[UVDM_SILICON_ID_VDO_INDEX+1]);
                /*
                 * Increment the VDO number to send the VDO containing Silicon ID
                 * and 2 VDOs for UUID.
                 */
                no_of_vdo += 3;
                response_code = CCG_STAT_SUCCESS;
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

        case UVDM_CMD_DEVICE_RESET_OPCODE:
            /* Validate command length and command signature. */
            if ((cmd_length == UVDM_DEVICE_RESET_CMD_SIZE) &&
                    (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                     UVDM_DEVICE_RESET_CMD_SIG))
            {
                response_code = uvdm_handle_device_reset (0);
                /* Check if no response. */
                if (response_code == CCG_STAT_NO_RESPONSE)
                {
                    /* No VDM response. */
                    response_state = UVDM_HANDLED_NO_RESPONSE;
                }
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

#if (CCG_BOOT == 0) && (CCG_FIRMWARE_APP_ONLY == 0)
        case UVDM_CMD_JUMP_TO_BOOT_OPCDOE:
            /* Validate Command Length. */
            if (cmd_length == UVDM_JUMP_TO_BOOT_CMD_SIZE)
            {
                /* Check if JUMP TO BOOT Request. */
                if (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                        UVDM_JUMP_TO_BOOT_CMD_SIG)
                {
                    /*
                     * We can branch to bootloader only if Bootloader has a FW
                     * update interface and does not support secure boot feature.
                     * Bootlaoder stores this info (BOOT_TYPE) at a fixed offset in
                     * flash. Take decision based on that.
                     */
                    if ((*(uint8_t *)(SYS_BOOT_TYPE_FIELD_OFFSET)) & (SYS_BOOT_TYPE_SECURE_BOOT_MASK
                                | SYS_BOOT_TYPE_FW_UPDATE_INTERFACE_MASK))
                    {
                        response_code = CCG_STAT_INVALID_COMMAND;
                    }
                    else
                    {
                        /* This function does not return. */
                        response_code = uvdm_handle_device_reset (CCG_BOOT_MODE_RQT_SIG);
                    }
                }
#if (!CCG_DUALAPP_DISABLE)
                /* Check if JUMP TO ALT Image Request. */
                else if (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                        UVDM_JUMP_TO_ALT_FW_SIG)
                {
#if (FLASH_ENABLE_NB_MODE == 0)
                    sys_fw_metadata_t *addr;
                    uint32_t bootsig;

                    /* If secure boot is supported, don't handle this request. */
                    if ((*(uint8_t *)(SYS_BOOT_TYPE_FIELD_OFFSET)) &
                            (SYS_BOOT_TYPE_SECURE_BOOT_MASK))
                    {
                        response_code = CCG_STAT_INVALID_COMMAND;
                    }
                    else
                    {
                        if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
                        {
                            addr    = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
                            bootsig = CCG_FW2_BOOT_RQT_SIG;
                        }
                        else
                        {
                            addr    = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
                            bootsig = CCG_FW1_BOOT_RQT_SIG;
                        }

                        if (boot_validate_fw (addr) != CCG_STAT_SUCCESS)
                        {
                            response_code = CCG_STAT_INVALID_FW;
                        }
                        else
                        {
                            response_code = CCG_STAT_SUCCESS;
                        }

                        /* If the alternate firmware is valid, we can proceed. */
                        if (response_code == CCG_STAT_SUCCESS)
                        {
                            /* Jump to alternate image. */
                            response_code = uvdm_handle_device_reset (bootsig);
                        }
                    }
#else
                    /*
                     * Don't support JUMP_TO_ALT image command if non blocking
                     * FW update feature is enabled.
                     */
                    response_code = CCG_STAT_INVALID_COMMAND;
#endif /* (FLASH_ENABLE_NB_MODE == 0) */
                }
#endif /* (!CCG_DUALAPP_DISABLE) */
                else
                {
                    response_code = CCG_STAT_INVALID_SIGNATURE;
                }

                /* Check if no response. */
                if (response_code == CCG_STAT_NO_RESPONSE)
                {
                    /* No VDM response. */
                    response_state = UVDM_HANDLED_NO_RESPONSE;
                }
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;
#endif /* (CCG_BOOT == 0) && (CCG_FIRMWARE_APP_ONLY == 0) */

        case UVDM_CMD_ENTER_FLASHING_MODE_OPCODE:
            /* Validate Command Length */
            if (cmd_length == UVDM_ENTER_FLASHING_MODE_CMD_SIZE)
            {
                /* Check if request is to Enable Flashing Mode. */
                if (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                        UVDM_ENTER_FLASHING_MODE_CMD_SIG)
                {
                    /* Check if Flashing Mode is already enabled. */
                    if (flash_access_get_status (~(1 << FLASH_IF_UVDM)))
                    {
                        /* This is an invalid command. */
                        response_code = CCG_STAT_INVALID_COMMAND;
                    }
                    else
                    {
                        flash_enter_mode (true, FLASH_IF_UVDM, true);
                        response_code = CCG_STAT_SUCCESS;
                    }
                }
                /* Request is to Exit Flashing Mode. */
                else
                {
                    /*
                     * If flashing mode is enable and CC was the active
                     * interface responsible for it, exit the mode and send
                     * success. Otherwise, send failure.
                     */
                    if (uvdm_exit_flashing_mode ())
                    {
                        response_code = CCG_STAT_SUCCESS;
                    }
                    else
                    {
                        response_code = CCG_STAT_INVALID_COMMAND;
                    }
                }
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

        case UVDM_CMD_SEND_DATA_OPCODE:
            /* Invoke update data memory function and pass the sequence number */
            if (true == uvdm_update_data_memory (cmd_param, cmd_length, seq_num))
            {
                /* No response to this VDM. */
                response_state = UVDM_HANDLED_NO_RESPONSE;
            }
            else
            {
                /*
                 * Set the NAK field in the header VDO as the command
                 * paramters are not correct.
                 */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

        case UVDM_CMD_FLASH_WRITE_OPCODE:
            /*
             * Make sure cmd_length is 4 in this case, as just 1 command specific
             * Data object is expected. Also ensure that there is no pending
             * FLASH WRITE command in the system.
             */
            if ((cmd_length == UVDM_FLASH_READ_WRITE_CMD_SIZE) &&
                    (!gl_flash_write_cmd_pending) &&
                    (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                     UVDM_FLASH_READ_WRITE_CMD_SIG))
            {
#if (CCG_BOOT == 0)
                /*
                 * Check the value of tracker to determine that all sections
                 * of data buffer were updated through SEND_DATA UVDM before sending
                 * FLASH WRITE request.
                 */
                if (gl_send_data_section_tracker)
                {
                    response_code = CCG_STAT_FLASH_UPDATE_FAILED;
                }
                else
#endif /* (CCG_BOOT == 0) */
                {
                    row_num = MAKE_WORD(cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET],
                                cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]);
                    response_code = flash_row_write (row_num, gl_data_buf.data_buf, flash_cb);
#if (CCG_BOOT == 0)
                    /*
                     * For non blocking case, if response is NO_RESPONSE, don't send
                     * Response VDM. Response VDM will be sent only after Flash Write
                     * completes.
                     */
                    if (response_code == CCG_STAT_NO_RESPONSE)
                    {
                        /* Set the flag to indicate that CCG is handling a FLASH write command currently. */
                        gl_flash_write_cmd_pending = true;
                        /* Response VDM is pending. */
                        response_state = UVDM_HANDLED_RESPONSE_NOT_READY;
                        gl_cur_nb_cmd = UVDM_CMD_FLASH_WRITE_OPCODE;
                    }
                    else

                    {
                        /* Revert back SEND_DATA UVDM tracker. */
                        gl_send_data_section_tracker =
                            UVDM_SEND_DATA_SECTION_UPDATE_MASK;
                    }
#endif /* (CCG_BOOT == 0) */
                    if(row_num == CCG_IMG1_METADATA_ROW_NUM)
                    {
                        gl_img_status.status.fw1_invalid = 1;
                    }
#if (!CCG_DUALAPP_DISABLE)
                    else if(row_num == CCG_IMG2_METADATA_ROW_NUM)
                    {
                        gl_img_status.status.fw2_invalid = 1;
                    }
#endif /* !CCG_DUALAPP_DISABLE */
                }
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

        case UVDM_CMD_READ_DATA_OPCODE:
#if (SECURE_FW_UPDATE == 0)
            /*
             *Invoke read data memory functiona and pass the sequence
             * number. Make sure cmd_length is 0 in this case, as no command specific
             * Data object is expected.
             */
            if ((!cmd_length) && (true == uvdm_read_data_memory ((uint8 *)
                            &gl_response_vdo[UVDM_READ_DATA_RESPONSE_VDO_INDEX], seq_num)))
            {
                /* Increment the number of VDOs based on sequence number */
                if (seq_num < UVDM_DATA_BUF_LAST_SEC_INDEX)
                {
                    no_of_vdo += UVDM_NUM_DATA_MEMORY_SECTIONS;
                }
                else
                {
                    no_of_vdo += UVDM_READ_DATA_NO_OF_VDO_ERROR_CASE;
                }
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
#else
            response_code = CCG_STAT_INVALID_COMMAND;
#endif /* SECURE_FW_UPDATE */
            break;

        case UVDM_CMD_FLASH_READ_OPCODE:
#if (SECURE_FW_UPDATE == 0)
            /*
             * Make sure cmd_length is 4 in this case, as just 1 command specific
             * Data object is expected.
             */
            if ((cmd_length == UVDM_FLASH_READ_WRITE_CMD_SIZE) &&
                    (cmd_param[UVDM_SIGNATURE_BYTE_OFFSET] ==
                     UVDM_FLASH_READ_WRITE_CMD_SIG))
            {
                response_code = flash_row_read (
                        MAKE_WORD(cmd_param[UVDM_FLASH_ROW_NUM_MSB_OFFSET],
                            cmd_param[UVDM_FLASH_ROW_NUM_LSB_OFFSET]), gl_data_buf.data_buf);

                if(response_code == CCG_STAT_SUCCESS)
                {
                    response_code = CCG_STAT_FLASH_DATA_AVAILABLE;
                }
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
#else
            response_code = CCG_STAT_INVALID_COMMAND;
#endif /* SECURE_FW_UPDATE */
            break;

        case UVDM_CMD_VALIDATE_FW_OPCODE:
            /*
             * Make sure cmd_length is 4 in this case, as just 1 command specific
             * Data object is expected.
             */
            if (cmd_length == UVDM_VALIDATE_FW_CMD_SIZE)
            {
                response_code = boot_handle_validate_fw_cmd (cmd_param[UVDM_VALIDATE_FW_MODE_INDEX]);
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;
#if (CCG_BOOT == 1)
        case UVDM_CMD_REASON_FOR_BOOT_MODE:
            /*
             * Make sure cmd_length is 0 in this case, as no command specific
             * Data object is expected.
             */
            if (!cmd_length)
            {
                /* Get the reason for boot mode. */
                *((uint8 *)&gl_response_vdo[UVDM_REASON_FOR_BOOT_MODE_VDO_INDEX])
                    = get_boot_mode_reason().val;
                /* Increment number of VDOs to be sent by 1. */
                no_of_vdo++;
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;
#endif /* CCG_BOOT */

        case UVDM_CMD_GET_CHECKSUM:
#if (SECURE_FW_UPDATE == 0)
            /* Check for flash address and flash size parameters. */
            if (cmd_length == UVDM_GET_CHECKSUM_CMD_SIZE)
            {
                uint8_t *addr = (uint8 *)MAKE_DWORD(cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET+3],
                        cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET+2], cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET+1],
                        cmd_param[UVDM_FLASH_ADDR_LSB_OFFSET]);
                uint32_t size = MAKE_DWORD(cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET+3],
                        cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET+2], cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET+1],
                        cmd_param[UVDM_FLASH_SIZE_LSB_OFFSET]);
                if (((uint32_t)addr + size) >= CCG_FLASH_SIZE)
                {
                    /* Set the NAK field in the header VDO. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                        = CMD_TYPE_RESP_NAK;
                }
                else
                {
                    *((uint8 *)&gl_response_vdo[UVDM_CHEKCSUM_VDO_INDEX])
                        = mem_calculate_byte_checksum (addr, size);
                    /* Increment number of VDOs to be sent by 1. */
                    no_of_vdo++;
                }
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
#else
            response_code = CCG_STAT_INVALID_COMMAND;
#endif /* SECURE_FW_UPDATE */
            break;
            /* Don't support GET_FW_START_ADDRESS command in case of single app bootloader  */
#if (!(CCG_DUALAPP_DISABLE && CCG_BOOT))
            /* Return the start Address of Both FW images. */
        case UVDM_CMD_GET_FW_START_ADDRESS_OPCODE:
            /*
             * Make sure cmd_length is 0 in this case, as no command specific
             * Data object is expected.
             */
            if (!cmd_length)
            {
                gl_response_vdo[UVDM_FW1_START_ADDR_VDO_INDEX].val =
                    sys_get_fw_img1_start_addr();
#if (!CCG_DUALAPP_DISABLE)
                gl_response_vdo[UVDM_FW2_START_ADDR_VDO_INDEX].val =
                    sys_get_fw_img2_start_addr();
#else
                gl_response_vdo[UVDM_FW2_START_ADDR_VDO_INDEX].val =
                    SYS_INVALID_FW_START_ADDR;
#endif /* CCG_DUALAPP_DISABLE */
                no_of_vdo += UVDM_GET_FW_START_ADDR_UVDM_NO_OF_VDO;
            }
            else
            {
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;
#endif /* !(CCG_DUALAPP_DISABLE && CCG_BOOT) */

#if (CCG_BOOT == 0)
            /*
             * This command is used for setting FW application priority value in App
             * Priority Row for debugging purpose.
             */
        case UVDM_CMD_SET_APP_PRIORITY_OPCODE:
#if (APP_PRIORITY_FEATURE_ENABLE == 1u)
            /*
             * Make sure cmd_length is 4 in this case, as just 1 command specific
             * Data object is expected.
             */
            if (cmd_length == UVDM_SET_APP_PRIORITY_CMD_SIZE)
            {
                response_code = flash_set_app_priority(
                        cmd_param[UVDM_SET_APP_PRIORITY_INDEX]);
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
#else
            response_code = CCG_STAT_INVALID_COMMAND;
#endif /* APP_PRIORITY_FEATURE_ENABLE */
            break;
#endif /* CCG_BOOT */

#if (CCG_BOOT == 0)
        case UVDM_CMD_SEND_SIGNATURE_OPCODE:
#if (SECURE_FW_UPDATE == 1)
            /*
             * This is CY UVDM used to collect FW Image Signature and store it in
             * pre-defined Flash location.
             * Sequence number determines the section of Signature.
             * Seq 1: First 24 bytes
             * Seq 2: Next 24 bytes
             * Seq 3: Last 16 bytes
             */
            switch (seq_num)
            {
                case UVDM_SEND_SIGN_SEQUENCE_1:
                case UVDM_SEND_SIGN_SEQUENCE_2:
                    if (cmd_length == UVDM_SEND_SIGN_SEC_1_2_SIZE)
                    {
                        sboot_accumulate_sig (cmd_param, cmd_length,
                                (seq_num - 1) * UVDM_SEND_SIGN_SEC_1_2_SIZE);
                        response_code = CCG_STAT_SUCCESS;
                    }
                    else
                    {
                        response_code = CCG_STAT_INVALID_ARGUMENT;
                    }
                    break;
                case UVDM_SEND_SIGN_SEQUENCE_3:
                    if ((cmd_length == UVDM_SEND_SIGN_SEC_3_SIZE)
                            && (!gl_flash_write_cmd_pending))
                    {
                        sboot_accumulate_sig (cmd_param, cmd_length,
                                (seq_num - 1) * UVDM_SEND_SIGN_SEC_1_2_SIZE);
                        response_code = sboot_program_sig (flash_cb, false, 0x00);
                        /* For Non-Blocking Flash Programming. */
                        if (response_code == CCG_STAT_NO_RESPONSE)
                        {
                            /*
                             * Set the flag to indicate that CCG is handling a FLASH write
                             * command currently.
                             */
                            gl_flash_write_cmd_pending = true;
                            /* Response VDM is pending. */
                            response_state = UVDM_HANDLED_RESPONSE_NOT_READY;
                            gl_cur_nb_cmd = UVDM_CMD_SEND_SIGNATURE_OPCODE;
                        }
                    }
                    else
                    {
                        response_code = CCG_STAT_INVALID_ARGUMENT;
                    }
                    break;
                default:
                    response_code = CCG_STAT_INVALID_ARGUMENT;
                    break;
            }
#else
            response_code = CCG_STAT_INVALID_COMMAND;
#endif /* SECURE_FW_UPDATE */
            break;

        case UVDM_CMD_GET_BOOT_TYPE:
            /*
             * Send back Bootloader type which is stored at a fixed offset in
             * Bootloader flash region.
             */
            if (!cmd_length)
            {
                gl_response_vdo[0x01].val = 0;
                *((uint8_t *)&gl_response_vdo[0x01]) =
                    (*(uint8_t *)(SYS_BOOT_TYPE_FIELD_OFFSET));
                no_of_vdo++;
            }
            else
            {
                /* Set the NAK field in the header VDO. */
                gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
            }
            break;

        case UVDM_CMD_GET_CUSTOMER_INFO:
            /* Respond with Customer specific info based on sequence number. */
            if ((seq_num != UVDM_GET_CUSTOMER_INFO_SEQ_1) &&
                    (seq_num != UVDM_GET_CUSTOMER_INFO_SEQ_2))
            {
                response_code = CCG_STAT_INVALID_ARGUMENT;
            }
            else
            {
                /* Get address of customer info. */
                uint32_t addr = sys_get_custom_info_addr ();
                /* Send first 16 bytes for seq number 1 and next 16 bytes for
                 * sequence number 2. */
                addr = addr + ((seq_num - 1) << 4);
                MEM_COPY ((uint8_t *)&gl_response_vdo[0x01], (uint8_t *)addr,
                        UVDM_GET_CUSTOMER_INFO_RESPONSE_SIZE);
                no_of_vdo += UVDM_GET_CUSTOMER_INFO_RESPONSE_VDO_NUM;
            }
            break;
#endif /* CCG_BOOT */

        default:
            /* Can't recognise VDM */
            response_state = UVDM_NOT_HANDLED;
            break;
    }

    /* Handle response. */
    if (response_state == UVDM_HANDLED_RESPONSE_READY)
    {
        /* Place the cmd opcode in header VDO. */
        gl_response_vdo[UVDM_HEADER_INDEX].ustd_vdm_hdr.cmd = cmd_opcode;

        /*
         * Check if response is not "NO_RESPONSE". In case of NO_RESPONSE,
         * we have to send out the response code in index 1 VDO.
         */
        if (response_code != CCG_STAT_NO_RESPONSE)
        {
            no_of_vdo += 1;
            gl_response_vdo[UVDM_RESPONSE_VDO_INDEX] = (pd_do_t)((uint32_t)(response_code +
                CCG_STATUS_CODE_OFFSET));
        }

        /* Update the pointer to point to Response VDM. */
        *vdm_rspn_pkt = &gl_response_vdo[0];
        /* Update count of VDOs. */
        *vdo_count = no_of_vdo;
    }
    else
    {
        /* No VDM response required. */
        *vdo_count = 0;
    }

    return response_state;
}

#if QC_4_0_ENABLE

#define QC_RESPONSE_ACK             (0xA0)
#define QC_RESPONSE_NACK            (0x50)

#define QC_CHARGER_TYPE             (0x04)  /* Quick Charge 4. */
#define QC_CHARGER_VERSION          (0x30)  /* 3.0 */

uvdm_response_state_t uvdm_qc_4_0_handler(uint8_t port, uint32_t *rx_pkt,
    pd_do_t **vdm_rspn_pkt, uint8_t *vdo_count)
{
    uvdm_response_state_t response_state = UVDM_NOT_HANDLED;
    uint16_t qc_cmd = 0;
    uint16_t volt_connector = 0;
    pd_do_t* d_obj = (pd_do_t *)(&(rx_pkt[1]));
    const dpm_status_t* dpm_stat = dpm_get_info(port);
    *vdo_count = 0;

    /* TODO: Handle UVDMs only if in Source mode. */

    /*
     * Handle QC UVDM only if:
     * 1) SVID is Qualcomm VID
     * 2) QC 4.0 support is enabled in configuration table.
     */
    if (((pd_get_ptr_chg_cfg_tbl(port)->src_sel & BC_SRC_QC_4_0_MODE_ENABLE_MASK) != 0) &&
        (d_obj->ustd_qc_4_0_hdr.svid == QC_4_0_SVID))
    {
        /* Copy the header in response VDO. */
        gl_response_vdo[UVDM_HEADER_INDEX].val = d_obj->val;
        /* Set ACK as default response. */
        gl_response_vdo[UVDM_HEADER_INDEX].ustd_qc_4_0_hdr.cmd_0 = QC_RESPONSE_ACK;
        /* Set Default response VDO count to 2: Header + 1 VDO. */
        *vdo_count = 2;

       /* Get QC Command. */
       qc_cmd = (d_obj->ustd_qc_4_0_hdr.cmd_0 |
            (d_obj->ustd_qc_4_0_hdr.cmd_1 << 8));

       /* As per QC 4.0, we should respond only if this is a valid request. */
       response_state = UVDM_HANDLED_NO_RESPONSE;

        /* Check if UVDM Command has only header VDO and that we are UFP. If not, do not respond. */
        if ((GET_PD_HDR_CNT(rx_pkt[0]) == 1) && (gl_dpm_port_type[port] == PRT_TYPE_UFP))
        {
            /* Assume this to be a valid request by default to optimize. */
            response_state = UVDM_HANDLED_RESPONSE_READY;

            switch (qc_cmd)
            {
                case UVDM_QC_GET_CASE_TEMP:
                    /* Temperature senser not available. Send NAK. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_qc_4_0_hdr.cmd_0 = QC_RESPONSE_NACK;
                    *vdo_count = 1;
                    break;

                case UVDM_QC_GET_CONNECTOR_TEMP:
                    /* Temperature senser not available. Send NAK. */
                    gl_response_vdo[UVDM_HEADER_INDEX].ustd_qc_4_0_hdr.cmd_0 = QC_RESPONSE_NACK;
                    *vdo_count = 1;
                    break;

                case UVDM_QC_GET_CONNECTOR_VOLT:
                    volt_connector = dpm_stat->app_cbk->vbus_get_value(port);
                    /* Convert in 10mV units from mV units. */
                    volt_connector /= 10;
                    gl_response_vdo[1].qc_4_0_data_vdo.data_0 = 0;
                    gl_response_vdo[1].qc_4_0_data_vdo.data_1 = (volt_connector & 0xFF);
                    gl_response_vdo[1].qc_4_0_data_vdo.data_2 = ((volt_connector & 0x0F00) >> 8);
                    gl_response_vdo[1].qc_4_0_data_vdo.data_3 = 0;
                    break;

                case UVDM_QC_GET_CHARGER_TYPE:
                    gl_response_vdo[1].val = 0;
                    gl_response_vdo[1].qc_4_0_data_vdo.data_0 = QC_CHARGER_TYPE;
                    break;

                case UVDM_QC_GET_CHARGER_VERSION:
                    gl_response_vdo[1].val = 0;
                    gl_response_vdo[1].qc_4_0_data_vdo.data_0 = QC_CHARGER_VERSION;
                    break;

                default:
                    response_state = UVDM_HANDLED_NO_RESPONSE;
                    break;
            }
        }

        if (response_state == UVDM_HANDLED_RESPONSE_READY)
        {
            *vdm_rspn_pkt = &gl_response_vdo[0];
        }
    }

    return response_state;
}
#endif /* QC_4_0_ENABLE */

/* [] END OF FILE */
