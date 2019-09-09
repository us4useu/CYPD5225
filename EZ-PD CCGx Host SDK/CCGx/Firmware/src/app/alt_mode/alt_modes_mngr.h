/**
 * @file alt_modes_mngr.h
 *
 * @brief @{Alternate Mode Manager header file.@}
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

#ifndef _ALT_MODES_MNGR_H_
#define _ALT_MODES_MNGR_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <pd.h>
#include <config.h>
#include <vdm_task_mngr.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
  @brief Internal alt modes manager denotation of attached UFP target.
 */
#define ATCH_TGT                        (1u)
/**
  @brief Internal alt modes manager denotation of the attached cable.
 */
#define CABLE                           (2u)
/**
  @brief Internal alt modes manager denotation of non supported alternative mode.
 */
#define MODE_NOT_SUPPORTED              (0xFFu)
/**
  @brief Internal alt modes manager denotation in case if VDO sending not needed.
 */
#define EMPTY_VDO                       (0x00000000u)
/**
  @brief Internal alt modes manager denotation of zero VDO .
 */
#define NONE_VDO                        (0xFFFFFFFFu)
/**
  @brief Maximum number of alternative modes which alt modes manager could operates in the same time.
 */
#define MAX_SUPP_ALT_MODES              (4u)
/**
  @brief Start index of VDO data in VDM message.
 */
#define VDO_START_IDX                   (1u)
/**
  @brief Empty mask.
 */
#define NONE_MODE_MASK                  (0x00000000u)
/**
  @brief All bit set full mask.
 */
#define FULL_MASK                       (0xFFFFFFFFu)
/**
  @brief This mask uses in IS_FLAG_CHECKED macro to check if bit is set in some mask .
 */
#define EN_FLAG_MASK                    (0x00000001u)
/**
  @brief Exit all active modes command.
 */
#define EXIT_ALL_MODES                  (0x7u)
/**
  @brief Mask to find out cable dirrectionality support for cable discovery ID response.
 */
#define CBL_DIR_SUPP_MASK               (0x780u)
/**
  @brief Mask to find out cable USB 2.0 support in USB Superspeed Signaling Support
  field for cable discovery ID response.
 */
#define USB_2_0_SUPP                    (0u)
/**
  @brief Mask to find out cable USB Gen 1 support in USB Superspeed Signaling Support
  field for cable discovery ID response.
 */
#define USB_GEN_1_SUPP                  (1u)
/**
  @brief Mask to find out cable USB Gen 2 support in USB Superspeed Signaling Support
  field for cable discovery ID response.
 */
#define USB_GEN_2_SUPP                  (2u)

/* Events Related to ALT MODES */

/**
  @brief Size in ALT_MODE_EVT_SIZE * 4 bytes of alt mode APP event.
 */
#define ALT_MODE_EVT_SIZE               (2u)
/**
  @brief Index of alt mode APP event.
 */
#define ALT_MODE_EVT_IDX                (0u)
/**
  @brief Index of data for alt mode APP event.
 */
#define ALT_MODE_EVT_DATA_IDX           (1u)
/**
  @brief Internal alt modes manager denotation of object without useful data .
 */
#define NO_DATA                         (0u)
/**
  @brief Maximum number of VDM send retries in case of no CRC response, timeout or BUSY response.
 */
#define MAX_RETRY_CNT                   (9u)
/**
  @brief Set appropriate bit_idx to 1 in the status variable.
 */
#define SET_FLAG(status, bit_idx)       ((status) |= (1 << ((uint32_t)(bit_idx))))
/**
  @brief Set appropriate bit_idx to 0 in the status variable.
 */
#define REMOVE_FLAG(status, bit_idx)    ((status) &= (~(1 << ((uint32_t)(bit_idx)))))
/**
  @brief Return true if bit_idx of the status variable is set to 1.
 */
#define IS_FLAG_CHECKED(status, bit_idx)        (((status) >> (uint32_t)(bit_idx)) & EN_FLAG_MASK)
/**
  @brief Quick access to Standard VDM header fields macros.
 */
#define VDM_HDR                          vdm_header.std_vdm_hdr
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef alt_mode_mngr_state_t
  @brief This enumeration holds all possible Alt modes manager states when DFP.
 */
typedef enum
{
    ALT_MODE_MNGR_STATE_DISC_MODE = 0,  /**< Alt modes manager discovery mode state. */
    ALT_MODE_MNGR_STATE_PROCESS,        /**< Alt modes manager alternative modes processing state. */

}alt_mode_mngr_state_t;

/**
  @typedef alt_mode_state_t
  @brief This enumeration holds all possible states of each alt mode which is handled by Alt modes manager.
 */
typedef enum
{
    ALT_MODE_STATE_DISABLE = 0,         /**< State when alternative mode functionality is disabled. */
    ALT_MODE_STATE_IDLE,                /**< State when alternative mode is idle. */
    ALT_MODE_STATE_INIT,                /**< State when alternative mode initiate its functionality. */
    ALT_MODE_STATE_SEND,                /**< State when alternative mode needs to send VDM message. */
    ALT_MODE_STATE_WAIT_FOR_RESP,       /**< State while alternative mode wait for VDM response. */
    ALT_MODE_STATE_FAIL,                /**< State when alternative mode VDM response fails. */
    ALT_MODE_STATE_RUN,                 /**< State when alternative mode need to be running. */
    ALT_MODE_STATE_EXIT,                /**< State when alternative mode exits and resets all related variables. */

}alt_mode_state_t;

/**
  @typedef alt_mode_app_evt_t
  @brief This enumeration holds all possible application events related to Alt modes handling.
 */
typedef enum
{
    AM_NO_EVT = 0,                      /**< Empty event. */
    AM_EVT_SVID_NOT_FOUND,              /**< Sends to EC if UFP doesn't support any SVID. */
    AM_EVT_ALT_MODE_ENTERED,            /**< Alternative mode entered. */
    AM_EVT_ALT_MODE_EXITED,             /**< Alternative mode exited. */
    AM_EVT_DISC_FINISHED,               /**< Discovery process was finished. */
    AM_EVT_SVID_NOT_SUPP,               /**< CCGx doesn't support received SVID. */
    AM_EVT_SVID_SUPP,                   /**< CCGx supports received SVID. */
    AM_EVT_ALT_MODE_SUPP,               /**< CCGx supports alternate mode. */
    AM_EVT_SOP_RESP_FAILED,             /**< UFP VDM response failed. */
    AM_EVT_CBL_RESP_FAILED,             /**< Cable response failed. */
    AM_EVT_CBL_NOT_SUPP_ALT_MODE,       /**< Cable capabilities couldn't provide alternative mode hanling. */
    AM_EVT_NOT_SUPP_PARTNER_CAP,        /**< CCGx and UFP capabilities not consistent. */
    AM_EVT_DATA_EVT,                    /**< Specific alternative mode event with data. */
#if CCG_UCSI_ENABLE    
    AM_EVT_SUPP_ALT_MODE_CHNG,          /**< Same functionality as ALT_MODE_SUPP; new one for UCSI */    
#endif /*CCG_UCSI_ENABLE*/

}alt_mode_app_evt_t;

/**
  @typedef alt_mode_app_cmd_t
  @brief This enumeration holds all possible APP command related to Alt modes handling.
 */
typedef enum
{
    AM_NO_CMD = 0,                      /**< Empty command. */
    AM_SET_TRIGGER_MASK,                /**< Sets trigger mask to prevent auto-entering of the selected alternative modes. */
    AM_CMD_ENTER = 3,                   /**< Enter to selected alternative mode. */
    AM_CMD_EXIT,                        /**< Exit from selected alternative mode. */
    AM_CMD_SPEC,                        /**< Specific alternative EC mode command with data. */
#if CCG_PD_REV3_ENABLE
    AM_CMD_RUN_UFP_DISC                 /**< Runs Discover command when CCG is UFP due to PD 3.0 spec . */
#endif /* CCG_PD_REV3_ENABLE */

}alt_mode_app_cmd_t;

/**
 * @typedef fail_status_t
 * @brief Enum of the VDM failure response status.
 */
typedef enum
{
    BUSY = 0,                           /**<  Target is BUSY.  */
    GOOD_CRC_NOT_RSVD,                  /**<  Good CRC wasn't received.  */
    TIMEOUT,                            /**<  No response or corrupted packet.  */
    NACK                                /**<  Sent VDM NACKed.  */
} fail_status_t;


/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
  @union alt_mode_evt_t
  @brief Alt modes manager application event/command structure.
 */
typedef union
{
    uint32_t val;                        /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing alternate modes manager event/command . */
    struct ALT_MODE_EVT
    {
        uint32_t data_role       : 1;    /**< Current event sender data role. */
        uint32_t alt_mode_evt    : 7;    /**< Alt mode event index from alt_mode_app_evt_t structure. */
        uint32_t alt_mode        : 8;    /**< Alt mode ID. */
        uint32_t svid            : 16;   /**< Alt mode related SVID. */
    }alt_mode_event;                     /**< Union containing the alt mode event value. */

    /** @brief Struct containing alternate modes manager event/command data. */
    struct ALT_MODE_EVT_DATA
    {
        uint32_t evt_data        : 24;   /**< Alt mode event's data. */
        uint32_t evt_type        : 8;    /**< Alt mode event's type. */
    }alt_mode_event_data;                /**< Union containing the alt mode event's data value. */

}alt_mode_evt_t;

/**
 * @brief This type of the function is used by alt modes manager to communicate with
 * any of supported alt modes.
 *
 * @param port Port index the function is performed for .
 *
 * @return None.
 */
typedef void
(*alt_mode_cbk_t) (
        uint8_t         port);

/**
 * @brief This type of the function is used by alt modes manager to run alternative
 * mode analisys of received APP command .
 *
 * @param port Port index the function is performed for.
 * @param hpi_cmd Received APP command data.
 *
 * @return true if APP command passed successful, false if APP command is invalid
 * or contain unacceptable fields.
 */
typedef bool
(*alt_mode_app_cbk_t) (
        uint8_t         port,
        alt_mode_evt_t  app_cmd);

/**
  @struct comp_tbl_t
  @brief This structure are used in alt_modes_config.h file to set alternative
  modes compatibility and priority.
 */
typedef struct
{
    uint16_t svid;                          /**< Alternative mode SVID. */
    uint8_t alt_mode_id;                    /**< Alternative mode ID. */

}comp_tbl_t;

/**
  @struct alt_mode_reg_info_t
  @brief This structure holds all necessary information on Discovery Mode stage
  for supported alternative mode when alt modes manager registers new alt mode.
 */
typedef struct
{
    uint8_t atch_type;                      /**< Target of disc svid (cable or device/ama). */
    uint8_t data_role;                      /**< Current data role. */
    uint8_t alt_mode_id;                    /**< Alt mode ID. */
    sop_t cbl_sop_flag;                     /**< Sop indication flag. */
    pd_do_t svid_emca_vdo;                  /**< SVID VDO from cable Discovery mode response. */
    pd_do_t svid_vdo;                       /**< SVID VDO from Discovery mode SOP response. */
    const atch_tgt_info_t* atch_tgt_info;   /**< Attached trgets info (dev/ama/cbl) from Discovery ID response. */
    alt_mode_app_evt_t app_evt;             /**< APP event. */
} alt_mode_reg_info_t;

/**
  @struct alt_mode_info_t
  @brief This structure holds all necessary information  for interaction between
  alt modes manager and selected alternative mode.
 */
typedef struct
{
    alt_mode_state_t mode_state;                /**< Alternate mode state. */
    alt_mode_state_t sop_state[SOP_DPRIME + 1]; /**< VDM state for SOP/SOP'/SOP" packets. */
    uint8_t vdo_max_numb;                       /**< Maximum number of VDO that alt mode can handle */
    uint8_t obj_pos;                            /**< Alternate mode object position. */
    uint8_t cbl_obj_pos;                        /**< Cabel object position. */
    uint8_t alt_mode_id;                        /**< Alternative mode ID. */
    pd_do_t vdm_header;                         /**< Buffer to hold VDM header. */
    pd_do_t* vdo[SOP_DPRIME + 1];               /**< Pointers array to alt mode VDO buffers */
    uint8_t vdo_numb[SOP_DPRIME + 1];           /**< Current number of VDOs used for processing in VDO buffers */
    alt_mode_cbk_t cbk;                         /**< Alternate mode callback function. */
    bool is_active;                             /**< Active mode flag. */
    bool custom_att_obj_pos;                    /**< Object position field in Att VDM used by alt mode as custom. */
    bool uvdm_supp;                             /**< Flag to indicate if alt mode support unstructured VDMs. */
    bool set_mux_isolate;                       /**< Flag to indicate if MUX should be set to isolate while ENTER/EXIT alt mode */
    /* Application control information */
    bool app_evt_needed;                        /**< APP event flag. */
    alt_mode_evt_t app_evt_data;                /**< APP event data. */
    alt_mode_app_cbk_t eval_app_cmd;            /**< APP command cbk. */

}alt_mode_info_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function register pointers to attached dev/ama/cable info and run
 * alt modes manager if success.
 *
 * @param port Port index the function is performed for.
 * @param atch_tgt_info Pointer to struct which holds discovery info about attached targets.
 * @param vdm_msg_info Pointer to struct which holds info of received/sent VDM
 *
 * @return VDM manager task VDM_TASK_ALT_MODE if CCG support any alternative
 * mode. If CCG doesn't support alternative modes function returns VDM_TASK_EXIT.
 */
vdm_task_t reg_alt_mode_mngr(uint8_t port, atch_tgt_info_t* atch_tgt_info, vdm_msg_info_t* vdm_msg_info);

/**
 * @brief This function uses by DFP VDM manager to run alt modes manager processing.
 *
 * @param port Port index the function is performed for.
 * @param vdm_evt Current DFP VDM manager event.
 *
 * @return DFP VDM manager task based on alt modes manager processing results.
 */
vdm_task_t vdm_task_mngr_alt_mode_process(uint8_t port, vdm_evt_t vdm_evt);

/**
 * @brief This function run received VDM analisys if CCG is UFP
 *
 * @param port Port index the function is performed for.
 * @param vdm Pointer to pd packet which contains received VDM.
 *
 * @return true if received VDM could handled successful and VDM response need
 * to be sent with ACK. If received VDM should be NACKed then returns false
 */
bool eval_rec_vdm(uint8_t port, const pd_packet_t *vdm);

/**
 * @brief Fill alt mode APP event fields with appropriate values.
 *
 * @param port Port index the event is performed for.
 * @param svid SVID of alternative mode which event refers to.
 * @param am_idx Index of alternative mode in compatibility table which event refers to.
 * @param evt Alternative mode APP event.
 * @param data Alternative mode APP event data.
 *
 * @return pointer to the event related data.
 */
const uint32_t* form_alt_mode_event(uint8_t port, uint16_t svid, uint8_t am_idx, alt_mode_app_evt_t evt, uint32_t data);

/**
 * @brief This function analises, parses and run alternative mode analisys function
 * if received command is specific alt mode command.
 *
 * @param port Port index the function is performed for.
 * @param cmd Pointer to received alt mode APP command.
 * @param data Pointer to received alt mode APP command additional data.
 *
 * @return true if APP command passed successful, false if APP command is invalid
 * or contain unacceptable fields.
 */
bool eval_app_alt_mode_cmd(uint8_t port, uint8_t *cmd, uint8_t *data);

/**
 * @brief Check whether the DFP VDM manager for the selected port is idle.
 *
 * @param port Port index the function is performed for.
 *
 * @return true if manager is busy, false - if idle.
 */
bool is_alt_mode_mngr_idle(uint8_t port);

/**
 * @brief Prepare the alt modes manager for device deep sleep.
 *
 * @param port Index of USB-PD port to be prepared for sleep.
 *
 * @return None
 */
void alt_mode_mngr_sleep(uint8_t port);

/**
 * @brief Restore the  alt modes manager state after waking from deep sleep.
 *
 * @param port Index of USB-PD port to be restored.
 *
 * @return None
 */
void alt_mode_mngr_wakeup(uint8_t port);

/**
 * @brief Resets alternative mode command info structure.
 *
 * @param info Pointer to alternative mode info structure.
 *
 * @return None.
 */
void reset_alt_mode_info(alt_mode_info_t* info);

/**
 * @brief Returns pointer to VDM buffer.
 *
 * @param port Index of Type-C port.
 *
 * @return Pointer to VDM buffer.
 */
dpm_pd_cmd_buf_t* get_vdm_buff(uint8_t port);

/**
 * @brief Check for presence of alt modes for given svid in alt modes compatibility table.
 *
 * @param svid Received SVID.
 * @param port Index of Type-C port.
 *
 * @return True if SVID is supported by CCG.
 */
uint8_t is_svid_supported(uint16_t svid, uint8_t port);

/**
 * @brief Retrieve the current alternate mode status for a PD port.
 *
 * @param port PD port to be queried.
 * @return The alternate mode status.
 */
uint8_t alt_mode_get_status(uint8_t port);

/**
 * @brief Exit all active alternate modes.
 * @param port PD port index.
 * @return None
 */
void alt_mode_mngr_exit_all(uint8_t port);

/**
 * @brief Get number of the alternate modes for choosen port.
 * @param port PD port index.
 * @return Number of the alternates modes for the choosen port and current data role.
 */
uint8_t get_alt_mode_numb(uint8_t port);


#if ( CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED )

/* Returns alt mode index for given svid */
 uint8_t get_alt_mode_svid_idx(uint8_t port, uint16_t svid);

/* Resets alt mode mngr info */
void reset_mngr_info(uint8_t port);

/* Check for presence of any alt modes in alt modes compatibility table*/
uint8_t is_alt_modes_available(uint8_t port);

/* Returns alt mode index in compatibility table */
uint8_t get_alt_mode_tbl_idx(uint8_t port, uint16_t svid, uint8_t id);

/* Fast access functions */

/* Get alt mode info */
alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx);

/* Get alternate mode's state */
alt_mode_state_t get_alt_mode_state(alt_mode_info_t* alt_mode_info_ptr);

/* Get VDM information of that alt mode */
vdm_msg_info_t* get_vdm_info(uint8_t port);

/* Get the attached target info */
const atch_tgt_info_t* get_atch(uint8_t port);

/* Get the alternate mode's registered ID */
uint32_t get_alt_mode_id(uint8_t port);


/* Get info from the compatibility table */
const comp_tbl_t get_tbl_info(uint8_t port, uint8_t idx);

/* Get info from the mode table */
uint32_t get_tbl_mode_info(uint8_t port, uint8_t idx);

/**
 * @brief This function set the alt mode state for given port.
 *
 * @param port Index of Type-C port.
 * @param alt mode Index of Type-C port.
 *
 * @return True if SVID is supported by CCG.
 */
void set_alt_mode_state(uint8_t port_idx, uint8_t alt_mode_idx);

/* Composes alt mode info to vdm_msg_info_t struct before sending */
uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type);

/* Parces received VDM info and moves it to alt mode info struct */
void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type);

/* Get alternate mode info structure */
alt_mode_reg_info_t get_alt_mode_reg_info(uint8_t port);

/* Check if the mode is active */
uint8_t is_alt_mode_active(uint8_t port, uint8_t alt_mode_idx);

/* Start the alternate mode */
void init_alt_mode(uint8_t port_idx, uint8_t alt_mode_idx);

/* Get a bitmap of supported alt modes, given the current connection */
uint32_t get_supp_alt_modes(uint8_t port);


#endif /*CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED*/

#endif /* _ALT_MODES_MNGR_H_ */

/* [] END OF FILE */

