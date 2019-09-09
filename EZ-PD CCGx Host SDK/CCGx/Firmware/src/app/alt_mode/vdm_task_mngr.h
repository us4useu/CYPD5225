/**
 * @file vdm_task_mngr.h
 *
 * @brief @{VDM task manager header file.@}
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

#ifndef _VDM_TASK_MNGR_H_
#define _VDM_TASK_MNGR_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <pd.h>
#include <config.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
  @brief Start index of received VDM packet in case of response to DISC SVID command .
 */
#define PD_SVID_ID_HDR_VDO_START_IDX        (4u)

/**
  @brief Index of AMA VDO in DISCOVER_ID response from port partner (after skipping the
  VDM header).
 */
#define PD_DISC_ID_AMA_VDO_IDX              (3u)

/**
  @brief Maximum number of attached target SVIDs VDM task manager can hold in the memory.
 */
#if SAVE_SUPP_SVID_ONLY
#define MAX_SVID_VDO_SUPP                   (4u)
#else
#define MAX_SVID_VDO_SUPP                   (32u)
#endif

/**
  @brief Maximum number of cable SVIDs VDM task manager can hold in the memory.
 */
#define MAX_CABLE_SVID_SUPP                 (4u)
/**
  @brief Standard vendor ID allocated to USB PD specification.
 */
#define STD_SVID                            (0xFF00u)

/**
  @brief Maximum number of DISCOVER_SVID attempts that will be performed.
 */
#define MAX_DISC_SVID_COUNT                 (10u)

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef vdm_task_t
  @brief   VDM manager tasks definition.

  This enumeration lists the various VDM mngr tasks to handle VDMs.
 */
typedef enum
{
    VDM_TASK_WAIT = 0,           /**< DFP manager wait task while waiting for VDM response. */
    VDM_TASK_INIT,               /**< This task is responsible for inializing of  VDM manager. */
    VDM_TASK_DISC_ID,            /**< This task is responsible for VDM Discovery ID flow. */
    VDM_TASK_DISC_SVID,          /**< This task is responsible for VDM Discovery SVID flow. */
    VDM_TASK_REG_ATCH_TGT_INFO,  /**< This task is responsible for registering of Discovery result
                                      information in alt mode manager. */
    VDM_TASK_EXIT,               /**< This task deinits  VDM task manager. */
    VDM_TASK_SEND_MSG,           /**< This task is responsible for forming and sending VDM message . */
    VDM_TASK_ALT_MODE,           /**< This task is responsible for running of alt mode manager . */

}vdm_task_t;

/**
  @typedef vdm_evt_t
  @brief   VDM manager events definition.

  This enumeration lists the various VDM mngr events to handle VDMs.
 */
typedef enum
{
    VDM_EVT_RUN = 0,             /**< This event is responsible for running any of DFP VDM manager task . */
    VDM_EVT_EVAL,                /**< This event is responsible for eveluating VDM response . */
    VDM_EVT_FAIL,                /**< This event notifies task manager task if VDM rersponse fails . */
    VDM_EVT_EXIT,                /**< This event runs exiting from VDM task manager task . */

}vdm_evt_t;

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
  @struct atch_tgt_info_t
  @brief   Struct holds Discovery info for alt modes manager.

  This struct holds Discovery information which uses by alt modes manager.
 */
typedef struct
{
    pd_do_t tgt_id_header;                  /**< Holds dev/ama discovery ID header . */
    pd_do_t ama_vdo;                        /**< Holds AMA discovery ID response VDO . */
    const pd_do_t* cbl_vdo;                 /**< Pointer to cable VDO . */
    uint16_t tgt_svid[MAX_SVID_VDO_SUPP];   /**< Holds received SVID for dev/ama . */
    uint16_t cbl_svid[MAX_SVID_VDO_SUPP];   /**< Holds received SVID for cable . */

}atch_tgt_info_t;

/**
  @struct vdm_msg_info_t
  @brief   Struct holds received/sent VDM info.

  This struct holds received/sent VDM information which uses by VDM alternative modes managers.
 */
typedef struct
{
    pd_do_t vdm_header;                /**< Holds VDM buffer. */
    uint8_t sop_type;                   /**< VDM SOP type. */
    uint8_t vdo_numb;                   /**< Number of received VDOs in VDM. */
    pd_do_t vdo[MAX_NO_OF_VDO];         /**< VDO objects buffer. */

}vdm_msg_info_t;

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/

/**
 * @brief Enables VDM task mngr functionality.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
void enable_vdm_task_mngr(uint8_t port);

/**
 * @brief Main VDM task mngr function.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
void vdm_task_mngr(uint8_t port);

/**
 * @brief This function deinits VDM task manager and resets all related variables.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
void vdm_task_mngr_deinit(uint8_t port);

/**
 * @brief Check whether the VDM task for the port is idle.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return None.
 */
bool is_vdm_task_idle(uint8_t port);

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP))
/**
 * @brief Starts Discover process when CCG is UFP due to PD 3.0 spec.
 *
 * @param port PD port corresponding to the VDM task manager.
 *
 * @return true if Discover process has started, false if VDM manager id busy. 
 */
bool is_ufp_disc_started(uint8_t port);
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) */

/**
 * @brief Obtain the last DISC_ID response received by the CCG device from a port partner.
 * @param port PD port index.
 * @param resp_len_p Length of response in PD data objects.
 * @return Pointer to the actual response data.
 */
uint8_t *vdm_get_disc_id_resp(uint8_t port, uint8_t *resp_len_p);

/**
 * @brief Obtain the collective DISC_SVID response received by the CCG device from a port partner.
 * @param port PD port index.
 * @param resp_len_p Length of response in PD data objects.
 * @return Pointer to the actual response data.
 */
uint8_t *vdm_get_disc_svid_resp(uint8_t port, uint8_t *resp_len_p);

#endif /* _VDM_TASK_MNGR_H_ */

/* [] END OF FILE */

