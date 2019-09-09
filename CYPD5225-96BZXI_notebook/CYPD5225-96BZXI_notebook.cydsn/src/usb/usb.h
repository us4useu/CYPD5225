/**
 * @file usb.h
 *
 * @brief @{USB driver header file for CCG3 & DMC (Dock Management Controller).@}
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

#ifndef _USB_H_
#define _USB_H_

#include "stdint.h"
#include "stdbool.h"
#include "config.h"
#include "status.h"
#include "usbconst.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/**
 * @brief Number of non-control endpoints supported by the device.
 * The value is hardware dependant and should not be changed.
 */
#define USB_NUM_EP                              (8)

/**
 * @brief Largest packet size supported by the device. This is defined by
 * the firmware and should not be modified.
 */
#define USB_EP_MAX_PKT_SIZE                     (64)

/**
 * @brief The LS nibble is the buffer size index for IN endpoints and the
 * MS nibble is the buffer size inded for OUT endpoints.
 */
#define USB_BUF_SIZE_64BYTE                     (0x66)

/**
 * @brief Shared USB memory available for EP data. This is  hardware defined
 * and should not be changed.
 */
#define USB_EP_HW_BUFFER_SIZE                   (0x200)

/**
 * @brief Allowed EP0 size. The value is hardware dependant and
 * should not be changed.
 */
#define USB_EP0_SIZE                            (8)

/**
 * @brief Maximum number of configurations supported. This is always 1.
 */
#define USB_NUM_CONFIGURATION                   (1)

/**
 * @brief Maximum number of alternate interfaces supported.
 */
#define USB_NUM_ALT_INTERFACE                   (1)

/**
 * @brief Maximum number of interfaces supported.
 */
#define USB_NUM_INTERFACE                       (4)

/**
 * @brief Instance of the timer module to be used for the USB module. Since only
 * one port is supported, the instance is always 0.
 */
#define USB_TIMER_INSTANCE                      (0)

/**
 * @brief Timer ID allocation range for USB module is from 144 to 151.
 */
#define USB_TIMER_ID_BASE                        (144u)

/**
 * @brief This timer is used to check for USB bus suspend condition.
 */
#define USB_SUSPEND_TIMER                       (USB_TIMER_ID_BASE)
/**
 * @brief USB bus suspend condition detection timeout in ms.
 *
 * If no SOF is received within 3ms, the USB bus is defined to be in suspend mode.
 * But the timing from USB 2.0 specification is being relaxed here to accomodate
 * for hosts which are not able to send SOFs within 3ms after resume.
 */
#define USB_SUSPEND_TIMER_PERIOD                (10)

/**
 * @brief This timer is used to signal remote wakeup.
 */
#define USB_REMOTE_WAKEUP_TIMER                 (145u)
/**
 * @brief USB bus remote wakeup signalling duration in ms.
 *
 * A remote wakeup is indicated by signalling K-signal on the USB 2.0 bus for
 * 15ms. This period counts the time the signal should be driven.
 */
#define USB_REMOTE_WAKEUP_TIMER_PERIOD          (15)

/**
 * @brief Voltage threshold in mV for block VBUS input. 
 * For CCG3 & Dock Management Controller (DMC) this is the 
 * voltage at the VDDD line. Any voltage above this threshold, requires the
 * internal regulator to be turned on. This value is dependant on the hardware
 * block and should not be changed.
 */
#define USB_VBUS_REG_EN_THRESHOLD               (3700)

/*********************** Register definitions ********************************/

/**
 * Non-control endpoint count register
 * The Endpoint Count Register 0 (CNT0) is used for configuring endpoints
 * one through eight.
 */
#define USBDEV_SIE_EPx_CNT0_ADDRESS             (USBDEV_SIE_EP1_CNT0_ADDRESS)
#define USBDEV_SIE_EPx_CNT0(m)                  (*(volatile uint32_t *)(USBDEV_SIE_EP1_CNT0_ADDRESS + ((m) * 0x40)))
#define USBDEV_SIE_EPx_CNT0_DEFAULT             (0x00000000)

/**
 * These bits are the 3 MSb bits of an 11-bit counter. The LSb are the Data
 * Count[7:0] bits of the CNT1 register. Refer to the CNT1 register for more
 * information.
 */
#define USBDEV_SIE_EPx_CNT0_DATA_COUNT_MSB_MASK (0x00000007) /* <0:2> RW:RW:0: */
#define USBDEV_SIE_EPx_CNT0_DATA_COUNT_MSB_POS  (0)

/**
 * This bit is used for OUT transactions only and is read only. It is cleared
 * to '0' if CRC bit stuffing errors or PID errors occur. This bit does not
 * update for some endpoint mode settings.
 */
#define USBDEV_SIE_EPx_CNT0_DATA_VALID          (1u << 6) /* <6:6> RW1S:RW0C:0: */

/**
 * This bit selects the DATA packet's toggle state. For IN transactions firmware
 * must set this bit to the expected state. For OUT transactions the hardware
 * sets this bit to the state of the received Data Toggle bit.
 */
#define USBDEV_SIE_EPx_CNT0_DATA_TOGGLE         (1u << 7) /* <7:7> RW:RW:0: */

/**
 * The Endpoint Count Register 1 (CNT1) sets or reports the number of bytes
 * in a USB data transfer to the non-control endpoints. For IN transactions
 * firmware loads the count with the number of data bytes to transmit to
 * the host. Valid values for MODE 1 and MODE 2 are 0 to 514 and for MODE
 * 3 it is 0 to 1025. For an OUT transaction the full 11-bit count is updated
 * by the SIE to the actual number of data bytes received by the SIE plus
 * two for the packet's CRC. The bytes (Data + CRC) received both the data
 * from the USB packet and the 2-byte CRC are written to the USB's dedicated
 * SRAM. Valid values for MODE 1 and MODE 2 are 2 to 514 and for MODE 3 it
 * is 2 to 1025. To get the actual number of bytes received firmware needs
 * to decrement the 11-bit count by two.
 */
#define USBDEV_SIE_EPx_CNT1_ADDRESS             (USBDEV_SIE_EP1_CNT1_ADDRESS)
#define USBDEV_SIE_EPx_CNT1                     (*(volatile uint32_t *)(USBDEV_SIE_EP1_CNT1_ADDRESS + ((m) * 0x40)))
#define USBDEV_SIE_EPx_CNT1_DEFAULT             (0x00000000)

/**
 * These bits are the 8 LSb of a 11-bit counter.  The 3 MSb bits are in the
 * CNT0 register. The 11-bit count indicates the number of data bytes in
 * a transaction.
 */
#define USBDEV_SIE_EPx_CNT1_DATA_COUNT_MASK     (0x000000ff) /* <0:7> RW:RW:0: */
#define USBDEV_SIE_EPx_CNT1_DATA_COUNT_POS      (0)

/**
 * Non-control endpoint's control Register
 * The Endpoint Control Register 0 (CR0) is used for status and configuration
 * of the non-control endpoints .
 */
#define USBDEV_SIE_EPx_CR0_ADDRESS              (USBDEV_SIE_EPx_CR0_ADDRESS)
#define USBDEV_SIE_EPx_CR0                      (*(volatile uint32_t *)(USBDEV_SIE_EPx_CR0_ADDRESS))
#define USBDEV_SIE_EPx_CR0_DEFAULT              (0x00000000)

/**
 * The mode controls how the USB SIE responds to traffic and how the USB
 * SIE changes the mode of that endpoint as a result of host packets to the
 * endpoint.
 */
#define USBDEV_SIE_EPx_CR0_MODE_MASK            (0x0000000f) /* <0:3> RW:RW:0: */
#define USBDEV_SIE_EPx_CR0_MODE_POS             (0)

/**
 * The ACK'd transaction bit is set whenever the SIE engages in a transaction
 * to the register's endpoint that completes with an ACK packet. This bit
 * is cleared by any writes to the register.
 */
#define USBDEV_SIE_EPx_CR0_ACKED_TXN             (1u << 4) /* <4:4> RW1S:RWC:0: */

/**
 * When set this bit causes an endpoint interrupt to be generated even when
 * a transfer completes with a NAK.
 */
#define USBDEV_SIE_EPx_CR0_NAK_INT_EN            (1u << 5) /* <5:5> R:RW:0: */

/**
 * The Error in transaction bit is set whenever an error is detected. For
 * an IN transaction, this indicates a no response from HOST scenario. For
 * an OUT transaction, this represents an RxErr (PID
 * error/ CRC error/ bit-stuff error scenario). This bit is cleared by any
 * writes to the register.
 */
#define USBDEV_SIE_EPx_CR0_ERR_IN_TXN           (1u << 6) /* <6:6> RW1S:RWC:0: */

/**
 * When this bit is set the SIE stalls an OUT packet if the Mode bits are
 * set to ACK-OUT. The SIE stalls an IN packet if the mode bits are set to
 * ACK-IN. This bit must be clear for all other modes.
 */
#define USBDEV_SIE_EPx_CR0_STALL                (1u << 7) /* <7:7> R:RW:0: */

/**
 * Indication that Endpoint Packet Data is Ready in Main memory
 */
#define USBDEV_ARB_EPx_CFG_IN_DATA_RDY          (1u << 0) /* <0:0> R:RW:0: */

/**
 * Manual DMA Request for a particular (1 to 8) endpoint; changing this field
 * from 0 to 1 causes a DMA request to be generated.
 */
#define USBDEV_ARB_EPx_CFG_DMA_REQ              (1u << 1) /* <1:1> R:RW:0: */

/**
 * Configuration Setting to prevent CRC bytes from being written to memory
 * and being read by firmware
 */
#define USBDEV_ARB_EPx_CFG_CRC_BYPASS           (1u << 2) /* <2:2> R:RW:0: */

/**
 * Configuration Setting to Reset the RA and WA Pointers to their start values
 * at the End of Packet transaction.
 */
#define USBDEV_ARB_EPx_CFG_RESET_PTR            (1u << 3) /* <3:3> R:RW:0: */

/**
  Endpoint Data Register
  Data Register for Endpoint
  */
#define USBDEV_ARB_RWx_DR_ADDRESS(m)    (USBDEV_ARB_RW1_DR_ADDRESS + ((m) * (0x40)))
#define USBDEV_ARB_RWx_DR(m)            (*(volatile uint32_t *)(USBDEV_ARB_RW1_DR_ADDRESS + ((m) * 0x40)))
#define USBDEV_ARB_RWx_DR_DEFAULT       (0x00000000)

/**
  Endpoint Data Register
  Data Register for Endpoint.  Note: ARB_RWx_DR16 and ARB_RWx_DR cannot
  both be used to access the same data packet.
  */
#define USBDEV_ARB_RWx_DR16_ADDRESS(m)  (USBDEV_ARB_RW1_DR16_ADDRESS + ((m) * (0x40)))
#define USBDEV_ARB_RWx_DR16(m)          (*(volatile uint32_t *)(USBDEV_ARB_RW1_DR16_ADDRESS + ((m) * 0x40)))
#define USBDEV_ARB_RWx_DR16_DEFAULT     (0x00000000)

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/

/**
 * @brief USB device mode states.
 */
typedef enum
{
    USB_STATE_DEINITED = 0,     /**< Un-initialized state. */
    USB_STATE_DISABLED,         /**< USB module is disabled. */
    USB_STATE_WAIT_FOR_VBUS,    /**< Waiting for connection. */
    USB_STATE_CONNECTED,        /**< Connected to an external USB host. */
    USB_STATE_RESET,            /**< When the device has received a reset. */
    USB_STATE_ADDRESSED,        /**< Device address has been set. */
    USB_STATE_CONFIGURED,       /**< A valid device configuration is selected. */
    USB_STATE_SUSPENDED,        /**< USB is in suspended condition. */
    USB_STATE_RESUMED,          /**< Virtual state for the sake of callback. */
    USB_STATE_UNCONFIGURED,     /**< Virtual state for the sake of callback. */

} usb_state_t;

/**
 * @brief USB EP0 states.
 */
typedef enum
{
    USB_EP0_STATE_DISABLED = 0, /**< EP0 is disabled. */
    USB_EP0_STATE_SETUP,        /**< EP0 in setup phase. */
    USB_EP0_STATE_DATA_IN,      /**< EP0 in data IN phase. */
    USB_EP0_STATE_DATA_OUT,     /**< EP0 in data OUT phase. */
    USB_EP0_STATE_STATUS_IN,    /**< EP0 in status IN ZLP phase. */
    USB_EP0_STATE_STATUS_OUT,   /**< EP0 in status OUT ZLP phase. */
    USB_EP0_STATE_STALL         /**< EP0 in stall phase. */

} usb_ep0_state_t;

/**
 * @brief Endpoint identifier
 *
 * Endpoint number is converted to zero indexed. EP1 is treated as 0.
 * This avoids the substraction requirement to create the indexed value.
 */
typedef enum
{
    USB_EP_INDEX_EP1,           /**< EP1 identifier */
    USB_EP_INDEX_EP2,           /**< EP2 identifier */
    USB_EP_INDEX_EP3,           /**< EP3 identifier */
    USB_EP_INDEX_EP4,           /**< EP4 identifier */
    USB_EP_INDEX_EP5,           /**< EP5 identifier */
    USB_EP_INDEX_EP6,           /**< EP6 identifier */
    USB_EP_INDEX_EP7,           /**< EP7 identifier */
    USB_EP_INDEX_EP8            /**< EP8 identifier */

} usb_ep_index_t;

/**
 * @brief Setup callback function pointer.
 *
 * Various types of setup function callback pointers can be registered.
 * The following data type defines the function prototype for the setup
 * callback function. The callback function is expected to return error
 * codes as to how the request has handled. If the status is SUCCESS,
 * the USB module stops handling the request internally. If the status is
 * NOT_SUPPORTED, then the module shall try to handle the request internally.
 * If any other error is returned, it shall stall EP0.
 *
 * Return value expectation:
 *      CCG_STAT_SUCCESS = The setup packet is successfully handled.
 *      CCG_STAT_NOT_SUPPORTED = The setup packet was not handled.
 *      Any other value is treated as error and handled accordingly.
 */
typedef ccg_status_t
(*usb_setup_cb_t) (
        usb_setup_pkt_t *setup_pkt /**< Pointer to the setup packet information. */
        );

/**
 * @brief Event callback function pointer.
 *
 * The callback shall be invoked by the USB module if any USB state change occurs.
 */
typedef void
(*usb_event_cb_t) (
        usb_state_t state,      /**< Current USB state. */
        uint32_t data           /**< Data parameter associated with the state. */
        );

/**
 * @brief USB device mode configuration information.
 *
 * The structure holds the USB device mode configuration paramters.
 */
typedef struct usb_config
{
    bool bus_power;             /**< Whether the device is bus powered or self powered. */
    usb_event_cb_t event_cb;    /**< Callback function to be invoked when any
                                  USB events happen. */
    usb_setup_cb_t get_dscr_cb; /**< Callback function to be invoked when a
                                  GET_DESCRIPTOR request is received. */
    usb_setup_cb_t class_rqt_cb;
                                /**< Callback function to be invoked when a
                                  class request is received. */
    usb_setup_cb_t vendor_rqt_cb;
                                /**< Callback function to be invoked when a
                                  vendor request is received. */
    usb_setup_cb_t fallback_rqt_cb;
                                /**< Callback function to be invoked when a
                                  standard request unknown to the stack
                                  is received. */

} usb_config_t;

/**
 * @brief USB endpoint handle.
 *
 * The structure holds information about the USB endpoint. The struture is for
 * internal use and no additional instance of the structure is expected
 * to be created.
 */
typedef struct
{
    bool is_out;                /**< Direction of endpoint. */
    bool toggle;                /**< Active data toggle. */
    bool enabled;               /**< Whether the endpoint is enabled. */

} usb_ep_handle_t;

/**
 * @brief USB device controller handle.
 *
 * The structure holds information about the USB device controller.
 * The struture is for internal use and no additional instance of the
 * structure is expected to be created.
 */
typedef struct
{
    uint16_t dev_stat;          /**< Device status to be returned during GET_STATUS call. */
    usb_state_t state;          /**< Current USB state. */
    usb_state_t prev_state;     /**< Previous USB state. */
    uint32_t int_event;         /**< Internal interrupt event state. */
    usb_config_t cfg;           /**< USB configuration information. */
    bool suspend_check;         /**< Internal flag indicating whether to check for suspend. */
    uint32_t sof_data;          /**< Internal SOF check data to identify a USB bus suspend. */

    usb_setup_pkt_t setup_pkt;  /**< Current USB setup packet. */
    usb_setup_cb_t ep0_xfer_cb; /**< Transfer completion callback. */
    usb_ep0_state_t ep0_state;  /**< Current EP0 transfer state. */
    bool ep0_toggle;            /**< Current EP0 toggle state. */
    uint8_t *ep0_buffer;        /**< Current EP0 data transfer buffer. */
    uint16_t ep0_length;        /**< Pending transfer size for EP0. */
    bool ep0_zlp_rqd;           /**< Whether an ZLP need to be sent on EP0 to indicate
                                  a short transfer. */
    bool ep0_last;              /**< Wether the current EP0 read / write request is the last. */
    uint8_t active_cfg;         /**< Current configuration value set by SET_CONFIG. */
    uint8_t active_alt_inf[USB_NUM_INTERFACE];
    /**< Current alternate interface selected by SET_INTERFACE. */
    usb_ep_handle_t ep_handle[USB_NUM_EP];
    /**< Endpoint handle for internal state machine. */

} usb_handle_t;

/*********************** Register definitions ********************************/

/**
 * USB endpoint modes
 * The enumeration lists all the possible register configuration for endpoints
 * states. This list is dependant on the hardware and should not be changed.
 */
typedef enum
{
    USBDEV_EP_MODE_DISABLE,             /* Ignore all USB traffic to this EP */
    USBDEV_EP_MODE_NAK_INOUT,           /* SETUP: Accept, IN: NAK, OUT: NAK */
    USBDEV_EP_MODE_STATUS_OUT_ONLY,     /* SETUP: Accept, IN: STALL, OUT: ACK 0B, NAK others */
    USBDEV_EP_MODE_STALL_INOUT,         /* SETUP: Accept, IN: STALL, OUT: STALL */
    USBDEV_EP_MODE_ISO_OUT = 5,         /* SETUP: Ignore, IN: Ignore, OUT: Accept ISO */
    USBDEV_EP_MODE_STATUS_IN_ONLY,      /* SETUP: Accept, IN: Respond 0B, OUT: STALL */
    USBDEV_EP_MODE_ISO_IN,              /* SETUP: Ignore, IN: Accept ISO, OUT: Ignore */
    USBDEV_EP_MODE_NAK_OUT,             /* SETUP: Ignore, IN: Ignore, OUT: NAK */
    USBDEV_EP_MODE_ACK_OUT,             /* SETUP: Ignore, IN: Ignore, OUT: ACK or STALL
                                           (changes mode to NAK_OUT after ACK) */
    USBDEV_EP_MODE_ACK_OUT_STATUS_IN = 11,
    /* SETUP: Ignore, IN: Respond 0B, OUT: ACK */
    USBDEV_EP_MODE_NAK_IN,              /* SETUP: Ignore, IN: NAK, OUT: Ignore */
    USBDEV_EP_MODE_ACK_IN,              /* SETUP: Ignore, IN: ACK or STALL, OUT: Ignore
                                           (changes mode to NAK_IN after ACK) */
    USBDEV_EP_MODE_ACK_IN_STATUS_OUT = 15,
    /* SETUP: Ignore, IN: Respond, OUT: ACK 0B, NAK others */

} usbdev_ep_mode_t;

/**
 * The following are register definitions made for ease of use. Refer
 * to the register definition macros for mode details.
 */
typedef struct
{
    volatile uint32_t sie_epx_cnt0;
    volatile uint32_t sie_epx_cnt1;
    volatile uint32_t sie_epx_cr0;
    volatile uint32_t rsrvd0[0x0D];

} USBSIE_REGS_T, *PUSBSIE_REGS_T;

/**
 * Register structure extern
 */
extern const PUSBSIE_REGS_T USBSIE[];

/**
 * The following are register definitions made for ease of use. Refer
 * to the register definition macros for mode details.
 */
typedef struct
{
    volatile uint32_t arb_epx_cfg;
    volatile uint32_t arb_epx_int_en;
    volatile uint32_t arb_epx_sr;
    volatile uint32_t rsrvd0;
    volatile uint32_t arb_rwx_wa;
    volatile uint32_t arb_rwx_wa_msb;
    volatile uint32_t arb_rwx_ra;
    volatile uint32_t arb_rwx_ra_msb;
    volatile uint32_t arb_rwx_dr;
    volatile uint32_t rsrvd1[8];

} USBARB_REGS_T, *PUSBARB_REGS_T;

/**
 * Register structure extern
 */
extern const PUSBARB_REGS_T USBARB[];

/**
 * The following are register definitions made for ease of use. Refer
 * to the register definition macros for mode details.
 */
typedef struct
{
    volatile uint32_t arb_rwx_wa16;
    volatile uint32_t rsrvd0;
    volatile uint32_t arb_rwx_ra16;
    volatile uint32_t rsrvd1;
    volatile uint32_t arb_rwx_dr16;
    volatile uint32_t rsrvd2[0x0B];

} USBARB16_REGS_T, *PUSBARB16_REGS_T;

/**
 * Register structure extern
 */
extern const PUSBARB16_REGS_T USBARB16[];

/*****************************************************************************
 ************************ Global variable declarations ***********************
 *****************************************************************************/

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief VBUS state change handler function
 *
 * The function is expected to be invoked from by the application code whenever
 * VBUS state changes. For a USB device, the VBUS turning on is a connect event
 * and VBUS turning off is a disconnect event. The USB block control shall be
 * done accordingly.
 *
 * @param vbus_state Current VBUS state. true = VBUS present, false = VBUS absent.
 *
 * @return None
 */
void usb_vbus_int_handler(bool vbus_state);

/**
 * @brief The function locks all USB interrupts to provide USB module specific
 * mutex lock.
 *
 * The function allows to lock all USB module interrupts without having to lock
 * all interrupts in the system. It should be noted that USB module cannot be
 * started or stopped from a locked state. The function can be called in a nested
 * fashion. But it should be noted that the unlock should be done in the reverse
 * order of locking to ensure correct state on exit.
 *
 * @return 32-bit lock status at the time of the lock.
 */
uint32_t usb_intr_lock(void);

/**
 * @brief The function un-locks USB interrupts due to a previous lock call.
 *
 * The function can be called in a nested fashion. But it should be noted that
 * the unlock should be done in the reverse order of locking to ensure correct
 * state on exit.
 *
 * @param lock Lock state returned the previous lock call
 *
 * @return None
 */
void usb_intr_unlock(uint32_t lock);

/**
 * @brief USB module task handler
 *
 * USB module interrupts have considerable processing and the task handler is
 * used to deferring the handling to main task loop. The function is expected
 * to be invoked continously from the main task loop. It is expected that the
 * loop gets to run at least once every millisecond whenever there is activity.
 *
 * @return None
 */
void usb_task(void);

/**
 * @brief Initialize the USB module
 *
 * The API initializes the USB interface. This is mainly a software state
 * machine initialization. The PHY is not enabled at this point. The API
 * helps to cleanup previous state information.
 *
 * @param cfg USB module configuration parameters.
 *
 * @return Status of the call
 */
ccg_status_t usb_init(usb_config_t *cfg);

/**
 * @brief Enables the USB device mode of operation.
 *
 * The function initializes the USB hardware and enables the D+/D- lines and does
 * a pull up on the D+ line for the external host to detect the presence of
 * the device. The API expects that the USB block is already initialized.
 *
 * @param Whether to enable the internal regulator for the block.
 *
 * @return Status of the call
 */
ccg_status_t usb_enable(bool reg_enable);

/**
 * @brief Disables the USB device mode of operation
 *
 * The function disables the USB hardware and disconnects the D+/D- lines.
 *
 * @return Status of the call
 */
ccg_status_t usb_disable(void);

/**
 * @brief Get current USB state.
 *
 * The API returns the current state of the USB device module.
 *
 * @return Current USB state
 */
usb_state_t usb_get_state(void);

/**
 * @brief Check whether the USB module is idle or not.
 *
 * The API checks if there are any pending task for USB module. The function
 * is expected to be invoked before entering SLEEP mode. It should be noted
 * that the DEEP_SLEEP mode requires USB block to be in suspend state and
 * this has be checked using usb_get_state() function.
 *
 * @return true = USB module is idle, false = USB module is busy.
 */
bool usb_is_idle(void);

#if (USB_SUSPEND_CHECK_DISABLE == 0)
/**
 * @brief Sends a remote wakeup signal on the USB bus.
 *
 * The API sends a remote wakeup event on the USB bus if the USB bus is in
 * suspended state and if the host has requested for a remote wakeup. 
 * 
 * @return Status of the call.
 */
ccg_status_t usb_remote_wakeup(void);
#endif /* USB_SUSPEND_CHECK_DISABLE */

/**
 * @brief Sends / receives EP0 ZLP status phase
 *
 * The API completes the status phase of the current EP0 request. The caller
 * is expected to call the function in sequence. The status phase is handled
 * inplicitly when the setup_read and setup_write functions are invoked correctly
 * with the last flag set to true. The function does not wait for the transfer to
 * complete. In case of an error, the EP0 shall be stalled by the USB module.
 *
 * @return Status of the call
 */
ccg_status_t usb_ep0_send_recv_status(void);

/**
 * @brief Waits for the EP0 status phase to complete
 *
 * The function does a blocking wait until the status phase is completed.
 * This function should be invoked only if the task loop can be safely blocked.
 * Otherwise use the callback functionality available with the transfers.
 *
 * @return Status of the call
 */
ccg_status_t usb_ep0_wait_for_ack(void);

/**
 * @brief Stall EP0
 *
 * The function stalls endpoint zero to indicate error to current request. The
 * stall is automatically cleared on receiving a new setup request.
 *
 * @return Status of the call
 */
ccg_status_t usb_ep0_set_stall(void);

/**
 * @brief Setup a read data transfer on EP0
 *
 * The API does not wait for the read to complete. The function just updates
 * the state machine and queues the first packet read. The read has to be
 * completed by repeatedly queueing packet read requests. The last parameter
 * can be used to do  multiple partial transfers. For default single tranfers,
 * the last parameter should always be true.
 *
 * @param data Buffer to write the received USB EP0 data. The caller should
 *        ensure that the buffer is capable of receiving upto a size of (length)
 * @param length Length of data to be transfer. This has to be a multiple of eitght bytes.
 * @param last  Wether the request is a partial transfer or not. Set to true
 *        if the module needs to implicily handle the status phase after
 *        completing the transfer.
 * @param cb Callback to be invoked at the end of transfer. Optional.
 *
 * @return Status of the call
 */
ccg_status_t usb_ep0_setup_read(uint8_t *data, uint16_t length, bool last, usb_setup_cb_t cb);

/**
 * @brief Setup a write data transfer on EP0
 * The API does not wait for the write to complete. The function just updates
 * the state machine and queues the first packet. The write has to be completed
 * by repeatedly queueing packet requests. The last parameter can be used to do
 * multiple partial transfers. For default single tranfers, the last parameter
 * should always be true.
 *
 * @param data Buffer to write the USB EP0 data.
 * @param length Length of data to be transfer.
 * @param last Wether the request is a partial transfer or not. Set to true
 *      if the module needs to implicily handle the status phase
 *      after completing the transfer.
 * @param cb Callback to be invoked at the end of transfer. Optional.
 *
 * @return Status of the call
 */
ccg_status_t usb_ep0_setup_write(uint8_t *data, uint16_t length, bool last, usb_setup_cb_t cb);

/**
 * @brief Enable an endpoint with the selected configuration
 *
 * The endpoint shall be initialized and configured with the provided parameters.
 * The API expects the endpoint to be in disabled state.
 *
 * @param ep_index Endpoint to be enabled - Caller needs to ensure validity
 * @param is_out Direction of the endpoint
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_enable(usb_ep_index_t ep_index, bool is_out);

/**
 * @brief Disables a previously enabled endpoint
 *
 * The endpoint shall be disabled and all data in the FIFO cleared. The endpoint
 * shall stop responding to requests from USB host.
 *
 * @param ep_index Endpoint to be disabled
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_disable(usb_ep_index_t ep_index);

/**
 * @brief Stall the selected endpoint
 *
 * The endpoint shall stall all IN / OUT tokens after the function has been
 * executed.
 *
 * @param ep_index Endpoint to be stalled
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_set_stall(usb_ep_index_t ep_index);

/**
 * @brief Clear the stall on the selected endpoint
 *
 * The function shall clear a previously stalled endpoint. The function shall
 * reset the data toggle even if the endpoint was not previously stalled.
 * This call shall  also reset the endpoint to the default state. So read
 * should be explicitly invoked for an OUT endpoint. The endpoint shall start
 * NAKing all the IN / OUT tokens once the function gets executed.
 *
 * @param ep_index Endpoint to clear the stall
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_clear_stall(usb_ep_index_t ep_index);

/**
 * @brief Checks whether the endpoint is ready for data transfer
 *
 * The API expects that the endpoint is enabled and active
 *
 * @param ep_index Endpoint to be checked
 *
 * @return Endpoint status:
 *      true  - If this is an IN endpoint, then the EP is ready to send data.
 *              If there was a previous transfer then it has completed successfully.
 *              If this is an OUT endpoint, then the EP has received a packet
 *              of data from the USB host.
 *      false - If this is an IN endpoint, data is being sent out and not completed.
 *              If this is an OUT endpoint, then the data is not yet received.
 *              If the endpoint is not active or USB connection is not active,
 *              this API returns CyFalse.
 */
bool usb_ep_is_ready(usb_ep_index_t ep_index);

/**
 * @brief Queues a read operation on the selected endpoint
 *
 * The function enables the selected endpoint to receive one packet of data.
 * It does not wait for the data to be received.
 *
 * @param ep_index Endpoint to be selected
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_queue_read_single(usb_ep_index_t ep_index);

/**
 * @brief Retreives the data packet available on the endpoint
 *
 * The function expects that a data is already available and retreives the
 * packet from the endpoint buffer.
 *
 * @param ep_index Endpoint to be selected
 * @param data Buffer pointer to read the data into
 * @param count Pointer to return the actual count of data received
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_read_single(usb_ep_index_t ep_index, uint8_t *data, uint8_t *count);

/**
 * @brief Send ZLP on the selected endpoint
 *
 * The function sends a zero-length packet on the selected IN endpoint.
 *
 * @param ep_index Endpoint to be selected
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_send_zlp(usb_ep_index_t ep_index);

/**
 * @brief Queue a packet on the selected IN endpoint
 *
 * The function copies the data available in the buffer to the endpoint buffer
 * and arms the endpoint for transfer.
 *
 * @param ep_index Endpoint to be selected
 * @param data Buffer pointer where the data is available
 * @param count Numbers of bytes to transmit. The size cannot exceed 64.
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_write_single(usb_ep_index_t ep_index, uint8_t *data, uint8_t  count);

/**
 * @brief Flushes and clears the selected endpoint
 *
 * The function resets the endpoint and re-arms an OUT endpoint to receive
 * data if a receive was already queued.
 *
 * @param ep_index Endpoint to be selected
 *
 * @return Status of the call
 */
ccg_status_t usb_ep_flush(usb_ep_index_t ep_index);

#endif /* _USB_H_ */

/*[]*/

