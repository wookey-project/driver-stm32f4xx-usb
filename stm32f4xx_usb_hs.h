#ifndef STM32F4XX_USB_HS_H
# define STM32F4XX_USB_HS_H

#include "autoconf.h"
#ifdef CONFIG_USR_DRV_USB_HS

#include "libc/regutils.h"
#include "libc/types.h"
#include "stm32f4xx_usb_hs_regs.h"
#include "api/usb_control.h"
#include "api/usb.h"
#define MAX_TIME_DETACH     4000

#define USB_GLOBAL_OUT_NAK        0b0001 /* Global OUT NAK (triggers an interrupt) */
#define USB_OUT_PACKET_RECEIVED   0b0010 /* OUT data packet received */
#define USB_OUT_TRANSFERT_COMPLETED   0b0011 /* OUT transfer completed (triggers an interrupt) */
#define USB_SETUP_TRANS_COMPLETED 0b0100 /* SETUP transaction completed (triggers an interrupt) */
#define USB_SETUP_PACKET_RECEIVED 0b0110 /* SETUP data packet received */


#define assert(val) if (!(val)) { while (1) ; };

typedef enum {
    USB_EP_STATE_IDLE  = 0,
    USB_EP_STATE_SETUP = 1,
    USB_EP_STATE_STATUS = 2,
    USB_EP_STATE_STALL = 3,
    USB_EP_STATE_DATA_IN = 4,
    USB_EP_STATE_DATA_OUT = 5,
} usb_ep_state_t;

typedef enum {
    USB_EP_IN  = 0x80,
    USB_EP_OUT = 0x01,
} usb_ep_dir_t;


typedef const volatile struct __packed {
    usb_ep_nb_t      num;
    usb_ep_mpsize_t  max_packet_size;
    usb_ep_type_t    type;
    usb_ep_toggle_t  start_data_toggle;
    usb_ep_state_t   state;
    usb_ep_dir_t     dir;
} usb_ep_t;

 /* usb_driver_init - Initialize the USB layer
 * @data_received: callback called when data is received. The parameter is the
 * size of received data.
 * @data_sent: callback called when data has been sent
 */
void usb_hs_driver_init(void);

void usb_hs_driver_early_init(void (*data_received)(uint32_t), void (*data_sent)(void));

/**
 * usb_driver_send - Send data throw USB
 * @src: address of the data to send. The buffer's size must be at least @size.
 * @size: number of bytes to send.
 * @ep: endpoint on which send data.
 */
void usb_hs_driver_send(const void *src, uint32_t size, uint8_t ep);

/**
 * usb_driver_read - Read data from USB
 * @dst: buffer in which read data will be written.
 * @size: number of bytes to read.
 * @ep: endpoint on which read data.
 */
void usb_hs_driver_read(void *dst, uint32_t size, uint8_t ep);

/**
 * usb_driver_set_address - Set the address of the device
 * @addr: Device's address
 */
void usb_hs_driver_set_address(uint16_t addr);

/**
 * \brief Stall IN endpoint
 *
 * @param ep Endpoint
 */
usb_ep_error_t usb_hs_driver_stall_in(uint8_t ep);
void usb_hs_driver_stall_in_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle);

/**
 * \brief Stall OUT endpoint
 *
 * @param ep Endpoint
 */
void usb_hs_driver_stall_out(uint8_t ep);
void usb_hs_driver_stall_out_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle);


void usb_hs_driver_map(void);

void usb_hs_driver_send_zlp(uint8_t ep);

#endif /*!CONFIG_USR_DRV_USB_HS*/

#endif /* STM32F4XX_USB_HD_H */
