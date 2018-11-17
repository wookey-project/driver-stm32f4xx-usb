#ifndef STM32F4XX_USB_H
#define STM32F4XX_USB_H

#include "api/types.h"
#include "autoconf.h"

#ifdef CONFIG_USR_DRV_USB_FS
    #define EP0 USB_FS_DXEPCTL_EP0
#endif

#ifdef CONFIG_USR_DRV_USB_HS
    #define EP0 USB_HS_DXEPCTL_EP0
#endif


typedef enum {
    USB_OK  = 0,
    USB_ERROR_BUSY = 1,
    USB_ERROR_BAD_INPUT = 2,
    USB_ERROR_RANGE = 3,
    USB_ERROR_NOT_SUPORTED = 4,
    USB_ERROR_ALREADY_ACTIVE,
    USB_ERROR_NOT_ACTIVE,
    USB_ERROR_NO_SPACE
} usb_ep_error_t;

void usb_driver_early_init(void (*data_received)(uint32_t), void (*data_sent)(void));

 /* usb_driver_init - Initialize the USB layer
 * @data_received: callback called when data is received. The parameter is the
 * size of received data.
 * @data_sent: callback called when data has been sent
 */
void usb_driver_init(void);

/**
 * usb_driver_setup_send - Send data throw USB
 * @src: address of the data to send. The buffer's size must be at least @size.
 * @size: number of bytes to send.
 * @ep: endpoint on which send data.
 */
void usb_driver_setup_send(const void *src, uint32_t size, uint8_t ep);


void usb_driver_setup_send_status(int status);
void usb_driver_setup_read_status(void);

/**
 * usb_driver_setup_read - Read data from USB
 * @dst: buffer in which read data will be written.
 * @size: number of bytes to read.
 * @ep: endpoint on which read data.
 */
void usb_driver_setup_read(void *dst, uint32_t size, uint8_t ep);

/**
 * usb_driver_set_address - Set the address of the device
 * @addr: Device's address
 */
void usb_driver_set_address(uint16_t addr);

/**
 * \brief Stall IN endpoint
 *
 * @param ep Endpoint
 */
usb_ep_error_t usb_driver_stall_in(uint8_t ep);
void usb_driver_stall_in_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle);

/**
 * \brief Stall OUT endpoint
 *
 * @param ep Endpoint
 */
void usb_driver_stall_out(uint8_t ep);
void usb_driver_stall_out_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle);

#endif /* STM32F4XX_USB_H */
