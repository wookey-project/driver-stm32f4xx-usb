#include "autoconf.h"
#include "api/usb.h"

/*
 * This should be replaced in Kconfig and source:
 * - both fs and hs mode in the same driver
 */
#ifdef CONFIG_USR_DRV_USB
#include "stm32f4xx_usb_fs.h"
#endif

#ifdef CONFIG_USR_DRV_USB_HS
#include "stm32f4xx_usb_hs.h"
#endif



 /* usb_driver_init - Initialize the USB layer
 * @data_received: callback called when data is received. The parameter is the
 * size of received data.
 * @data_sent: callback called when data has been sent
 */
void usb_driver_init(void)
{
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_init();
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_init();
#endif
}




 /* usb_driver_init - Initialize the USB layer
 * @data_received: callback called when data is received. The parameter is the
 * size of received data.
 * @data_sent: callback called when data has been sent
 */
void usb_driver_early_init(void (*data_received)(uint32_t), void (*data_sent)(void)){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_early_init(data_received, data_sent);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_early_init(data_received, data_sent);
#endif
}

/**
 * usb_driver_send - Send data throw USB
 * @src: address of the data to send. The buffer's size must be at least @size.
 * @size: number of bytes to send.
 * @ep: endpoint on which send data.
 */
void usb_driver_send(const void *src, uint32_t size, uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_send(src, size, ep);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_send(src, size, ep);
#endif
}

/**
 * usb_driver_read - Read data from USB
 * @dst: buffer in which read data will be written.
 * @size: number of bytes to read.
 * @ep: endpoint on which read data.
 */
void usb_driver_read(void *dst, uint32_t size, uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_read(dst, size, ep);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_read(dst, size, ep);
#endif
}

/**
 * usb_driver_set_address - Set the address of the device
 * @addr: Device's address
 */
void usb_driver_set_address(uint16_t addr){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_set_address(addr);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_set_address(addr);
#endif
}

/**
 * \brief Stall IN endpoint
 *
 * @param ep Endpoint
 */
usb_ep_error_t usb_driver_stall_in(uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS
	return usb_fs_driver_stall_in(ep);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	return usb_hs_driver_stall_in(ep);
#endif

}

void usb_driver_stall_in_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_stall_in_clear(ep, type, start_data_toggle);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_stall_in_clear(ep, type, start_data_toggle);
#endif
}

/**
 * \brief Stall OUT endpoint
 *
 * @param ep Endpoint
 */
void usb_driver_stall_out(uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_stall_out(ep);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_stall_out(ep);
#endif
}

void usb_driver_stall_out_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_driver_stall_out_clear(ep, type, start_data_toggle);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_driver_stall_out_clear(ep, type, start_data_toggle);
#endif
}
