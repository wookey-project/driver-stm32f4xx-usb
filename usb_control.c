/** @file usb_control.c
 *
 */

#include "autoconf.h"
//#include "product.h"
//#include "debug.h"
#include "api/usb.h"
#ifdef CONFIG_USR_DRV_USB_FS
#include "stm32f4xx_usb_fs.h"
#endif

#ifdef CONFIG_USR_DRV_USB_HS
#include "stm32f4xx_usb_hs.h"
#endif


#include "api/usb_control.h"
#include "api/print.h"

//#include "usb_device.h"

/* Microsoft Vendor Code used for OS Descriptor request */
static uint8_t MSFT100_SIG[MSFT100_SIG_SIZE] = {
            0x4D, 0x00, 0x53, 0x00, 0x46, 0x00, 0x54, 0x00,
            0x31, 0x00, 0x30, 0x00, 0x30, 0x00
        };


usb_ctrl_device_descriptor_t usb_ctrl_device_desc = {0};
usb_ctrl_full_configuration_descriptor_t usb_ctrl_conf_desc = {0};


#ifdef CONFIG_USR_DRV_USB_FS 
/**
 * \brief Read on EP0.
 */
static void read_fs_status(void){
    // FIXME THIS IS WRONG ... MUST BE REMOVED
	usb_driver_read(NULL, 0, USB_FS_DXEPCTL_EP0);
}
#endif
#ifdef CONFIG_USR_DRV_USB_HS
/**
 * \brief Read on EP0.
 */
static void read_hs_status(void){
    // FIXME THIS IS WRONG ... MUST BE REMOVED
	usb_driver_read(NULL, 0, USB_HS_DXEPCTL_EP0);
}
#endif

static void read_status(void){
#ifdef CONFIG_USR_DRV_USB_FS 
	read_fs_status();
#endif
#ifdef CONFIG_USR_DRV_USB_HS
	read_hs_status();
#endif
}

/**
 * \brief Stall OUT endpoint
 *
 * @param ep Endpoint
 */
#ifdef CONFIG_USR_DRV_USB_FS 
static void usb_fs_ctrl_stall_out(uint8_t ep){
	////printf("STALL IN\n");
	/* Put core in Global OUT NAK mode */
	set_reg(r_CORTEX_M_USB_FS_DCTL, 1, USB_FS_DCTL_SGONAK);

	/* Disable OUT EP and set STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_STALL);

	/* Clear STALL bit when app is ready */
	/* Done when request or data are received */
}
#endif
#ifdef CONFIG_USR_DRV_USB_HS
static void usb_hs_ctrl_stall_out(uint8_t ep){
	////printf("STALL IN\n");
	/* Put core in Global OUT NAK mode */
	set_reg(r_CORTEX_M_USB_HS_DCTL, 1, USB_HS_DCTL_SGONAK);

	/* Disable OUT EP and set STALL bit */
	set_reg(r_CORTEX_M_USB_HS_DIEPCTL(ep), 1, USB_HS_DIEPCTL_STALL);

	/* Clear STALL bit when app is ready */
	/* Done when request or data are received */
}
#endif

void usb_ctrl_stall_out(uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS 
	usb_fs_ctrl_stall_out(ep);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_ctrl_stall_out(ep);
#endif
}

/**
 * \brief Stall IN endpoint
 *
 * @param ep Endpoint
 */
#ifdef CONFIG_USR_DRV_USB_FS 
void usb_fs_ctrl_stall_in(uint8_t ep __attribute__((unused))){
	////printf("STALL OUT\n");
	/* Disable IN EP and set STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_EPDIS);
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_STALL);

	/* Assert on Endpoint Disabled interrupt */
	if (!(read_reg_value(r_CORTEX_M_USB_FS_DIEPINT(ep)) & USB_FS_DIEPINT_EPDISD_Msk)) { while(1); // panic
    }

	/* Flush transmit FIFO
	 * p1279 Rev14 RM0090
	 * Read NAK Eff Int and write AHBIL bit
	 */
	while (read_reg_value(r_CORTEX_M_USB_FS_GINTSTS) & USB_FS_GINTSTS_GINAKEFF_Msk) {}
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_AHBIDL);

	/* Select which ep to flush and do it */
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, ep, USB_FS_GRSTCTL_TXFNUM);
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_TXFFLSH);

	/* Clear STALL bit */
	/* Done when request is received */
}
#endif

#ifdef CONFIG_USR_DRV_USB_HS 
void usb_hs_ctrl_stall_in(uint8_t ep){
	////printf("STALL OUT\n");
	/* Disable IN EP and set STALL bit */
	set_reg(r_CORTEX_M_USB_HS_DIEPCTL(ep), 1, USB_HS_DIEPCTL_EPDIS);
	set_reg(r_CORTEX_M_USB_HS_DIEPCTL(ep), 1, USB_HS_DIEPCTL_STALL);

	/* Assert on Endpoint Disabled interrupt */
	if (!(read_reg_value(r_CORTEX_M_USB_HS_DIEPINT(ep)) & USB_HS_DIEPINT_EPDISD_Msk)) {
        while(1);
    }

	/* Flush transmit FIFO
	 * p1279 Rev14 RM0090
	 * Read NAK Eff Int and write AHBIL bit
	 */
	while (read_reg_value(r_CORTEX_M_USB_HS_GINTSTS) & USB_HS_GINTSTS_GINAKEFF_Msk) {}
	set_reg(r_CORTEX_M_USB_HS_GRSTCTL, 1, USB_HS_GRSTCTL_AHBIDL);

	/* Select which ep to flush and do it */
	set_reg(r_CORTEX_M_USB_HS_GRSTCTL, ep, USB_HS_GRSTCTL_TXFNUM);
	set_reg(r_CORTEX_M_USB_HS_GRSTCTL, 1, USB_HS_GRSTCTL_TXFFLSH);

	/* Clear STALL bit */
	/* Done when request is received */
}
#endif

void usb_ctrl_stall_in(uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_ctrl_stall_in(ep);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_ctrl_stall_in(ep);
#endif
}

/**
 *
 */
#ifdef CONFIG_USR_DRV_USB_FS 
void usb_fs_ctrl_stall_clear(uint8_t ep){
	/* Clear STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 0, USB_FS_DIEPCTL_STALL);

}
#endif

#ifdef CONFIG_USR_DRV_USB_HS 
void usb_hs_ctrl_stall_clear(uint8_t ep){
	/* Clear STALL bit */
	set_reg(r_CORTEX_M_USB_HS_DIEPCTL(ep), 0, USB_HS_DIEPCTL_STALL);

}
#endif

void usb_ctrl_stall_clear(uint8_t ep){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_ctrl_stall_clear(ep);
#endif
#ifdef CONFIG_USR_DRV_USB_HS 
	usb_hs_ctrl_stall_clear(ep);
#endif
}

/**
 * \brief Send acknowledgment.
 *
 * @param status Discarded.
 */
#ifdef CONFIG_USR_DRV_USB_FS 
static void usb_fs_ctrl_send_status(int status){
	if (!status) {
		usb_fs_driver_send(NULL, 0, USB_FS_DXEPCTL_EP0);
	}
}
#endif

#ifdef CONFIG_USR_DRV_USB_HS 
static void usb_hs_ctrl_send_status(int status){
	if (!status) {
		usb_hs_driver_send(NULL, 0, USB_HS_DXEPCTL_EP0);
	}
}
#endif

static void usb_ctrl_send_status(int status){
#ifdef CONFIG_USR_DRV_USB_FS 
	usb_fs_ctrl_send_status(status);
#endif

#ifdef CONFIG_USR_DRV_USB_HS 
	usb_hs_ctrl_send_status(status);
#endif
}

/**
 * \brief Send device descriptor.
 */
#ifdef CONFIG_USR_DRV_USB_FS 
static void usb_fs_ctrl_device_desc_rqst_handler(void){
	usb_fs_driver_send((uint8_t *)&usb_ctrl_device_desc, sizeof(usb_ctrl_device_desc), USB_FS_DXEPCTL_EP0);
	read_fs_status();
}
#endif

#ifdef CONFIG_USR_DRV_USB_HS 
static void usb_hs_ctrl_device_desc_rqst_handler(void){
	usb_hs_driver_send((uint8_t *)&usb_ctrl_device_desc, sizeof(usb_ctrl_device_desc), USB_HS_DXEPCTL_EP0);
	read_hs_status();
}
#endif

static void usb_ctrl_device_desc_rqst_handler(void){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_ctrl_device_desc_rqst_handler(); 
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_ctrl_device_desc_rqst_handler();
#endif
}

/**
 * \brief Send configuration descriptor.
 *
 * @param wLength Descriptor length.
 */
#ifdef CONFIG_USR_DRV_USB_FS
static void usb_fs_ctrl_configuration_desc_rqst_handler(uint16_t wLength){
	uint32_t len = usb_ctrl_conf_desc.config_desc.wTotalLength > wLength ? wLength : usb_ctrl_conf_desc.config_desc.wTotalLength;
	usb_fs_driver_send((uint8_t *)&usb_ctrl_conf_desc, len, USB_FS_DXEPCTL_EP0);
	read_fs_status();
}
#endif
#ifdef CONFIG_USR_DRV_USB_HS
static void usb_hs_ctrl_configuration_desc_rqst_handler(uint16_t wLength){
	uint32_t len = usb_ctrl_conf_desc.config_desc.wTotalLength > wLength ? wLength : usb_ctrl_conf_desc.config_desc.wTotalLength;
	usb_hs_driver_send((uint8_t *)&usb_ctrl_conf_desc, len, USB_HS_DXEPCTL_EP0);
	read_hs_status();
}
#endif

static void usb_ctrl_configuration_desc_rqst_handler(uint16_t wLength){
#ifdef CONFIG_USR_DRV_USB_FS
	usb_fs_ctrl_configuration_desc_rqst_handler(wLength);
#endif

#ifdef CONFIG_USR_DRV_USB_HS
	usb_hs_ctrl_configuration_desc_rqst_handler(wLength);	
#endif
}


/**
 * \brief Default Class request handler.
 *
 * Class request handling (Run-Time Mode).
 *
 * @param packet Setup packet
 */

static void usb_ctrl_default_class_rqst_handler(struct usb_setup_packet *packet){
    printf("No class rqst handler is defined for class \n", packet->bRequest);
}


/**
 * \brief Default Vendor request handler.
 *
 * Not implemented.
 */
static void usb_ctrl_default_vendor_rqst_handler( __attribute__((unused)) struct usb_setup_packet *packet) {
	printf("[Vendor rqst: Not implemented\n");
}

static void usb_ctrl_default_set_interface_rqst_handler(int iface){
		if (0) printf("Set interface (%x - not implemented yet)\n", iface);
		/* Program EP 0*/

		/* Unmask interrupt in OTG_FS_DAINTMSK reg */

		/* Set up Data FIFO RAM (?) */

		/* Send status IN packet */
		usb_ctrl_send_status(0);
}

static void usb_ctrl_default_set_configuration_rqst_handler(int conf){
	//printf("Set configuration\n");
	if (conf == 1) {
        usb_ctrl_send_status(0);
	} else {
		/* TODO: send error status */
		printf("[USB] SET_CONFIGURATION %d 1\n", conf);
	}
}


/**
 * \brief Send functional descriptor.
 */
static void usb_ctrl_default_functional_desc_rqst_handler(void){
    printf("usb_ctrl_cbs.functional_rqst_handler is not defined\n");
}


/**
 * \brief usb_ctrl_callbacks defined at usb class level and called from usb_ctrl
 */

usb_ctrl_callbacks_t usb_ctrl_callbacks = {
        .class_rqst_handler             = usb_ctrl_default_class_rqst_handler,
        .vendor_rqst_handler            = usb_ctrl_default_vendor_rqst_handler,
        .set_configuration_rqst_handler = usb_ctrl_default_set_configuration_rqst_handler,
        .set_interface_rqst_handler     = usb_ctrl_default_set_interface_rqst_handler,
        .functional_rqst_handler        = usb_ctrl_default_functional_desc_rqst_handler,
};


/**
 * \brief Class request handler.
 *
 * Class request handling (Run-Time Mode).
 *
 * @param packet Setup packet
 */
static void usb_ctrl_class_rqst_handler(struct usb_setup_packet *packet){
    usb_ctrl_callbacks.class_rqst_handler(packet);
}


/**
 * \brief Vendor request handler.
 *
 * Class request handling (Run-Time Mode).
 *
 * @param packet Setup packet
 */
static void usb_ctrl_vendor_rqst_handler(struct usb_setup_packet *packet){
    usb_ctrl_callbacks.vendor_rqst_handler(packet);
}

static void usb_ctrl_set_interface_rqst_handler(int iface){
    usb_ctrl_callbacks.set_interface_rqst_handler(iface);
}

static void usb_ctrl_set_configuration_rqst_handler(int conf){
    usb_ctrl_callbacks.set_configuration_rqst_handler(conf);
}


static void usb_ctrl_functional_desc_rqst_handler(void){
    usb_ctrl_callbacks.functional_rqst_handler();
}

/**
 * \brief Send string descriptor.
 *
 * @param index String index.
 */
static void usb_ctrl_string_desc_rqst_handler(uint8_t index){
	uint32_t i;
	uint32_t len;
	usb_string_descriptor_t string_desc;
	string_desc.bDescriptorType = USB_DESC_STRING;
	switch (index) {
	case 0:
		string_desc.bLength = 4;
		string_desc.wString[0] = LANGUAGE_ENGLISH;
			break;
	case CONFIG_USB_DEV_MANUFACTURER_INDEX:
		len = sizeof(CONFIG_USB_DEV_MANUFACTURER);
		string_desc.bLength = 2 + 2 * len;
		for (i = 0; i < len; i++)
			string_desc.wString[i] = CONFIG_USB_DEV_MANUFACTURER[i];
		break;
	case CONFIG_USB_DEV_PRODNAME_INDEX:
		len = sizeof(CONFIG_USB_DEV_PRODNAME);
		string_desc.bLength = 2 + 2 * len;
		for (i = 0; i < len; i++)
			string_desc.wString[i] = CONFIG_USB_DEV_PRODNAME[i];
		break;
	case CONFIG_USB_DEV_SERIAL_INDEX:
		len = sizeof(CONFIG_USB_DEV_SERIAL);
		string_desc.bLength = 2 + 2 * len;
		for (i = 0; i < len; i++)
			string_desc.wString[i] = CONFIG_USB_DEV_SERIAL[i];
		break;
	case STRING_MICROSOFT_INDEX:
		len = MSFT100_SIG_SIZE + 4;
		string_desc.bLength = 0x12;
		string_desc.bDescriptorType = 0x03;
		for (i = 0; i < MSFT100_SIG_SIZE; i++)
			string_desc.wString[i] = MSFT100_SIG[i];
		string_desc.wString[MSFT100_SIG_SIZE] = 0x05;
		string_desc.wString[MSFT100_SIG_SIZE + 1] = 0x00;
		break;
	default:
		/* TODO: send error status */
		printf("Invalid string index\n");

#ifdef CONFIG_USR_DRV_USB_FS
        usb_driver_stall_in(USB_FS_DXEPCTL_EP0);
#endif
#ifdef CONFIG_USR_DRV_USB_HS 
        usb_driver_stall_in(USB_HS_DXEPCTL_EP0);
#endif
        return;
	}

#ifdef CONFIG_USR_DRV_USB_FS
	usb_driver_send((uint8_t *)&string_desc, string_desc.bLength, USB_FS_DXEPCTL_EP0);
#endif
#ifdef CONFIG_USR_DRV_USB_HS
	usb_driver_send((uint8_t *)&string_desc, string_desc.bLength, USB_HS_DXEPCTL_EP0);
#endif
	read_status();
}



/**
 * \brief Switch function to send descriptors.
 *
 * @param packet Setup packet
 */
static void usb_ctrl_get_descriptor_rqst_handler(struct usb_setup_packet *packet){
	switch (packet->wValue >> 8) {
	case USB_DESC_DEVICE:
		printf("Device descriptor Rqst\n");
		usb_ctrl_device_desc_rqst_handler();
		break;
	case USB_DESC_CONFIG:
		printf("Configuration descriptor Rqst\n");
		usb_ctrl_configuration_desc_rqst_handler(packet->wLength);
		break;
	case USB_DESC_STRING:
		printf("String descriptor Rqst, Index: %x\n", (packet->wValue & 0xff));
		usb_ctrl_string_desc_rqst_handler(packet->wValue & 0xff);
		break;
	case USB_DESC_FUNCT:
		printf("Functional descriptor Rqst\n");
		usb_ctrl_functional_desc_rqst_handler();
		break;
	default:
		/* In case of unsupported descriptor request, we send a
		 * dummy packet to avoid waiting a timeout from the
		 * host before sending another request.
		 */
		usb_ctrl_send_status(0);
		read_status();
		printf("Unhandled descriptor Rqst: %x\n", packet->wValue >> 8);
	}
}




/**
 * \brief Standard request handler.
 *
 * Standard request handling (Run-Time Mode).
 *
 * @param packet Setup packet
 */
static void usb_ctrl_standard_rqst_handler(struct usb_setup_packet *packet){
	switch (packet->bRequest) {

	case USB_RQST_SET_ADDRESS:
		//printf("Set adress Rqst\n");
		usb_driver_set_address(packet->wValue);
        	usb_ctrl_send_status(0);
		break;

	case USB_RQST_GET_DESCRIPTOR:
		//printf("Get descriptor Rqst\n");
		usb_ctrl_get_descriptor_rqst_handler(packet);
		break;

	case USB_RQST_SET_CONFIGURATION:
        	//printf("Set confiuration Rqst\n");
        	usb_ctrl_set_configuration_rqst_handler(packet->wValue);
		break;

	case USB_RQST_SET_INTERFACE: // TODO > DocID018909 Rev12 p1354/1744
        	//printf("Set interface Rqst\n");
        	usb_ctrl_set_interface_rqst_handler(packet->wValue);
		break;

	default:
		printf("Unhandled std request: %x\n", packet->bRequest);
	}
}


/**
 * \brief USB control function.
 *
 * Handles request from host.
 *
 * @param packet Setup packet
 */
void usb_ctrl_handler(struct usb_setup_packet *packet){
	switch ((packet->bmRequestType >> 5) & 0x3) {
	case USB_RQST_TYPE_STANDARD:
		usb_ctrl_standard_rqst_handler(packet);
		break;
	case USB_RQST_TYPE_CLASS:
		usb_ctrl_class_rqst_handler(packet);
		break;
	case USB_RQST_TYPE_VENDOR:
		usb_ctrl_vendor_rqst_handler(packet);
		break;
	default:
		printf("Unhandled request type: %x\n",(packet->bmRequestType >> 5) & 0x3);
	}
}




/**
 * \brief Initialization of usb ctrl callbacks functions.
 * @param callbacks
 */
void usb_ctrl_init( usb_ctrl_callbacks_t cbs, usb_ctrl_device_descriptor_t device_desc, usb_ctrl_full_configuration_descriptor_t conf_desc ){

        if (cbs.class_rqst_handler != NULL){
            usb_ctrl_callbacks.class_rqst_handler = cbs.class_rqst_handler;
        }

        if (cbs.vendor_rqst_handler != NULL){
            usb_ctrl_callbacks.vendor_rqst_handler = cbs.vendor_rqst_handler;
        }

        if (cbs.set_configuration_rqst_handler != NULL){
            usb_ctrl_callbacks.set_configuration_rqst_handler = cbs.set_configuration_rqst_handler;
        }

        if (cbs.set_interface_rqst_handler != NULL){
            usb_ctrl_callbacks.set_interface_rqst_handler = cbs.set_interface_rqst_handler;
        }
        if (cbs.functional_rqst_handler != NULL){
            usb_ctrl_callbacks.functional_rqst_handler = cbs.functional_rqst_handler;
        }

        usb_ctrl_device_desc = device_desc;
        usb_ctrl_conf_desc = conf_desc;

}
