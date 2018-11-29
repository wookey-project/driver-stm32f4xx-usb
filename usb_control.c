/** @file usb_control.c
 *
 */

#include "autoconf.h"
#include "api/usb.h"


#include "api/usb_control.h"
#include "api/print.h"

//#include "usb_device.h"

usb_ctrl_device_descriptor_t usb_ctrl_device_desc;
usb_ctrl_full_configuration_descriptor_t usb_ctrl_conf_desc;

static void usb_ctrl_init_structures(void)
{
    memset((void*)&usb_ctrl_device_desc, 0, sizeof(usb_ctrl_device_descriptor_t));
    memset((void*)&usb_ctrl_conf_desc, 0, sizeof(usb_ctrl_full_configuration_descriptor_t));
}


/**
 * \brief Send device descriptor.
 */
static void usb_ctrl_device_desc_rqst_handler(uint16_t wLength){
	printf("wLength:%d - sizeof(usb_ctrl_device_desc):%d\n", wLength, sizeof(usb_ctrl_device_desc));
    if ( wLength == 0 ){
        usb_driver_setup_send_status(0);
        usb_driver_setup_read_status();
        return;
    }

    if ( wLength > sizeof(usb_ctrl_device_desc)){
        usb_driver_setup_send((uint8_t *)&usb_ctrl_device_desc, sizeof(usb_ctrl_device_desc), EP0);
    }else{
        usb_driver_setup_send((uint8_t *)&usb_ctrl_device_desc, wLength, EP0);
    }

    usb_driver_setup_read_status();
}

/**
 * \brief Send configuration descriptor.
 *
 * @param wLength Descriptor length.
 */
static void usb_ctrl_configuration_desc_rqst_handler(uint16_t wLength){
	printf("wLength:%d - sizeof(usb_ctrl_device_desc):%d\n", wLength, usb_ctrl_conf_desc.config_desc.wTotalLength);
    if ( wLength == 0 ){
        usb_driver_setup_send_status(0);
        usb_driver_setup_read_status();
        return;
    }

    if ( wLength > usb_ctrl_conf_desc.config_desc.wTotalLength ){
        usb_driver_setup_send((uint8_t *)&usb_ctrl_conf_desc,
                                    usb_ctrl_conf_desc.config_desc.wTotalLength , EP0);
    }else{
        usb_driver_setup_send((uint8_t *)&usb_ctrl_conf_desc, wLength, EP0);

    }
    usb_driver_setup_read_status();
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
		printf("Set interface (%x - not implemented yet)\n", iface);
		/* Program EP 0*/

		/* Unmask interrupt in OTG_FS_DAINTMSK reg */

		/* Set up Data FIFO RAM (?) */

		/* Send status IN packet */

	    if (iface == 1) {
            usb_driver_setup_send_status(0);
	    } else {
            /* FIXME We should handle multiple interface */
		    printf("%d 1\n", iface);
            usb_driver_stall_in(EP0);
	    }
}


static void usb_ctrl_default_set_configuration_rqst_handler(int conf){
	printf("%d\n", conf);
	if (conf == 1) {
        usb_driver_setup_send_status(0);
	} else {
        /* FIXME We should handle multiple configuration */
		printf("SET_CONFIGURATION %d 1\n", conf);
        usb_driver_stall_in(EP0);
	}
}


/**
 * \brief Send functional descriptor.
 */
static void usb_ctrl_default_functional_desc_rqst_handler(uint16_t wLength){
    printf("usb_ctrl_cbs.functional_rqst_handler is not defined, wlength:%d\n", wLength);
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


static void usb_ctrl_functional_desc_rqst_handler(uint16_t wLength){
    usb_ctrl_callbacks.functional_rqst_handler(wLength);
}


static void mass_storage_mft_string_desc_rqst_handler(uint16_t wLength){
	printf("MFT String not supported (Stalling), wLength:\n", wLength);
    usb_driver_stall_in(EP0);
}


/**
 * \brief Send string descriptor.
 *
 * @param index String index.
 */
static void usb_ctrl_string_desc_rqst_handler(uint8_t index, uint16_t wLength){
	uint32_t i;
	uint32_t len;
	usb_string_descriptor_t string_desc;

    if ( wLength == 0 ){
        usb_driver_setup_send_status(0);
        usb_driver_setup_read_status();
        return;
    }


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
        printf("STRING_MICROSOFT_INDEX");
        usb_ctrl_callbacks.mft_string_rqst_handler(wLength);
		break;
	default:
		/* TODO: send error status */
		printf("Invalid string index\n");
        usb_driver_stall_in(EP0);
        return;
	}

    if ( wLength > string_desc.bLength){
    	usb_driver_setup_send((uint8_t *)&string_desc, string_desc.bLength, EP0);
    }else{
        usb_driver_setup_send((uint8_t *)&string_desc, wLength, EP0);
    }
    usb_driver_setup_read_status();
}


static void print_setup_packet(struct usb_setup_packet *packet){
    printf("bmRequestType:%x, bRequest:%x, wValue:%x, wIndex:%x, wLength:%x\n",
            packet->bmRequestType,
            packet->bRequest,
            packet->wValue,
            packet->wIndex,
            packet->wLength);
}


/**
 * \brief Switch function to send descriptors.
 *
 * @param packet Setup packet
 */
static void usb_ctrl_get_descriptor_rqst_handler(struct usb_setup_packet *packet){

    print_setup_packet(packet);

    switch (packet->wValue >> 8) {
	case USB_DESC_DEVICE:
		printf("Device descriptor Rqst\n");
		usb_ctrl_device_desc_rqst_handler(packet->wLength);
		break;
	case USB_DESC_CONFIG:
		printf("Configuration descriptor Rqst\n");
		usb_ctrl_configuration_desc_rqst_handler(packet->wLength);
		break;
	case USB_DESC_STRING:
		printf("String descriptor Rqst, Index: %x\n", (packet->wValue & 0xff));
		usb_ctrl_string_desc_rqst_handler(packet->wValue & 0xff, packet->wLength);
		break;
	case USB_DESC_FUNCT:
		printf("Functional descriptor Rqst\n");
		usb_ctrl_functional_desc_rqst_handler(packet->wLength);
		break;
    case USB_DESC_DEVQUAL:
        usb_driver_stall_out(0);
        break;
	default:
		/* In case of unsupported descriptor request, we send a
		 * dummy packet to avoid waiting a timeout from the
		 * host before sending another request.
		 */
		usb_driver_setup_send_status(0);
		usb_driver_setup_read_status();
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
		usb_driver_set_address(packet->wValue);
        	usb_driver_setup_send_status(0);
		break;

	case USB_RQST_GET_DESCRIPTOR:
		usb_ctrl_get_descriptor_rqst_handler(packet);
		break;

	case USB_RQST_SET_CONFIGURATION:
        usb_ctrl_set_configuration_rqst_handler(packet->wValue);
		break;

	case USB_RQST_SET_INTERFACE: // TODO > DocID018909 Rev12 p1354/1744
        usb_ctrl_set_interface_rqst_handler(packet->wValue);
		break;

    case USB_RQST_GET_STATUS:
        // FIXME Work around
        printf("USB_RQST_GET_STATUS\n");
		usb_driver_setup_send("\x00\x00", 2, EP0);
		usb_driver_setup_read_status();
        break;
	default:
		printf("Unhandled std request: %x\n", packet->bRequest);
        usb_driver_stall_in(EP0);
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
void usb_ctrl_init( usb_ctrl_callbacks_t cbs,
                    usb_ctrl_device_descriptor_t device_desc,
                    usb_ctrl_full_configuration_descriptor_t conf_desc ){

        usb_ctrl_init_structures();

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
        if (cbs.mft_string_rqst_handler != NULL){
            usb_ctrl_callbacks.mft_string_rqst_handler = cbs.mft_string_rqst_handler;
        }

        usb_ctrl_device_desc = device_desc;
        usb_ctrl_conf_desc = conf_desc;

}
