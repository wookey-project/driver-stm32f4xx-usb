/** @file usb_control.h
 *
 */
#ifndef _USB_CONTROL_H
#define _USB_CONTROL_H

#include "api/types.h"
#include "api/regutils.h"

#define LANGUAGE_ENGLISH        0x0409

/* Request types */
#define USB_RQST_TYPE_STANDARD		0x0
#define USB_RQST_TYPE_CLASS		0x1
#define USB_RQST_TYPE_VENDOR		0x2

/* Standard device requests */
#define USB_RQST_GET_STATUS		0x0
#define USB_RQST_CLEAR_FEATURE		0x1
#define USB_RQST_SET_FEATURE		0x3
#define USB_RQST_SET_ADDRESS		0x5
#define USB_RQST_GET_DESCRIPTOR		0x6
#define USB_RQST_SET_DESCRIPTOR		0x7
#define USB_RQST_GET_CONFIGURATION	0x8
#define USB_RQST_SET_CONFIGURATION	0x9
#define USB_RQST_GET_INTERFACE		0xA
#define USB_RQST_SET_INTERFACE		0xB
#define USB_RQST_SYNCH_FRAME		0xC

/* Class device requests */
#define USB_RQST_GET_MAX_LUN		0xfe
#define USB_RQST_MS_RESET		0xff

/* Descriptor types */
#define USB_DESC_DEVICE			0x1
#define USB_DESC_CONFIG			0x2
#define USB_DESC_STRING			0x3
#define USB_DESC_INTERFACE		0x4
#define USB_DESC_EP			0x5
#define USB_DESC_FUNCT			0x21 /* Value found thanks to dfu-util */

/* EP */
#define USB_EP_ATTR_NO_SYNC (0 << 2)
#define USB_EP_ATTR_ASYNC (1 << 2)
#define USB_EP_ATTR_ADAPTIVE (2 << 2)
#define USB_EP_ATTR_SYNC (3 << 2)
#define USB_EP_USAGE_DATA (0 << 4)
#define USB_EP_USAGE_FEEDBACK (1 << 4)
#define USB_EP_USAGE_IMPLICIT_FEEDBACK (2 << 4)

/* EP Transfer types */
#define USB_EP_TYPE_CONTROL         0x00
#define USB_EP_TYPE_ISOCHRONOUS     0x01
#define USB_EP_TYPE_BULK            0x02
#define USB_EP_TYPE_INTERRUPT       0x03

/* Strings */
#define MAX_DESC_STRING_SIZE		32


/* Microsoft */
#define STRING_MICROSOFT_INDEX	    0xEE
#define MSFT100_SIG_SIZE            0x14


/**
 * \brief Device descriptor set, for both run-time and DFU mode.
 */
typedef struct __packed usb_ctrl_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} usb_ctrl_device_descriptor_t;

/**
 * \brief Configuration descriptor set for run-time mode.
 */
typedef struct __packed usb_ctrl_configuration_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	struct {
		uint8_t reserved:5;
		uint8_t remote_wakeup:1;
		uint8_t self_powered:1;
		uint8_t reserved7:1;
	} bmAttributes;
	uint8_t bMaxPower;
} usb_ctrl_configuration_descriptor_t;

/**
 * Interface descriptor set, for both run-time and DFU mode.
 */
typedef struct __packed usb_ctrl_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} usb_ctrl_interface_descriptor_t;

/**
 * Endpoint descriptor set for run-time mode (only?).
 */
typedef struct __packed usb_ctrl_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
    uint8_t bmAttributes;
/*	struct {
		uint8_t transfer_type:2;
		uint8_t iso_synchro_type:2;
		uint8_t iso_usage_type:2;
		uint8_t reserved:2;
	} bmAttributes;
*/
	uint16_t wMaxPacketSize;
	uint8_t bInterval;

} usb_ctrl_endpoint_descriptor_t;



typedef struct __packed usb_ctrl_full_configuration_descriptor {
	usb_ctrl_configuration_descriptor_t config_desc;
    /*  Each interfaces if followed by its endpoints configuration
     *  FIXME We only support one interface per configuration
     */
	usb_ctrl_interface_descriptor_t interface_desc;
	usb_ctrl_endpoint_descriptor_t ep_in;
	usb_ctrl_endpoint_descriptor_t ep_out;
} usb_ctrl_full_configuration_descriptor_t;


/**
 * \brief String descriptor.
 */
typedef struct __packed usb_string_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wString[MAX_DESC_STRING_SIZE];
} usb_string_descriptor_t;


typedef struct __packed usb_functional_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    struct {
        uint8_t bitCanDnload:1;
        uint8_t bitCanUpload:1;
        uint8_t bitManifestationTolerant:1;
        uint8_t bitWillDetach:1;
        uint8_t reserved:4;
    } bmAttributes;
    uint16_t wDetachTimeOut;
    uint16_t wTransferSize;
    uint16_t bcdDFUVersion;
} usb_functional_descriptor_t;

/**
 * \struct usb_setup_packet
 * \brief USB setup packet.
 *
 * Setup packet to receive requests on EP0.
 */
struct usb_setup_packet {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};


/* Callbacks */
typedef void (*cb_class_rqst_handler_t) (struct usb_setup_packet *);
typedef void (*cb_functional_rqst_desc_t) (void);
typedef void (*cb_vendor_rqst_handler_t) (struct usb_setup_packet *);
typedef void (*cb_set_configuration_rqst_handler_t) (int);
typedef void (*cb_set_interface_rqst_handler_t) (int);


typedef struct __packed usb_ctrl_callbacks {
    cb_class_rqst_handler_t             class_rqst_handler;
    cb_vendor_rqst_handler_t            vendor_rqst_handler;
    cb_set_configuration_rqst_handler_t set_configuration_rqst_handler;
    cb_set_interface_rqst_handler_t     set_interface_rqst_handler;
    cb_functional_rqst_desc_t           functional_rqst_handler;
} usb_ctrl_callbacks_t;


void usb_ctrl_handler(struct usb_setup_packet *packet);
void usb_ctrl_init(usb_ctrl_callbacks_t, usb_ctrl_device_descriptor_t, usb_ctrl_full_configuration_descriptor_t);
void usb_ctrl_stall_in(uint8_t ep);
void usb_ctrl_stall_out(uint8_t ep);
void usb_ctrl_stall_clear(uint8_t ep);

#define MAX_CTRL_PACKET_SIZE 64

#ifdef CONFIG_USR_DRV_USB_FS 

# define MAX_DATA_PACKET_SIZE 64

typedef enum {
    USB_FS_DXEPCTL_EP0 = 0,
    USB_FS_DXEPCTL_EP1 = 1,
    USB_FS_DXEPCTL_EP2 = 2,
    USB_FS_DXEPCTL_EP3 = 3,
} usb_ep_nb_t;

typedef enum {
    USB_FS_D0EPCTL_MPSIZ_64BYTES = 0,
    USB_FS_D0EPCTL_MPSIZ_32BYTES = 1,
    USB_FS_D0EPCTL_MPSIZ_16BYTES = 2,
    USB_FS_D0EPCTL_MPSIZ_8BYTES  = 3,
    USB_FS_DXEPCTL_MPSIZ_64BYTES = 64,
    USB_FS_DXEPCTL_MPSIZ_128BYTES = 128,
    USB_FS_DXEPCTL_MPSIZ_512BYTES = 512,
    USB_FS_DXEPCTL_MPSIZ_1024BYTES  = 1024,
} usb_ep_mpsize_t;

typedef enum {
    USB_FS_DXEPCTL_EPTYP_CONTROL = 0,
    USB_FS_DXEPCTL_EPTYP_ISOCHRO = 1,
    USB_FS_DXEPCTL_EPTYP_BULK    = 2,
    USB_FS_DXEPCTL_EPTYP_INT     = 3,
} usb_ep_type_t;

typedef enum {
    USB_FS_DXEPCTL_SD0PID_SEVNFRM  = 0,
    USB_FS_DXEPCTL_SD1PID_SODDFRM
} usb_ep_toggle_t;

#endif

#ifdef CONFIG_USR_DRV_USB_HS 

# define MAX_DATA_PACKET_SIZE 512
typedef enum {
    USB_HS_DXEPCTL_EP0 = 0,
    USB_HS_DXEPCTL_EP1 = 1,
    USB_HS_DXEPCTL_EP2 = 2,
    USB_HS_DXEPCTL_EP3 = 3,
} usb_ep_nb_t;

typedef enum {
    USB_HS_D0EPCTL_MPSIZ_64BYTES = 0,
    USB_HS_D0EPCTL_MPSIZ_32BYTES = 1,
    USB_HS_D0EPCTL_MPSIZ_16BYTES = 2,
    USB_HS_D0EPCTL_MPSIZ_8BYTES  = 3,
    USB_HS_DXEPCTL_MPSIZ_64BYTES = 64,
    USB_HS_DXEPCTL_MPSIZ_128BYTES = 128,
    USB_HS_DXEPCTL_MPSIZ_512BYTES = 512,
    USB_HS_DXEPCTL_MPSIZ_1024BYTES  = 1024,
} usb_ep_mpsize_t;

typedef enum {
    USB_HS_DXEPCTL_EPTYP_CONTROL = 0,
    USB_HS_DXEPCTL_EPTYP_ISOCHRO = 1,
    USB_HS_DXEPCTL_EPTYP_BULK    = 2,
    USB_HS_DXEPCTL_EPTYP_INT     = 3,
} usb_ep_type_t;

typedef enum {
    USB_HS_DXEPCTL_SD0PID_SEVNFRM  = 0,
    USB_HS_DXEPCTL_SD1PID_SODDFRM
} usb_ep_toggle_t;

#endif



#endif /* !_USB_CONTROL_H */
