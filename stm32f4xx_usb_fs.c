/** @file stm32f4xx_usb_fs.c
 * \brief Driver for STM32F4x5/4x7 device
 *
 * This driver is written according to STM32F4x5_4x7 reference manual
 * (ref. RM0090) Rev 11.
 **/

//#include "cortex_m_functions.h"
//#include "debug.h"
//#include "stm32f4xx_exti.h"
#include "autoconf.h"
#ifdef CONFIG_USR_DRV_USB_FS
#include "libc/syscall.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libc/sanhandlers.h"
#include "generated/usb_otg_fs.h"

/*
 * This should be replaced in Kconfig and source:
 * - both fs and hs mode in the same driver
 */
#include "stm32f4xx_usb_fs.h"

#include "stm32f4xx_usb_fs.h"
#include "stm32f4xx_usb_fs_regs.h"
#include "api/usb_control.h"
#include "api/usb.h"
#define ZERO_LENGTH_PACKET 0
#define OUT_NAK		0x01
#define DataOUT		0x02
#define Data_Done	0x03
#define SETUP_Done	0x04
#define SETUP		0x06

#define USB_REG_CHECK_TIMEOUT 50

#define USB_FS_RX_FIFO_SZ       128
#define USB_FS_TX_FIFO_SZ       128

#if USB_FS_DEBUG
#define log_printf(...) aprintf(__VA_ARGS__)
#else
#define log_printf(...) {};
#endif

/**
 * \struct setup_packet
 * \brief Setup packet, used to transfer data on EP0
 */
static struct {
	void (*data_sent_callback)(void);
	void (*data_received_callback)(uint32_t);
} usb_fs_callbacks;

static uint8_t 	setup_packet[8];


static volatile uint8_t *buffer_ep0;
static volatile uint32_t buffer_ep0_idx;
static volatile uint32_t buffer_ep0_size;
static volatile uint8_t *buffer_ep1;
static volatile uint32_t buffer_ep1_idx;
static volatile uint32_t buffer_ep1_size;


void OTG_FS_IRQHandler(uint8_t irq __UNUSED, // IRQ number
                       uint32_t sr,  // content of posthook.status,
                       uint32_t dr);  // content of posthook.data)

#define GPIO_AF10_OTG_FS	0xA

#define USB_MAX_ISR 32

static usb_ep_error_t usb_fs_driver_TXFIFO_flush(uint8_t ep){
    uint32_t count = 0;
	/* Select which ep to flush and do it
     * This is the FIFO number that must be flushed using the TxFIFO Flush bit.
     * This field must not be changed until the core clears the TxFIFO Flush bit.
     */
    count = 0;
	while (get_reg(r_CORTEX_M_USB_FS_GRSTCTL, USB_FS_GRSTCTL_TXFFLSH)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! Waiting for the core to clear the TxFIFO Flush bit GRSTCTL:TXFFLSH\n");
        return USB_ERROR_BUSY;
    }
    /*
     * The application must write this bit only after checking that the core is neither writing to the
     * TxFIFO nor reading from the TxFIFO. Verify using these registers:
     */

    /* FIXME Read: the NAK effective interrupt ensures the core is not reading from the FIFO */

    /* Write: the AHBIDL bit in OTG_FS_GRSTCTL ensures that the core is not writing anything to the FIFO */
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, ep, USB_FS_GRSTCTL_TXFNUM);
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_TXFFLSH);
    count = 0;
	while (get_reg(r_CORTEX_M_USB_FS_GRSTCTL, USB_FS_GRSTCTL_TXFFLSH)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! Waiting for the core to clear the TxFIFO Flush bit GRSTCTL:TXFFLSH\n");
        return USB_ERROR_BUSY;
    }
    return 0;
}


static usb_ep_error_t usb_fs_driver_TXFIFO_flush_all(void){
        unsigned int ep;
        usb_ep_error_t ret;

        /* [RB]: FIXME: put a macro defining the number of endpoints */
        for(ep = 0; ep < 4; ep++){
                if((ret = usb_fs_driver_TXFIFO_flush(ep)) != USB_OK){
                        return ret;
                }
        }
        return USB_OK;
}


static usb_ep_error_t usb_fs_driver_RXFIFO_flush(void){
    uint32_t count = 0;
	/* Select which ep to flush and do it
     * This is the FIFO number that must be flushed using the TxFIFO Flush bit.
     * This field must not be changed until the core clears the TxFIFO Flush bit.
     */
    count = 0;
	while (get_reg(r_CORTEX_M_USB_FS_GRSTCTL, USB_FS_GRSTCTL_RXFFLSH)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! Waiting for the core to clear the TxFIFO Flush bit GRSTCTL:RXFFLSH\n");
        return USB_ERROR_BUSY;
    }
    /*
     * The application must write this bit only after checking that the core is neither writing to the
     * RxFIFO nor reading from the RxFIFO. Verify using these registers:
     */

    /* FIXME Read: the NAK effective interrupt ensures the core is not reading from the FIFO */

    /* Write: the AHBIDL bit in OTG_FS_GRSTCTL ensures that the core is not writing anything to the FIFO */
	//set_reg(r_CORTEX_M_USB_FS_GRSTCTL, ep, USB_FS_GRSTCTL_RXFNUM);
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_RXFFLSH);
    count = 0;
	while (get_reg(r_CORTEX_M_USB_FS_GRSTCTL, USB_FS_GRSTCTL_RXFFLSH)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! Waiting for the core to clear the RxFIFO Flush bit GRSTCTL:TXFFLSH\n");
        return USB_ERROR_BUSY;
    }
    return 0;
}



void usb_fs_driver_device_connect(void){
    set_reg(r_CORTEX_M_USB_FS_DCTL, 0, USB_FS_DCTL_SDIS);
}

/**
 *
 * The powered state can be exited by software with the soft disconnect feature. The DP pullup
 * resistor is removed by setting the soft disconnect bit in the device control register (SDIS
 * bit in OTG_FS_DCTL), causing a device disconnect detection interrupt on the host side
 * even though the USB cable was not really removed from the host port.
 */
void usb_fs_driver_device_disconnect(void){
    set_reg(r_CORTEX_M_USB_FS_DCTL, 1, USB_FS_DCTL_SDIS);
}


static const char *name = "usb-otg-fs";

static uint8_t usb_device_early_init(void) {

    e_syscall_ret ret = 0;
    device_t dev;
    memset((void*)&dev, 0, sizeof(device_t));
    int      dev_desc = 0;

    memcpy(dev.name, name, strlen(name));
    dev.address = USB_OTG_FS_BASE;
    dev.size = 0x4000;
    dev.irq_num = 1;

    /* IRQ configuration */
    dev.irqs[0].handler = OTG_FS_IRQHandler;
    dev.irqs[0].irq = OTG_FS_IRQ; /* starting with STACK */
    dev.irqs[0].mode = IRQ_ISR_FORCE_MAINTHREAD; /* if ISR force MT immediat execution, use FORCE_MAINTHREAD instead of STANDARD, and activate FISR permission */

    /*
     * IRQ posthook configuration
     * The posthook is executed at the end of the IRQ handler mode, *before* the ISR.
     * It permit to clean potential status registers (or others) that may generate IRQ loops
     * while the ISR has not been executed.
     * register read can be saved into 'status' and 'data' and given to the ISR in 'sr' and 'dr' argument
     */
    dev.irqs[0].posthook.status = 0x0014; /* SR is first read */
    dev.irqs[0].posthook.data = 0x0018; /* Data reg  is 2nd read */


    dev.irqs[0].posthook.action[0].instr = IRQ_PH_READ;
    dev.irqs[0].posthook.action[0].read.offset = 0x0014;


    dev.irqs[0].posthook.action[1].instr = IRQ_PH_READ;
    dev.irqs[0].posthook.action[1].read.offset = 0x0018;


    dev.irqs[0].posthook.action[2].instr = IRQ_PH_MASK;
    dev.irqs[0].posthook.action[2].mask.offset_dest = 0x14; /* MASK register offset */
    dev.irqs[0].posthook.action[2].mask.offset_src = 0x14; /* MASK register offset */
    dev.irqs[0].posthook.action[2].mask.offset_mask = 0x18; /* MASK register offset */
    dev.irqs[0].posthook.action[2].mask.mode = 0; /* no binary inversion */


    dev.irqs[0].posthook.action[3].instr = IRQ_PH_AND;
    dev.irqs[0].posthook.action[3].and.offset_dest = 0x18; /* MASK register offset */
    dev.irqs[0].posthook.action[3].and.offset_src = 0x14; /* MASK register offset */
    dev.irqs[0].posthook.action[3].and.mask = USB_FS_GINTMSK_RXFLVLM_Msk; /* MASK register offset */
    dev.irqs[0].posthook.action[3].and.mode = 1; /* binary inversion */


    dev.irqs[0].posthook.action[4].instr = IRQ_PH_AND;
    dev.irqs[0].posthook.action[4].and.offset_dest = 0x18; /* MASK register offset */
    dev.irqs[0].posthook.action[4].and.offset_src = 0x14; /* MASK register offset */
    dev.irqs[0].posthook.action[4].and.mask = USB_FS_GINTMSK_IEPINT_Msk; /* MASK register offset */
    dev.irqs[0].posthook.action[4].and.mode = 1; /* binary inversion */


    dev.irqs[0].posthook.action[5].instr = IRQ_PH_AND;
    dev.irqs[0].posthook.action[5].and.offset_dest = 0x18; /* MASK register offset */
    dev.irqs[0].posthook.action[5].and.offset_src = 0x14; /* MASK register offset */
    dev.irqs[0].posthook.action[5].and.mask = USB_FS_GINTMSK_OEPINT_Msk; /* MASK register offset */
    dev.irqs[0].posthook.action[5].and.mode = 1; /* binary inversion */



    /* Now let's configure the GPIOs */
    dev.gpio_num = 4;

	/* SOF -> PA 8 */
    dev.gpios[0].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    dev.gpios[0].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_SOF].port;
    dev.gpios[0].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_SOF].pin;
    dev.gpios[0].mode         = GPIO_PIN_ALTERNATE_MODE;
    dev.gpios[0].pupd         = GPIO_NOPULL;
    dev.gpios[0].type         = GPIO_PIN_OTYPER_PP;
    dev.gpios[0].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    dev.gpios[0].afr          = GPIO_AF_OTG_FS;
	/* VBUS -> PA 9 */
    dev.gpios[1].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    dev.gpios[1].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_VBUS].port;
    dev.gpios[1].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_VBUS].pin;
    dev.gpios[1].mode         = GPIO_PIN_INPUT_MODE;
    dev.gpios[1].pupd         = GPIO_NOPULL;
    dev.gpios[1].type         = GPIO_PIN_OTYPER_PP;
    dev.gpios[1].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    dev.gpios[1].afr          = GPIO_AF_OTG_FS;

	/* DM pin -> PA11 */
    dev.gpios[2].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    dev.gpios[2].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_DM].port;
    dev.gpios[2].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_DM].pin;
    dev.gpios[2].mode         = GPIO_PIN_ALTERNATE_MODE;
    dev.gpios[2].pupd         = GPIO_NOPULL;
    dev.gpios[2].type         = GPIO_PIN_OTYPER_PP;
    dev.gpios[2].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    dev.gpios[2].afr          = GPIO_AF_OTG_FS;

	/* DP pin -> PA12 */
    dev.gpios[3].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    dev.gpios[3].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_DP].port;
    dev.gpios[3].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_DP].pin;
    dev.gpios[3].mode         = GPIO_PIN_ALTERNATE_MODE;
    dev.gpios[3].pupd         = GPIO_NOPULL;
    dev.gpios[3].type         = GPIO_PIN_OTYPER_PP;
    dev.gpios[3].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    dev.gpios[3].afr          = GPIO_AF_OTG_FS;

    ret = sys_init(INIT_DEVACCESS, &dev, &dev_desc);
    return ret;
}


static uint16_t size_from_mpsize(usb_ep_t *ep){
    if(ep == NULL){
        return 0;
    }
    if (ep->max_packet_size == USB_FS_D0EPCTL_MPSIZ_64BYTES){
        return 64;
    }
    if (ep->max_packet_size == USB_FS_D0EPCTL_MPSIZ_32BYTES){
        return 32;
    }
    if (ep->max_packet_size == USB_FS_D0EPCTL_MPSIZ_16BYTES){
        return 16;
    }
    if (ep->max_packet_size == USB_FS_D0EPCTL_MPSIZ_8BYTES){
        return 8;
    }
    return ep->max_packet_size;
}





/**
 * \brief IN Endpoint activation
 *
 *    This section describes the steps required to activate a device endpoint or to configure an
 *    existing device endpoint to a new type.
 *    1. Program the characteristics of the required endpoint into the following fields of the
 *    OTG_FS_DIEPCTLx register (for IN or bidirectional endpoints)
 *      – Maximum packet size
 *      – USB active endpoint = 1
 *      – Endpoint start data toggle (for interrupt and bulk endpoints)
 *      – Endpoint type
 *      – TxFIFO number
 *
 *    2. Once the endpoint is activated, the core starts decoding the tokens addressed to that
 *    endpoint and sends out a valid handshake for each valid token received for the
 *    endpoint.
 *
 */
usb_ep_error_t usb_fs_driver_in_endpoint_activate(usb_ep_t *ep)
{
    /* Sanitization check */
    if (!ep) {
        log_printf("EP%d is not initialized \n", ep->num);
        return USB_ERROR_BAD_INPUT;
    }

    /* Checking if enpoint is already active */
    if (get_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), USB_FS_DIEPCTL_EPENA)) {
        return USB_ERROR_ALREADY_ACTIVE;
    }

	/* Maximum packet size */
    if ((size_from_mpsize(ep) <= 0) || size_from_mpsize(ep) > MAX_DATA_PACKET_SIZE(ep->num)) {
        log_printf("EP%d bad maxpacket size: %d\n", ep->num, size_from_mpsize(ep));
        return USB_ERROR_RANGE;
    }

	set_reg_value(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), ep->max_packet_size,
                                                      USB_FS_DIEPCTL_MPSIZ_Msk(ep->num),
                                                      USB_FS_DIEPCTL_MPSIZ_Pos(ep->num));
    /* Define endpoint type */
    if (ep->type == USB_FS_DXEPCTL_EPTYP_ISOCHRO) {
        log_printf("EP%d Isochronous is not suported yet\n", ep->num);
        return USB_ERROR_NOT_SUPORTED;
    }

	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), ep->type, USB_FS_DIEPCTL_EPTYP);


    /* Endpoint start data toggle (for interrupt and bulk endpoints)
     * The application uses the SD0PID/SD1PID register fields to program either DATA0 or DATA1 PID.
     *   0: DATA0
     *   1: DATA1
     */
    if ((ep->type == USB_FS_DXEPCTL_EPTYP_INT) || (ep->type == USB_FS_DXEPCTL_EPTYP_BULK)){
        if (ep->start_data_toggle == USB_FS_DXEPCTL_SD0PID_SEVNFRM) {
	        set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), 1, USB_FS_DIEPCTL_SD0PID);
        }else{
            set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), 1, USB_FS_DIEPCTL_SD1PID);
        }
    }

    /* USB active endpoint
     * Indicates whether this endpoint is active in the current configuration and interface. The core
     * clears this bit for all endpoints (other than EP 0) after detecting a USB reset. After receiving
     * the SetConfiguration and SetInterface commands, the application must program endpoint registers
     * accordingly and set this bit.
     */
	set_reg_bits(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), USB_FS_DIEPCTL_USBAEP_Msk);


    /*  IN endpoint FIFOx transmit RAM start address
     *  XXX sould be set by user ?
     */
     /* FIXME add an allocator allowing to compact memory without holes*/
	set_reg(r_CORTEX_M_USB_FS_DIEPTXF(ep->num), (128 * 4)*ep->num + (128 * 4)*2, USB_FS_DIEPTXF_INEPTXSA);


    /*  IN endpoint TxFIFO depth
     *  XXX sould be set by user ?
     */
	set_reg(r_CORTEX_M_USB_FS_DIEPTXF(ep->num), 128, USB_FS_DIEPTXF_INEPTXFD);

    /* Clearing the NAK bit for the endpoint */
    set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep->num), ep->num, USB_FS_DIEPCTL_CNAK);


    /* Unmask IN endpoint global interrupt */
	set_reg_bits(r_CORTEX_M_USB_FS_GINTMSK, USB_FS_GINTMSK_IEPINT_Msk);

    /* IN EP interrupt mask bits
     *   The OTG_FS_DAINTMSK register works with the Device endpoint interrupt register to
     *   interrupt the application when an event occurs on a device endpoint.
     *   However, the OTG_FS_DAINT register bit corresponding to that interrupt is still set.
     */
	write_reg_value(r_CORTEX_M_USB_FS_DAINTMSK, USB_FS_DAINTMSK_IEPM(ep->num));

    return USB_OK;
}

/**
 * \brief Out Endpoint deactivation
 *
 *    This section describes the steps required to deactivate an existing endpoint.
 *    1. In the endpoint to be deactivated, clear the USB active endpoint bit in the
 *       OTG_FS_DIEPCTLx register (for IN or bidirectional endpoints) or the
 *       OTG_FS_DOEPCTLx register (for OUT or bidirectional endpoints).
 *    2. Once the endpoint is deactivated, the core ignores tokens addressed to that endpoint,
 *       which results in a timeout on the USB.
 *
 *    Note: The application must meet the following conditions to set up the device core to handle
 *          traffic: NPTXFEM and RXFLVLM in the OTG_FS_GINTMSK register must be cleared
 */
usb_ep_error_t usb_fs_driver_out_endpoint_deactivate(uint8_t ep){

    /* Sanitization check */
    if ((ep <= 0) || (ep > 3)) {
        return USB_ERROR_BAD_INPUT;
    }

    /* Checking if enpoint is active */
    if (get_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), USB_FS_DOEPCTL_EPENA)) {
        set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_EPENA);
        set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_SNAK);
	    set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_EPDIS);
    }
    return USB_OK;
}

/**
 * \brief IN Endpoint deactivation
 *
 *    This section describes the steps required to deactivate an existing endpoint.
 *    1. In the endpoint to be deactivated, clear the USB active endpoint bit in the
 *       OTG_FS_DIEPCTLx register (for IN or bidirectional endpoints) or the
 *       OTG_FS_DOEPCTLx register (for OUT or bidirectional endpoints).
 *    2. Once the endpoint is deactivated, the core ignores tokens addressed to that endpoint,
 *       which results in a timeout on the USB.
 *
 *    Note: The application must meet the following conditions to set up the device core to handle
 *          traffic: NPTXFEM and RXFLVLM in the OTG_FS_GINTMSK register must be cleared
 */
usb_ep_error_t usb_fs_driver_in_endpoint_deactivate(uint8_t ep){

    /* Sanitization check */
    if ((ep <= 0) || (ep > 3)) {
        return USB_ERROR_BAD_INPUT;
    }

    /* Checking if enpoint is active */
    if (get_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), USB_FS_DIEPCTL_EPENA)) {
        set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_EPENA);
	    set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_SNAK);
	    set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_EPDIS);

    }
    return USB_OK;
}



/**
 * \brief Nack OUT endpoint
 *
 * @param ep Endpoint
 */
void usb_fs_driver_global_nack_out(){
	/* Put core in Global OUT NAK mode
     *   A write to this field sets the Global OUT NAK.
     *   The application uses this bit to send a NAK handshake on all OUT endpoints.
     *
     *   FIXME The application must set the this bit only after making sure that the Global OUT NAK
     *   effective bit in the Core interrupt register (GONAKEFF bit in OTG_FS_GINTSTS) is cleared.
     */
	set_reg(r_CORTEX_M_USB_FS_DCTL, 1, USB_FS_DCTL_SGONAK);

}


/**
 * \brief Nack OUT endpoint
 *
 * @param ep Endpoint
 */
void usb_fs_driver_clear_global_nack_out(){
	/* A write to this field clears the Global OUT NAK. */
	set_reg(r_CORTEX_M_USB_FS_DCTL, 1, USB_FS_DCTL_CGONAK);

}


/**
 * \brief Nack IN endpoint
 *
 * @param ep Endpoint
 */
void usb_fs_driver_global_nack_in(){
	/* Set global IN NAK
     *   A write to this field sets the Global non-periodic IN NAK.The application uses this bit to send
     *   a NAK handshake on all non-periodic IN endpoints.
     *   The application must set this bit only after making sure that the Global IN NAK effective bit
     *   in the Core interrupt register (GINAKEFF bit in OTG_FS_GINTSTS) is cleared.
     */
	set_reg(r_CORTEX_M_USB_FS_DCTL, 1, USB_FS_DCTL_SGINAK);
}


/**
 * \brief Nack IN endpoint
 *
 * @param ep Endpoint
 */
void usb_fs_driver_clear_global_nack_in(){

	/* A write to this field clears the Global IN NAK. */
	set_reg(r_CORTEX_M_USB_FS_DCTL, 1, USB_FS_DCTL_CGINAK);

}


/**
 * \brief Stall OUT endpoint
 *
 * @param ep Endpoint
 */
void usb_fs_driver_stall_out(uint8_t ep){

    /* Wait for current transmission to END */
    while (get_reg_value(r_CORTEX_M_USB_FS_DOEPCTL(ep), USB_FS_DOEPCTL_EPENA_Msk, USB_FS_DOEPCTL_EPENA_Pos)){
        continue; //FIXME TIMEOUT
    }
	/* Disable OUT EP and set STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_EPDIS);
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_STALL);

	/* Clear STALL bit when app is ready */
	/* On Ctrl 0, this is done when request or data are received */
}


/**
 * \brief Stall IN endpoint
 *
 * @param ep Endpoint
 */
usb_ep_error_t usb_fs_driver_stall_in(uint8_t ep){

    /* Wait for current transmission to END */
    //while (get_reg_value(r_CORTEX_M_USB_FS_DIEPCTL(ep), USB_FS_DIEPCTL_EPENA_Msk, USB_FS_DIEPCTL_EPENA_Pos)){
    //    continue; //FIXME TIMEOUT
    //}
    int count = 0;
	while (get_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), USB_FS_DIEPCTL_EPENA)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! DIEPCTL:EPENA\n");
        return USB_ERROR_BUSY;
    }

	/* Disable IN EP and set STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_EPDIS);
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_STALL);

	/* Assert on Endpoint Disabled interrupt */
	assert(read_reg_value(r_CORTEX_M_USB_FS_DIEPINT(ep)) & USB_FS_DIEPINT_EPDISD_Msk);

	/* Flush transmit FIFO
	 * p1279 Rev14 RM0090
	 * Read NAK Eff Int and write AHBIL bit
	 */
	//while (read_reg_value(r_CORTEX_M_USB_FS_GINTSTS) & USB_FS_GINTSTS_GINAKEFF_Msk);
    count = 0;
	while (get_reg(r_CORTEX_M_USB_FS_GINTSTS, USB_FS_GINTSTS_GINAKEFF)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! GINTSTS:GINAKEFF\n");
        return USB_ERROR_BUSY;
    }

	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_AHBIDL);

	/* Select which ep to flush and do it */
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, ep, USB_FS_GRSTCTL_TXFNUM);
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_TXFFLSH);

	/* Clear STALL bit */
	/* On Ctrl 0, this is done when request is received */
    return 0;
}


/**
 *
 */
void usb_fs_driver_stall_in_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle){

	/* Clear STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 0, USB_FS_DIEPCTL_STALL);

    /* Reset PID */
    if ((type == USB_FS_DXEPCTL_EPTYP_INT) || (type == USB_FS_DXEPCTL_EPTYP_BULK)){
        if (start_data_toggle == USB_FS_DXEPCTL_SD0PID_SEVNFRM) {
	        set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_SD0PID); /* DATA0 */
        }else{
            set_reg(r_CORTEX_M_USB_FS_DIEPCTL(ep), 1, USB_FS_DIEPCTL_SD1PID); /* DATA1 */
        }
    }
}

/**
 *
 */
void usb_fs_driver_stall_out_clear(uint8_t ep, uint8_t type, uint8_t start_data_toggle){

	/* Clear STALL bit */
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 0, USB_FS_DIEPCTL_STALL);

    /* Reset PID */
    if ((type == USB_FS_DXEPCTL_EPTYP_INT) || (type == USB_FS_DXEPCTL_EPTYP_BULK)){
        if (start_data_toggle == USB_FS_DXEPCTL_SD0PID_SEVNFRM) {
	        set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_SD0PID); /* DATA0 */
        }else{
            set_reg(r_CORTEX_M_USB_FS_DOEPCTL(ep), 1, USB_FS_DOEPCTL_SD1PID); /* DATA1 */
        }
    }
}

/**
 * \brief USB On-the-Go core RESET
 *
 */
static usb_ep_error_t usb_otg_fs_core_reset(void)
{

    int count = 0;
	/* Wait for AHB master idle */
	while (!get_reg(r_CORTEX_M_USB_FS_GRSTCTL, USB_FS_GRSTCTL_AHBIDL)){
        if (++count > USB_REG_CHECK_TIMEOUT)
		    log_printf("HANG! AHB Idle GRSTCTL:AHBIDL\n");
        return USB_ERROR_BUSY;
    }

	/* Core soft reset */
	set_reg(r_CORTEX_M_USB_FS_GRSTCTL, 1, USB_FS_GRSTCTL_CSRST);
	while (get_reg(r_CORTEX_M_USB_FS_GRSTCTL, USB_FS_GRSTCTL_CSRST)){
        if (++count > USB_REG_CHECK_TIMEOUT){
		    log_printf("HANG! Core Soft RESET\n");
        }
        return USB_ERROR_BUSY;
    }

	/* Wait for 3 PHY Clocks */
	unsigned int i;
	for (i = 0; i < 0xff; i++)
		continue;
    return 0;
}


/**
 * \brief Core initialization.
 *
 * See section 34.17.1 of the reference manual
 * The first thing to do is to active the PHY's transceiver (cf. 34.8)
 *
 * The application must perform the core initialization sequence. If the cable is connected
 * during power-up, the current mode of operation bit in the OTG_FS_GINTSTS (CMOD bit in
 * OTG_FS_GINTSTS) reflects the mode. The OTG_FS controller enters host mode when an
 * “A” plug is connected or device mode when a “B” plug is connected.
 *
 * This section explains the initialization of the OTG_FS controller after power-on. The
 * application must follow the initialization sequence irrespective of host or device mode
 * operation. All core global registers are initialized according to the core’s configuration:
 *
 * 1. Program the following fields in the OTG_FS_GAHBCFG register:
 *      – Global interrupt mask bit GINTMSK = 1
 *      – RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
 *      – Periodic TxFIFO empty level (can be enabled only when the core is operating in Slave mode)
 *
 * 2. Program the following fields in the OTG_FS_GUSBCFG register:
 *      – HNP capable bit
 *      – SRP capable bit
 *      – FS timeout calibration field
 *      – USB turnaround time field
 *
 * 3. The software must unmask the following bits in the OTG_FS_GINTMSK register:
 *      - OTG interrupt mask
 *      - Mode mismatch interrupt mask
 *
 * 4. The software can read the CMOD bit in OTG_FS_GINTSTS to determine whether the
 * OTG_FS controller is operating in host or device mode.
 */
static void usb_otg_fs_core_init(void)
{

    /* The first thing to do is to active the PHY's transceiver (cf. 34.8) : We are using the internal FS PHY */
	set_reg(r_CORTEX_M_USB_FS_GCCFG, 1, USB_FS_GCCFG_PWRDWN);
    set_reg(r_CORTEX_M_USB_FS_GUSBCFG, 1, USB_FS_GUSBCFG_PHYSEL); /* Full Speed serial transceiver select */

    /* In device mode, just after Power On Reset or a Soft Reset,
     * the GINTSTS.Sof bit is set to 1'b1 for debug purposes.
     * This status must be cleared and can be ignored.
     */
    clear_reg_bits(r_CORTEX_M_USB_FS_GINTSTS, USB_FS_GINTSTS_SOF_Msk);


	/* [MR] TODO Read the User Hardware Configuration registers (GHWCFG1, 2, 3, and 4)
       to find the configuration parameters selected for DWC_otg core */

    /* Program the following fields in the Global AHB Configuration (GAHBCFG) register.
     *      - Global Interrupt Mask bit = 1
     *      - Periodic TxFIFO Empty Level (can be enabled only when the core is operating in Slave mode)
     */

	set_reg(r_CORTEX_M_USB_FS_GAHBCFG, 1, USB_FS_GAHBCFG_GINTMSK); /* Unmask the Global interrupt assertion to the application. */

    /* Non-periodic TxFIFO Empty Level (can be enabled only when the core is operating in Slave mode
     * as a host or as a device in Shared FIFO operation.)
     * Set the TXFELVL bit in the GAHBCFG register so that TxFIFO
     * interrupts will occur when the TxFIFO is truly empty (not just half full).
      */
	set_reg(r_CORTEX_M_USB_FS_GAHBCFG, 0, USB_FS_GAHBCFG_TXFELVL);

    /* FIXME GAHBCFG Periodic TxFIFO empty level (PTXFELVL) is not set because it is only accessible in host mode.*/

    /*  RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS) */
	set_reg(r_CORTEX_M_USB_FS_GINTMSK, 1, USB_FS_GINTMSK_RXFLVLM); /* Unmask Receive FIFO non-empty */

	set_reg(r_CORTEX_M_USB_FS_GUSBCFG, 0, USB_FS_GUSBCFG_HNPCAP); /* FIXME Device Only Configuration :  HNP capability is enabled. */

	set_reg(r_CORTEX_M_USB_FS_GUSBCFG, 0, USB_FS_GUSBCFG_SRPCAP); /* FIXME Device Only Configuration :  SRP capability is not enabled. */

	set_reg(r_CORTEX_M_USB_FS_GUSBCFG, 0, USB_FS_GUSBCFG_TOCAL); /* FS timeout calibration */
                       /*    The number of PHY clocks that the application programs in this field is added to the fullspeed
                        *    interpacket timeout duration in the core to account for any additional delays
                        *    introduced by the PHY. This can be required, because the delay introduced by the PHY in
                        *    generating the line state condition can vary from one PHY to another.
                        *    The USB standard timeout value for full-speed operation is 16 to 18 (inclusive) bit times.
                        *    FIXME The application must program this field based on the speed of enumeration.
                        *    The number of bit times added per PHY clock is 0.25 bit times.
                        */

	/* PHY clock is the same as AHB clock */
	set_reg(r_CORTEX_M_USB_FS_GUSBCFG, 6, USB_FS_GUSBCFG_TRDT); /* USB turnaround time see TRDT values (DocID018909 Rev 15) */
                        /*  These bits allow setting the turnaround time in PHY clocks. They must be configured
                         *  according to Table 201: TRDT values, depending on the application AHB frequency.
                         *  Higher TRDT values allow stretching the USB response time to IN tokens in order
                         *  to compensate for longer AHB read access latency to the Data FIFO.
                         *   FIXME Only accessible in device mode.
                         */
	set_reg(r_CORTEX_M_USB_FS_GINTMSK, 1, USB_FS_GINTMSK_MMISM); /* Unmask  Mode mismatch interrupt */

    /* [MR] TODO The software can read the CMOD bit in OTG_FS_GINTSTS to determine whether the
     * OTG_FS controller is operating in host or device mode.
     */
}

/**
 *\brief Device initialization.
 *
 * See section 34.17.3 of the reference manual
 * The application must perform the following steps to initialize the core as a device on powerup
 * or after a mode change from host to device.
 *
 *
 * 1. Program the following fields in the OTG_FS_DCFG register:
 *      – Device speed
 *      – Non-zero-length status OUT handshake
 *      - Periodic Frame Interval (If Periodic Endpoints are supported)
 *                       FIXME PERIODIC ENDPPOINTS NOT SUPPORTED YET
 *                       See 7.1 Device Initialization (USB 2.0 Hi-Speed On-The-Go (OTG) Programmer’s Guide)
 * 2. Program the Device threshold control register. This is required only if you are using DMA mode and
 *    you are planning to enable thresholding. /!\ NOT USED HERE

 *
 * 3. Program the OTG_FS_GINTMSK register to unmask the following interrupts:
 *      – USB reset
 *      – Enumeration done
 *      – Early suspend
 *      – USB suspend
 *      – SOF
 *
 * 4. Clear the DCTL.SftDiscon bit. The core issues a connect after this bit is cleared.
 *                      See 7.1 Device Initialization (USB 2.0 Hi-Speed On-The-Go (OTG) Programmer’s Guide)
 *
 * 5. Program the VBUSBSEN bit in the OTG_FS_GCCFG register to enable VBUS sensing
 *    in “B” device mode and supply the 5 volts across the pull-up resistor on the DP line.
 *
 * 6. Wait for the USBRST interrupt in OTG_FS_GINTSTS. It indicates that a reset has been
 *    detected on the USB that lasts for about 10 ms on receiving this interrupt.
 *
 * 7. Wait for the ENUMDNE interrupt in OTG_FS_GINTSTS. This interrupt indicates the end of
 *    reset on the USB.
 *
 * At this point, the device is ready to accept SOF packets and perform control transfers on
 * control endpoint 0.
 *
 */
static void usb_otg_fs_device_init(void)
{

	set_reg(r_CORTEX_M_USB_FS_DCFG, 3, USB_FS_DCFG_DSPD);     /* Device speed: 3 = Full Speed */
	set_reg(r_CORTEX_M_USB_FS_DCFG, 0, USB_FS_DCFG_NZLSOHSK); /* Non-zero-length status OUT handshake: */
                            /* The application can use this field to select the handshake the core sends on receiving a
                             * nonzero-length data packet during the OUT transaction of a control transfer’s Status stage.
                             * 1: Send a STALL handshake on a nonzero-length status OUT transaction and do not send
                             * the received OUT packet to the application.
                             * 0: Send the received OUT packet to the application (zero-length or nonzero-length) and
                             * send a handshake based on the NAK and STALL bits for the endpoint in the Device
                             * endpoint control register
                             */

	set_reg_bits(r_CORTEX_M_USB_FS_GINTMSK,
                        USB_FS_GINTMSK_USBRST_Msk   | /* USB reset: The core sets this bit to indicate that a reset is detected on the USB. */
						USB_FS_GINTMSK_ENUMDNEM_Msk | /* Unmask Speed Enumeration done interupt */
						USB_FS_GINTMSK_ESUSPM_Msk   | /* Unmask Early suspend mask interupt */
						USB_FS_GINTMSK_USBSUSPM_Msk | /* Unmask USB suspend mask */
						//USB_FS_GINTMSK_SOFM_Msk     | /* Unmask Start of frame */
						USB_FS_GINTMSK_OEPINT_Msk   | /* FIXME Unmask OUT endpoints interrupt */
						USB_FS_GINTMSK_IEPINT_Msk);   /* FIXME Unmask IN endpoints interrupt */

	set_reg(r_CORTEX_M_USB_FS_GCCFG, 1, USB_FS_GCCFG_VBUSBSEN); /* Enable the VBUS sensing “B” device */

    usb_fs_driver_device_connect();
}


static void usb_fs_init_isr_handlers(void);

/**
 * \brief Inititialize USB driver.
 *
 * Launch needeed initialization functions.
 * Before sys_init(INIT_DONE)
 */
void usb_fs_driver_early_init(void (*data_received)(uint32_t), void (*data_sent)(void))
{
	usb_fs_callbacks.data_sent_callback = data_sent;
	usb_fs_callbacks.data_received_callback = data_received;

	usb_device_early_init();

	usb_fs_init_isr_handlers();

	//NVIC_EnableIRQ(OTG_FS_IRQn);
}


static void ep_init_reset(void);

/*
 *
 * After sys_init(INIT_DONE)
 */
void usb_fs_driver_init(void)
{
	usb_otg_fs_core_init();
	set_reg(r_CORTEX_M_USB_FS_GUSBCFG, 1, USB_FS_GUSBCFG_FDMOD); /* FIXME Force device mode */
	usb_otg_fs_device_init();
}


/**
 * \brief Read FIFO.
 *
 * Read data put in the FIFO.
 *
 * @param dest Destination address
 * @param size Size of the data
 * @param ep Endpoint from where data is read
 */
static void _read_fifo(volatile uint8_t *dest, volatile uint32_t size, uint8_t ep)
{
	assert(ep <= 1);
	assert(size <= USB_FS_RX_FIFO_SZ);

        if((size != 0) && (dest == NULL)){
                return;
        }

	uint32_t i = 0;
	uint32_t size_4bytes = size / 4;
    uint32_t tmp;

    uint32_t oldmask = read_reg_value(r_CORTEX_M_USB_FS_GINTMSK);
    set_reg_value(r_CORTEX_M_USB_FS_GINTMSK, 0, 0xffffffff, 0);

    // FIXME: IP should has its own interrupts disable during ISR execution
	//disable_irq();

	for (i = 0; i < size_4bytes; i++, dest += 4){
		*(uint32_t *)dest = *(USB_FS_DEVICE_FIFO(ep));
	}

	switch (size % 4) {
	case 1:
		*dest = (*(USB_FS_DEVICE_FIFO(ep))) & 0xff;
		break;
	case 2:
		*(uint16_t *)dest = (*(USB_FS_DEVICE_FIFO(ep))) & 0xffff;
		break;
	case 3:
		tmp = *(USB_FS_DEVICE_FIFO(ep));
		dest[0] = tmp & 0xff;
		dest[1] = (tmp >> 8) & 0xff;
		dest[2] = (tmp >> 16) & 0xff;
		break;
	default:
		break;
	}

    // FIXME: IP should has its own interrupts disable during ISR execution
	//enable_irq();
    set_reg_value(r_CORTEX_M_USB_FS_GINTMSK, oldmask, 0xffffffff, 0);
}

static void read_fifo(volatile uint8_t *dest, volatile uint32_t size, uint8_t ep)
{
	unsigned int i;
	unsigned int num = size / USB_FS_RX_FIFO_SZ;


	for(i = 0; i < num; i++){
		_read_fifo(dest + (i * USB_FS_RX_FIFO_SZ), USB_FS_RX_FIFO_SZ, ep);
	}
	/* Handle the possible last packet */
	if(size > (num * USB_FS_RX_FIFO_SZ)){
		_read_fifo(dest + (num * USB_FS_RX_FIFO_SZ), size - (num * USB_FS_RX_FIFO_SZ), ep);
	}
}

/**
 * \brief Receive FIFO packet read.
 *
 */
static void usb_fs_driver_rcv_out_pkt(volatile uint8_t *buffer,
                                      volatile uint32_t *buffer_idx,
                                     volatile  uint32_t buffer_size,
                                    volatile uint32_t bcnt,
                                    usb_ep_nb_t epnum)
{
	uint32_t size;
	assert(buffer);
	assert(buffer_idx);
	size = (bcnt + *buffer_idx) >= buffer_size ? (buffer_size - *buffer_idx) : bcnt;
	read_fifo(buffer + *buffer_idx, size, epnum);
	*buffer_idx += size;
}


/*
 * IRQs
 */
void (* usb_fs_isr_handlers[USB_MAX_ISR])(void);
/**
 * \brief Handler for RXFLVL interrupt.
 *
 *   Packet read
 *   This section describes how to read packets (OUT data and SETUP packets) from the
 *   receive FIFO.
 *      1.  On catching an RXFLVL interrupt (OTG_FS_GINTSTS register), the application must
 *          read the Receive status pop register (OTG_FS_GRXSTSP).
 *      2.  The application can mask the RXFLVL interrupt (in OTG_FS_GINTSTS) by writing to
 *          RXFLVL = 0 (in OTG_FS_GINTMSK), until it has read the packet from the receive
 *          FIFO.
 *      3.  If the received packet’s byte count is not 0, the byte count amount of data is popped
 *          from the receive Data FIFO and stored in memory. If the received packet byte count is
 *          0, no data is popped from the receive data FIFO.
 *      4.  The receive FIFO’s packet status readout indicates one of the following:
 *          a) Global OUT NAK pattern:
 *              PKTSTS = Global OUT NAK, BCNT = 0x000, EPNUM = Don’t Care (0x0),
 *              DPID = Don’t Care (0b00).
 *              These data indicate that the global OUT NAK bit has taken effect.
 *          b) SETUP packet pattern:
 *              PKTSTS = SETUP, BCNT = 0x008, EPNUM = Control EP Num, DPID = D0.
 *              These data indicate that a SETUP packet for the specified endpoint is now
 *              available for reading from the receive FIFO.
 *          c) Setup stage done pattern:
 *              PKTSTS = Setup Stage Done, BCNT = 0x0, EPNUM = Control EP Num,
 *              DPID = Don’t Care (0b00).
 *              These data indicate that the Setup stage for the specified endpoint has completed
 *              and the Data stage has started. After this entry is popped from the receive FIFO,
 *              the core asserts a Setup interrupt on the specified control OUT endpoint.
 *          d) Data OUT packet pattern:
 *              PKTSTS = USB_OUT_PACKET_RECEIVED, BCNT = size of the received data OUT packet (0 ≤ BCNT
 *              ≤ 1 024), EPNUM = EPNUM on which the packet was received, DPID = Actual
 *              Data PID.
 *          e) Data transfer completed pattern:
 *              PKTSTS = Data OUT Transfer Done, BCNT = 0x0, EPNUM = OUT EP Num
 *              on which the data transfer is complete, DPID = Don’t Care (0b00).
 *              These data indicate that an OUT data transfer for the specified OUT endpoint has
 *              completed. After this entry is popped from the receive FIFO, the core asserts a
 *              Transfer Completed interrupt on the specified OUT endpoint.
 *      5.  After the data payload is popped from the receive FIFO, the RXFLVL interrupt
 *          (OTG_FS_GINTSTS) must be unmasked.
 *      6.  Steps 1–5 are repeated every time the application detects assertion of the interrupt line
 *          due to RXFLVL in OTG_FS_GINTSTS.
 *
 *      /!\ Reading an empty receive FIFO can result in undefined core behavior.
 *
 */
static void rxflvl_handler(void)
{
	uint32_t grxstsp;
	uint32_t pktsts;
	uint32_t bcnt;
	uint32_t epnum;
	uint32_t dpid;
	uint32_t size;

   	/* 2. Mask the RXFLVL interrupt (in OTG_FS_GINTSTS) by writing to RXFLVL = 0 (in OTG_FS_GINTMSK),
     	 *    until it has read the packet from the receive FIFO
     	 */
	set_reg(r_CORTEX_M_USB_FS_GINTMSK, 0, USB_FS_GINTMSK_RXFLVLM);

 	/* 1. Read the Receive status pop register */
    	grxstsp = read_reg_value(r_CORTEX_M_USB_FS_GRXSTSP);

    	/* FIXME grxstsp content must be interprited diferently in host and device mode.
     	 *       We only support device mode so far.
     	 */
	pktsts = (grxstsp & 0x1e0000) >> 17;// FIXME we should define a macro
	dpid = (grxstsp & 0x18000) >> 15;   // FIXME we should define a macro && XXX Should dpid become a global ?
	bcnt = (grxstsp & 0x7ff0) >> 4;     // FIXME we should define a macro
	epnum = grxstsp & 0xf;       // FIXME we should define a macro
	size = 0;
	//log_printf("EP:%d, PKTSTS:%x, BYTES_COUNT:%x,  DATA_PID:%x\n", epnum, pktsts, bcnt, dpid);


    /* 3. If the received packet’s byte count is not 0, the byte count amount of data is popped
     *    from the receive Data FIFO and stored in memory. If the received packet byte count is
     *    0, no data is popped from the receive data FIFO.
     *
     *   /!\ Reading an empty receive FIFO can result in undefined core behavior.
     */
    if (bcnt != ZERO_LENGTH_PACKET){

        /* 4. The receive FIFO’s packet status readout indicates one of the following: */
            switch (epnum) {
                case USB_FS_DXEPCTL_EP0:
                {
                        /* b) SETUP packet pattern
                         *      These data indicate that a SETUP packet for the specified endpoint is now
                         *      available for reading from the receive FIFO.
                         */
                        if (pktsts == SETUP && bcnt == 0x8 && dpid == 0) {
                                log_printf("EP0 Setup stage pattern\n");
                                read_fifo(setup_packet, 8, epnum);
                                                    /* After this, the Data stage begins.
                                                     * A Setup stage done is received,
                                                     * which triggers a Setup interrupt
                                                     */
                        }
                        /* c) Setup stage done pattern
                        *      These data indicate that the Setup stage for the specified endpoint has completed
                        *      and the Data stage has started. After this entry is popped from the receive FIFO,
                        *      the core asserts a Setup interrupt on the specified control OUT endpoint.
                        */
                        if (pktsts == SETUP_Done){
                             log_printf("EP0 Setup stage done pattern\n");
                         }
                         /* d) Data OUT packet pattern */
                        if (pktsts == DataOUT) {
                                if(buffer_ep0 != NULL){
                                        usb_fs_driver_rcv_out_pkt(buffer_ep0, &buffer_ep0_idx, buffer_ep0_size, bcnt, epnum);
                                }
                                /* In case of EP0, we have to manually check the completion and call the callback */
                                if (buffer_ep0_idx == buffer_ep0_size) {
                                   buffer_ep0 = NULL;
                                   if (usb_fs_callbacks.data_received_callback) {
					    /* Sanity check our callback before calling it */
			  		    if(handler_sanity_check_with_panic((physaddr_t)usb_fs_callbacks.data_received_callback)){
						return;
					    }
				  	  else{
	                                        usb_fs_callbacks.data_received_callback(buffer_ep0_idx);
					  }
                                   }
                                   buffer_ep0_idx = buffer_ep0_size = 0;
                                }
                        }
                        break;
                }
                case USB_FS_DXEPCTL_EP1:
                {
                    /* Data OUT packet pattern on EP1 */
                    if (pktsts == DataOUT && bcnt > 0 && buffer_ep1) {
                        usb_fs_driver_rcv_out_pkt(buffer_ep1, &buffer_ep1_idx, buffer_ep1_size, bcnt, epnum);
                    }
                    break;
                }
                default:
                        log_printf("RXFLVL on bad EP %d!", epnum);
                }
            }

    /* a) Global OUT NAK pattern (triggers an interrupt)
     *      These data indicate that the global OUT NAK bit has taken effect.
     */
    if (pktsts == OUT_NAK){
        log_printf("Global OUT NAK pattern on EP%d\n", epnum);
    }

    /* e) Data transfer completed pattern (triggers an interrupt)
     *      These data indicate that an OUT data transfer for the specified OUT endpoint has
     *      completed. After this entry is popped from the receive FIFO, the core asserts a
     *      Transfer Completed interrupt on the specified OUT endpoint.
     */
    if (pktsts == DataOUT){
        log_printf("OUT Data transfer completed pattern on EP%d\n", epnum);
    }

	set_reg(r_CORTEX_M_USB_FS_GINTMSK, 1, USB_FS_GINTMSK_RXFLVLM);

}


/**
 *\brief Endpoint initialization reset.
 *
 * See section 34.17.5 of the reference manual
 *
 * Endpoint initialization on USB reset
 * 1. Set the NAK bit for all OUT endpoints
 *      – SNAK = 1 in OTG_FS_DOEPCTLx (for all OUT endpoints)
 *
 * 2. Unmask the following interrupt bits
 *      – INEP0 = 1 in OTG_FS_DAINTMSK (control 0 IN endpoint)
 *      – OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
 *      – STUP = 1 in DOEPMSK
 *      – XFRC = 1 in DOEPMSK
 *      – XFRC = 1 in DIEPMSK
 *      – TOC = 1 in DIEPMSK
 *
 * 3. Set up the Data FIFO RAM for each of the FIFOs
 *      – Program the OTG_FS_GRXFSIZ register, to be able to receive control OUT data
 *        and setup data. If thresholding is not enabled, at a minimum, this must be equal to
 *        1 max packet size of control endpoint 0 + 2 Words (for the status of the control
 *        OUT data packet) + 10 Words (for setup packets).
 *      – Program the OTG_FS_TX0FSIZ register (depending on the FIFO number chosen)
 *        to be able to transmit control IN data. At a minimum, this must be equal to 1 max
 *        packet size of control endpoint 0.
 *
 *
 * At this point, all initialization required to receive SETUP packets is done.
 *
 */
static void ep_init_reset(void)
{
#if 1
    // FIXME MR We should check if the ep is IN/OUT
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP0), 1, USB_FS_DOEPCTL_SNAK);
    //set_reg(r_CORTEX_M_USB_FS_DOEPCTL(1), 1, USB_FS_DOEPCTL_SNAK);
#else
    /* 1. Set the NAK bit for all OUT endpoints
     *      – SNAK = 1 in OTG_FS_DOEPCTLx (for all OUT endpoints)
     */
    uint8_t i = 0;
    for (i=0;i<=PROD_MAX_USB_ENDPOINTS;i++){
        set_reg(r_CORTEX_M_USB_FS_DOEPCTL(i), 1, USB_FS_DOEPCTL_SNAK);
    }
#endif

    /*
     * 2. Unmask the following interrupt bits
     *      – INEP0 = 1 in OTG_FS_DAINTMSK (control 0 IN endpoint)
     *      – OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
     *      – STUP = 1 in DOEPMSK
     *      – XFRC = 1 in DOEPMSK
     *      – XFRC = 1 in DIEPMSK
     *      – TOC = 1 in DIEPMSK
     */

	write_reg_value(r_CORTEX_M_USB_FS_DAINTMSK,	USB_FS_DAINTMSK_IEPM(USB_FS_DXEPCTL_EP0)
                                                | USB_FS_DAINTMSK_IEPM(USB_FS_DXEPCTL_EP2)
                                                | USB_FS_DAINTMSK_OEPM(USB_FS_DXEPCTL_EP0)
                                                | USB_FS_DAINTMSK_OEPM(USB_FS_DXEPCTL_EP1));

	set_reg(r_CORTEX_M_USB_FS_DOEPMSK, 1, USB_FS_DOEPMSK_STUPM);
	set_reg(r_CORTEX_M_USB_FS_DOEPMSK, 1, USB_FS_DOEPMSK_XFRCM);
	set_reg(r_CORTEX_M_USB_FS_DIEPMSK, 1, USB_FS_DIEPMSK_XFRCM);
	set_reg(r_CORTEX_M_USB_FS_DIEPMSK, 1, USB_FS_DIEPMSK_TOM);



    /*
     * 3. Set up the Data FIFO RAM for each of the FIFOs
     *      – Program the OTG_FS_GRXFSIZ register, to be able to receive control OUT data
     *        and setup data. If thresholding is not enabled, at a minimum, this must be equal to
     *        1 max packet size of control endpoint 0 + 2 Words (for the status of the control
     *        OUT data packet) + 10 Words (for setup packets).
     *
     * See reference manual section 34.11 for peripheral FIFO architecture.
	 * XXX: The sizes of TX FIFOs seems to be the size of TX FIFO #0 for
	 * all FIFOs. We don't know if it is really the case or if the DTXFSTS
	 * register does not give the free space for the right FIFO.
	 *
	 * 0                512                1024
	 * +-----------------+------------------+-----------------+
	 * |     RX FIFO     |     TX0 FIFO     | TX1 FIFO (EP 2) |
	 * |    128 Words    |    128 Words     |    128 Words    |
	 * +-----------------+------------------+-----------------+
     *
	 */
	set_reg(r_CORTEX_M_USB_FS_GRXFSIZ, USB_FS_RX_FIFO_SZ, USB_FS_GRXFSIZ_RXFD);

    /*      – Program the OTG_FS_TX0FSIZ register (depending on the FIFO number chosen)
     *        to be able to transmit control IN data. At a minimum, this must be equal to 1 max
     *        packet size of control endpoint 0.
     *
     *      XXX: The register 'Enpoint 0 Transmit FIFO size' is called OTG_FS_TX0FSIZ in
     *            section 34.17.5 of the reference manual but it is called OTG_FS_DIEPTXF0 in the section 34.16.2.
     */

     /*
     EndPoint 0 TX FIFO configuration (should store a least 4 64byte paquets
     */
	set_reg(r_CORTEX_M_USB_FS_DIEPTXF0, USB_FS_TX_FIFO_SZ, USB_FS_DIEPTXF_INEPTXSA);
	set_reg(r_CORTEX_M_USB_FS_DIEPTXF0, 64, USB_FS_DIEPTXF_INEPTXFD);

    /*
     * 4. Program STUPCNT in the endpoint-specific registers for control OUT endpoint 0 to receive a SETUP packet
     *      – STUPCNT = 3 in OTG_FS_DOEPTSIZ0 (to receive up to 3 back-to-back SETUP packets)
     */

	set_reg(r_CORTEX_M_USB_FS_DOEPTSIZ(USB_FS_DXEPCTL_EP0), 3, USB_FS_DOEPTSIZ_STUPCNT);
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP0), 1, USB_FS_DOEPCTL_CNAK);
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP0), 1, USB_FS_DOEPCTL_EPENA);

	/*********** Set endpoints 0x82 (IN EP) ************/
#if 0
    usb_driver_in_endpoint_activate(USB_FS_DXEPCTL_EP2,(64 * 4 + 128 * 4), USB_FS_DXEPCTL_EPTYP_BULK, USB_FS_DIEPCTL_SD0PID, 64);
#else
    /*  Transmit FIFO address */
    /* FIXME TxFIFO number */
	set_reg(r_CORTEX_M_USB_FS_DIEPTXF(USB_FS_DXEPCTL_EP2), USB_FS_RX_FIFO_SZ + USB_FS_TX_FIFO_SZ, USB_FS_DIEPTXF_INEPTXSA);
	set_reg(r_CORTEX_M_USB_FS_DIEPTXF(USB_FS_DXEPCTL_EP2), USB_FS_TX_FIFO_SZ, USB_FS_DIEPTXF_INEPTXFD);

    /* Maximum packet size */
	set_reg_value(r_CORTEX_M_USB_FS_DIEPCTL(USB_FS_DXEPCTL_EP2), MAX_DATA_PACKET_SIZE(USB_FS_DXEPCTL_EP2), USB_FS_DIEPCTL_MPSIZ_Msk(USB_FS_DXEPCTL_EP2), USB_FS_DIEPCTL_MPSIZ_Pos(USB_FS_DXEPCTL_EP2));

    /* Start data toggle */
    set_reg(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP2), 1, USB_FS_DIEPCTL_SD0PID);

    /* Endpoint type */
	set_reg(r_CORTEX_M_USB_FS_DIEPCTL(USB_FS_DXEPCTL_EP2), USB_FS_DIEPCTL_EPTYP_BULK, USB_FS_DIEPCTL_EPTYP);

	/*  USB active endpoint XXX: should it be done after a SET_CONFIGURATION ? */
	set_reg_bits(r_CORTEX_M_USB_FS_DIEPCTL(USB_FS_DXEPCTL_EP2), USB_FS_DIEPCTL_USBAEP_Msk);

#endif



	/************ Set endpoint 0x1 (OUT EP) ************/
#if 0
    usb_driver_out_endpoint_activate(USB_FS_DXEPCTL_EP1, //FIFO NUM, USB_FS_DXEPCTL_EPTYP_BULK, USB_FS_DOEPCTL_SD0PID, 64);
#else

    /* Maximum packet size */
	set_reg_value(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP1), MAX_DATA_PACKET_SIZE(USB_FS_DXEPCTL_EP1), USB_FS_DOEPCTL_MPSIZ_Msk(USB_FS_DXEPCTL_EP1), USB_FS_DOEPCTL_MPSIZ_Pos(USB_FS_DXEPCTL_EP1));

    /* FIXME Start data toggle */
    set_reg(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP1), 1, USB_FS_DOEPCTL_SD0PID);

    /* Endpoint type */
	set_reg(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP1), USB_FS_DOEPCTL_EPTYP_BULK, USB_FS_DOEPCTL_EPTYP);

    /*  USB active endpoint */
	set_reg_bits(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP1), USB_FS_DOEPCTL_USBAEP_Msk);

#endif
}


/**
 * \brief Endpoint initialization on enumeration completion.
 *
 *  1. On the Enumeration Done interrupt (ENUMDNE in OTG_FS_GINTSTS), read the OTG_FS_DSTS register
 *     to determine the enumeration speed.
 *  2. Program the MPSIZ field in OTG_FS_DIEPCTL0 to set the maximum packet size. This
 *     step configures control endpoint 0. The maximum packet size for a control endpoint
 *     depends on the enumeration speed.
 *
 *  At this point, the device is ready to receive SOF packets and is configured to perform control
 *  transfers on control endpoint 0
 */
volatile uint32_t badspeed = 0;

static void ep_init_enum(void)
{
	/* 1. read the OTG_FS_DSTS register to determine the enumeration speed. */
	uint8_t speed = get_reg(r_CORTEX_M_USB_FS_DSTS, USB_FS_DSTS_ENUMSPD);
	if (speed != USB_FS_DSTS_ENUMSPD_FS) {
        badspeed++;
		//return;
	}

	////LOG("Enum speed: FS => MAX PACKET SIZE = 64\n");
    /* TODO Program the MPSIZ field in OTG_FS_DIEPCTL0 to set the maximum packet size. This
     * step configures control endpoint 0.
     */
	set_reg_value(r_CORTEX_M_USB_FS_DIEPCTL(USB_FS_DXEPCTL_EP0),
		      USB_FS_DIEPCTL0_MPSIZ_64BYTES,
		      USB_FS_DIEPCTL_MPSIZ_Msk(USB_FS_DXEPCTL_EP0),
		      USB_FS_DIEPCTL_MPSIZ_Pos(USB_FS_DXEPCTL_EP0));
   return;
}


/**
 * \brief Input interrupt handler.
 *
 * Handles an input interrupt.
 * The peripheral core provides the following status checks and interrupt generation:
 * • Transfer completed interrupt, indicating that data transfer was completed on both the application (AHB) and USB sides
 * • Setup stage has been done (control-out only)
 * • Associated transmit FIFO is half or completely empty (in endpoints)
 * • NAK acknowledge has been transmitted to the host (isochronous-in only)
 * • IN token received when Tx-FIFO was empty (bulk-in/interrupt-in only)
 * • Out token received when endpoint was not yet enabled
 * • Babble error condition has been detected
 * • Endpoint disable by application is effective
 * • Endpoint NAK by application is effective (isochronous-in only)
 * • More than 3 back-to-back setup packets were received (control-out only)
 * • Timeout condition detected (control-in only)
 * • Isochronous out packet has been dropped, without generating an interrupt
 */
static volatile unsigned int ep0_last_packet_sent = 0;
static volatile unsigned int ep0_packets_sent = 0;
static void iepint_handler(void)
{
    /*  read the device all endpoints interrupt (OTG_FS_DAINT) register to get the exact endpoint number
     *  for the Device endpoint-x interrupt register.
     */
	uint32_t daint = read_reg_value(r_CORTEX_M_USB_FS_DAINT);

	if (daint & USB_FS_DAINT_IEPINT(USB_FS_DXEPCTL_EP0)) {
		uint32_t diepint0 = read_reg_value(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0));

        /* Bit 7 TXFE: Transmit FIFO empty */
		if (diepint0 & USB_FS_DIEPINT_TOC_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DIEPINT_TXFE_Msk);
		}

        /* Bit 6 INEPNE: IN endpoint NAK effective */
		if (diepint0 & USB_FS_DIEPINT_INEPNE_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DIEPINT_INEPNE_Msk);
		}

        /* Bit 4 ITTXFE: IN token received when TxFIFO is empty */
		if (diepint0 & USB_FS_DIEPINT_ITTXFE_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DIEPINT_ITTXFE_Msk);
		}

        /* Bit 3 TOC: Timeout condition */
		if (diepint0 & USB_FS_DIEPINT_TOC_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DIEPINT_TOC_Msk);
		}

        /* bit 1 EPDISD: Endpoint disabled interrupt */
		if (diepint0 & USB_FS_DIEPINT_EPDISD_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DIEPINT_EPDISD_Msk);
            /* Now the endpiont is really disabled
             * We should update enpoint status
             */
		}

        /* Bit 0 XFRC: Transfer completed interrupt */
		if (diepint0 & USB_FS_DIEPINT_XFRC_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DIEPINT_XFRC_Msk);
                        ep0_packets_sent = 1;
                        /* Our callback */
                        if(ep0_last_packet_sent == 1){
                                if (usb_fs_callbacks.data_sent_callback){
				    /* Sanity check our callback before calling it */
		  		    if(handler_sanity_check_with_panic((physaddr_t)usb_fs_callbacks.data_sent_callback)){
					return;
				    }
				    else{
                                        usb_fs_callbacks.data_sent_callback();
				    }
                                }
                        }
		}
    }

	if (daint & USB_FS_DAINT_IEPINT(USB_FS_DXEPCTL_EP2)) {
		uint32_t diepint2 = read_reg_value(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2));

        /* Bit 7 TXFE: Transmit FIFO empty */
		if (diepint2 & USB_FS_DIEPINT_TOC_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2), USB_FS_DIEPINT_TXFE_Msk);
		}

        /* Bit 6 INEPNE: IN endpoint NAK effective */
		if (diepint2 & USB_FS_DIEPINT_INEPNE_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2), USB_FS_DIEPINT_INEPNE_Msk);
		}

        /* Bit 4 ITTXFE: IN token received when TxFIFO is empty */
		if (diepint2 & USB_FS_DIEPINT_ITTXFE_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2), USB_FS_DIEPINT_ITTXFE_Msk);
		}

        /* Bit 3 TOC: Timeout condition */
		if (diepint2 & USB_FS_DIEPINT_TOC_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2), USB_FS_DIEPINT_TOC_Msk);
		}

        /* bit 1 EP2DISD: Endpoint disabled interrupt */
		if (diepint2 & USB_FS_DIEPINT_EPDISD_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2), USB_FS_DIEPINT_EPDISD_Msk);
		}

        /* Bit 2 XFRC: Transfer completed interrupt */
		if (diepint2 & USB_FS_DIEPINT_XFRC_Msk) {
			set_reg_bits(r_CORTEX_M_USB_FS_DIEPINT(USB_FS_DXEPCTL_EP2), USB_FS_DIEPINT_XFRC_Msk);
			if (usb_fs_callbacks.data_sent_callback) {
				    /* Sanity check our callback before calling it */
		  		    if(handler_sanity_check_with_panic((physaddr_t)usb_fs_callbacks.data_sent_callback)){
					return;
				    }
				    else{
					usb_fs_callbacks.data_sent_callback();
				    }
            }
		}

	}

	set_reg(r_CORTEX_M_USB_FS_GINTMSK, 1, USB_FS_GINTMSK_IEPINT);
}


/**
 * \brief Output interrupt handler.
 *
 * Handles an output interrupt. Fill setup packet if on EP0.
 * If data on EP0, handles by DFU functions (only mode where
 * data is transfered on EP0)
 * Else handles data on EPx.
 */
static void oepint_handler(void)
{
        uint32_t daint = read_reg_value(r_CORTEX_M_USB_FS_DAINT);

        if (daint & USB_FS_DAINT_OEPINT(USB_FS_DXEPCTL_EP0)) {
                uint32_t doepint0 = read_reg_value(r_CORTEX_M_USB_FS_DOEPINT(USB_FS_DXEPCTL_EP0));
                if (doepint0 & USB_FS_DOEPINT_STUP_Msk) {
                        /* We can process the received SETUP Packet (RM0090 p1357) */
                        set_reg_bits(r_CORTEX_M_USB_FS_DOEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DOEPINT_STUP_Msk);
                        struct usb_setup_packet stp_packet = {
                                setup_packet[0],
                                setup_packet[1],
                                setup_packet[3] << 8 | setup_packet[2],
                                setup_packet[5] << 8 | setup_packet[4],
                                setup_packet[7] << 8 | setup_packet[6]
                        };
                        /* Inform the upper layer that a setup packet is available */
                        usb_ctrl_handler(&stp_packet);
                }
                if (doepint0 & USB_FS_DOEPINT_XFRC_Msk) {
                        set_reg_bits(r_CORTEX_M_USB_FS_DOEPINT(USB_FS_DXEPCTL_EP0), USB_FS_DOEPINT_XFRC_Msk);
                }
        }
        if (daint & USB_FS_DAINT_OEPINT(USB_FS_DXEPCTL_EP1)) {
                uint32_t doepint1 = read_reg_value(r_CORTEX_M_USB_FS_DOEPINT(USB_FS_DXEPCTL_EP1));
                if (doepint1 & USB_FS_DOEPINT_XFRC_Msk) {
                        set_reg_bits(r_CORTEX_M_USB_FS_DOEPINT(USB_FS_DXEPCTL_EP1), USB_FS_DOEPINT_XFRC_Msk);
                        set_reg_bits(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP1), USB_FS_DOEPCTL_SNAK_Msk); // WHERE in the datasheet ? In disabling an OUT ep (p1360)
                        buffer_ep1 = NULL;
                        if (usb_fs_callbacks.data_received_callback) {
				    /* Sanity check our callback before calling it */
		  		    if(handler_sanity_check_with_panic((physaddr_t)usb_fs_callbacks.data_received_callback)){
					return;
				    }
				    else{
	                                usb_fs_callbacks.data_received_callback(buffer_ep1_idx);
				    }
                        }
                        set_reg_bits(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP0), USB_FS_DOEPCTL_CNAK_Msk);
                        buffer_ep1_idx = 0;
                }
        }

        set_reg(r_CORTEX_M_USB_FS_GINTMSK, 1, USB_FS_GINTMSK_OEPINT);
}

/**
 * \brief Handles an unknown interrupt.
 */
static void default_handler(uint8_t i)
{
	switch (i) {
	case USB_FS_GINTSTS_USBSUSP_Pos:
	case USB_FS_GINTSTS_ESUSP_Pos:
	case USB_FS_GINTSTS_SOF_Pos:
		break;
	default:
		log_printf("[USB FS] Unhandled int %d\n", i);
	}
}


#ifdef CONFIG_WOOKEY
static uint32_t usb_fs_isr_handlers_count[USB_MAX_ISR] = { 0 };

/**
 * \brief Manage IRQ handler calls.
 *
 * IRQ Handlers as defined in startup_stm32f4xx.s
 */

static volatile uint32_t count = 0;
static volatile uint32_t lastsr[32] = { 0 };
static volatile uint32_t lastdr[32] = { 0 };


void OTG_FS_IRQHandler(uint8_t irq __UNUSED, // IRQ number
                       uint32_t sr,  // content of posthook.status,
                       uint32_t dr)  // content of posthook.data)
{
	uint8_t i;
	uint32_t intsts = sr;
	uint32_t intmsk = dr;
    lastsr[count % 32] = sr;
    lastdr[count % 32] = dr;

    count++;
	if (intsts & USB_FS_GINTSTS_CMOD_Msk){
//		log_printf("[USB FS] Int in Host mode !\n");
//		not in ISR mode!
	}
    uint32_t val = intsts;
    val &= intmsk;

	for (i = 0; i < 32; i++) {
		if (!(intsts & ((uint32_t)1 << i)) || !(intmsk & ((uint32_t)1 << i)))
			continue;
        usb_fs_isr_handlers_count[i]++;
		if (usb_fs_isr_handlers[i]){
			usb_fs_isr_handlers[i]();
		}
		else{
			default_handler(i);
		}
	}
}
#endif

/**
 * \brief Init ISR handler vector.
 */
static void usb_fs_init_isr_handlers(void)
{
#define X(interrupt, handler) (usb_fs_isr_handlers[interrupt] = handler)
    X(USB_FS_GINTSTS_CMOD_Pos, NULL);                   /* 0  CMOD    : Current mode of operation */
    X(USB_FS_GINTSTS_MMIS_Pos, NULL);                   /* 1  MMIS    : Mode mismatch interrupt */
    X(USB_FS_GINTSTS_OTGINT_Pos, NULL);                 /* 2  OTGINT  : OTG interrupt */
    X(USB_FS_GINTSTS_SOF_Pos, NULL);                    /* 3  SOF     : Start of frame */
    X(USB_FS_GINTSTS_RXFLVL_Pos, rxflvl_handler);       /* 4  RXFLVL  : RxFIFO non-empty */
    X(USB_FS_GINTSTS_NPTXFE_Pos, NULL);                 /* 5  NPTXFE  : Non-periodic TxFIFO empty (HOST MODE ONLY) */
    X(USB_FS_GINTSTS_GINAKEFF_Pos, NULL);               /* 6  GINAKEFF: Global IN non-periodic NAK effective */
    X(USB_FS_GINTSTS_GOUTNAKEFF_Pos, NULL);             /* 7  GONAKEFF: Global OUT NAK effective */
    X(8, NULL);                                         /* 8  Reserved */
    X(9, NULL);                                         /* 7  Reserved */
    X(USB_FS_GINTSTS_ESUSP_Pos, NULL);                  /* 10 ESUSP   : Early suspend */
    X(USB_FS_GINTSTS_USBSUSP_Pos, NULL);                /* 11 USBSUSP : USB suspend */
    X(USB_FS_GINTSTS_USBRST_Pos, ep_init_reset);        /* 12 USBRST  : USB reset */
    X(USB_FS_GINTSTS_ENUMDNE_Pos, ep_init_enum);        /* 13 ENUMDNE : Speed Enumeration done */
    X(USB_FS_GINTSTS_ISOODRP_Pos, NULL);                /* 14 ISOODRP : Isochronous OUT packet dropped interrupt */
    X(USB_FS_GINTSTS_EOPF_Pos, NULL);                   /* 15 EOPF    : End of periodic frame interrupt */
    X(16, NULL);                                        /* 16 Reserved */
    X(USB_FS_GINTSTS_EPMISM_Pos, NULL);                 /* 17 EPMISM  : Endpoint mismatch interrupt */
    X(USB_FS_GINTSTS_IEPINT_Pos, iepint_handler);       /* 18 IEPINT  : IN endpoint interrupt */
    X(USB_FS_GINTSTS_OEPINT_Pos, oepint_handler);       /* 19 OEPINT  : OUT endpoint interrupt */
    X(USB_FS_GINTSTS_IISOIXFR_Pos, NULL);               /* 20 IISOIXFR: Incomplete isochronous IN transfer */
    X(USB_FS_GINTSTS_IPXFR_Pos, NULL);                  /* 21 IPXFR   : Incomplete periodic transfer */
    X(22, NULL);                                        /* 22 Reserved */
    X(23, NULL);                                        /* 23 Reserved */
    X(USB_FS_GINTSTS_HPRTINT_Pos, NULL);                /* 24 HPRTINT : Host port interrupt (HOST MODE ONLY) */
    X(USB_FS_GINTSTS_HCINT_Pos, NULL);                  /* 25 HCINT   : Host channels interrupt (HOST MODE ONLY) */
    X(USB_FS_GINTSTS_PTXFE_Pos, NULL);                  /* 26 PTXFE   : Periodic TxFIFO empty (HOST MODE ONLY) */
    X(27, NULL);                                        /* 27 Reserved */
    X(USB_FS_GINTSTS_CIDSCHG_Pos, NULL);                /* 28 CIDSCHG : Connector ID status change */
    X(USB_FS_GINTSTS_DISCINT_Pos, NULL);                /* 29 DISCINT : Disconnect detected interrupt (HOST MODE ONLY)*/
    X(USB_FS_GINTSTS_SRQINT_Pos, NULL);                 /* 30 SRQINT  : Session request/new session detected interrupt */
    X(USB_FS_GINTSTS_WKUPINT_Pos, NULL);                /* 31 WKUPINT : Resume/remote wakeup detected interrupt */
#undef X
}


/**
 * \brief function to write data on FIFO
 *
 * Can only send 128B !
 * " The non-periodic transmit FIFO can hold two packets
 * (128 bytes for FS) "
 * See section 34.17.4 p1340 (RM0090 docID018909 Rev 13)
 * (% 4 == & 3)
 *
 * The application must ensure that at least one free space is available in the periodic/non-periodic
 * request queue before starting to write to the transmit FIFO.
 * The application must always write to the transmit FIFO in DWORDs.
 *      If the packet size is non-DWORD aligned, the application must use padding.
 * The OTG_FS host determines the actual packet size based on the programmed maximum packet size and transfer size.
 *
 * @param src Source adress
 * @param size Size of data
 * @param ep Endpoint used
 */
static void _write_fifo(const void *src, uint32_t size, uint8_t ep)
{
	uint32_t size_4bytes = size / 4;
	uint32_t needed_words = size / 4 + (size & 3 ? 1 : 0);
    uint32_t tmp;
	uint32_t i;

	if((size != 0) && (src == NULL)){
		return;
	}

	if (needed_words > USB_FS_TX_FIFO_SZ){
		log_printf("needed_words > %d", USB_FS_TX_FIFO_SZ);
	}

        while (get_reg(r_CORTEX_M_USB_FS_DTXFSTS(ep), USB_FS_DTXFSTS_INEPTFSAV) < needed_words){
                /* Are we suspended? */
                if(get_reg(r_CORTEX_M_USB_FS_DSTS, USB_FS_DSTS_SUSPSTS)){
                        return;
                }
        }

        // FIXME: IP should has its own interrupts disable during ISR execution
        uint32_t oldmask = read_reg_value(r_CORTEX_M_USB_FS_GINTMSK);
        set_reg_value(r_CORTEX_M_USB_FS_GINTMSK, 0, 0xffffffff, 0);
	//disable_irq();

	for (i = 0; i < size_4bytes; i++, src += 4){
		write_reg_value(USB_FS_DEVICE_FIFO(ep), *(const uint32_t *)src);
	}
	switch (size & 3) {
	case 1:
		write_reg_value(USB_FS_DEVICE_FIFO(ep), *(const uint8_t *)src);
		break;
	case 2:
		write_reg_value(USB_FS_DEVICE_FIFO(ep), *(const uint16_t *)src);
		break;
	case 3:
		tmp  = ((const uint8_t*) src)[0];
		tmp |= ((const uint8_t*) src)[1] << 8;
		tmp |= ((const uint8_t*) src)[2] << 16;
		write_reg_value(USB_FS_DEVICE_FIFO(ep), tmp);
		break;
	default:
		break;
	}

        // FIXME: IP should has its own interrupts disable during ISR execution
	//enable_irq();
        set_reg_value(r_CORTEX_M_USB_FS_GINTMSK, oldmask, 0xffffffff, 0);
}


/* Split the data to send in 128 bytes max packets */
static void write_fifo(const void *src, uint32_t size, uint8_t ep)
{
	unsigned int i;
	unsigned int num = size / USB_FS_TX_FIFO_SZ;

	for(i = 0; i < num; i++){
		_write_fifo(src + (i * USB_FS_TX_FIFO_SZ), USB_FS_TX_FIFO_SZ, ep);
	}
	/* Handle the possible last packet */
	if(size > (num * USB_FS_TX_FIFO_SZ)){
		_write_fifo(src + (num * USB_FS_TX_FIFO_SZ), size - (num * USB_FS_TX_FIFO_SZ), ep);
	}

}

void usb_fs_driver_send_zlp(uint8_t ep){

    /* 1. Program the OTG_FS_DIEPTSIZx register for the transfer size and the corresponding packet count. */
    set_reg_value(r_CORTEX_M_USB_FS_DIEPTSIZ(ep), 1, USB_FS_DIEPTSIZ_PKTCNT_Msk(ep), USB_FS_DIEPTSIZ_PKTCNT_Pos(ep));
    set_reg_value(r_CORTEX_M_USB_FS_DIEPTSIZ(ep), 0, USB_FS_DIEPTSIZ_XFRSIZ_Msk(ep), USB_FS_DIEPTSIZ_XFRSIZ_Pos(ep));

    /* 2. Enable endpoint for transmission. */
    set_reg_bits(r_CORTEX_M_USB_FS_DIEPCTL(ep), USB_FS_DIEPCTL_CNAK_Msk | USB_FS_DIEPCTL_EPENA_Msk);

}

/**
 * \brief Function to handle send call.
 *
 * Set register correctly for the device to send data to host next.
 * This is NOT where the actual sending is done.
 *
 * @param src Source adress
 * @param size Size of data
 * @param ep Endpoint used
 */
void _usb_fs_driver_send(const void *src, uint32_t size, uint8_t ep)
{
    uint32_t packet_count = 0;

    /* FIXME Only sending data from IN endpoints is implemented */
    if (src == NULL){
        usb_fs_driver_send_zlp(ep);
        return;
    }
    /* Program the transfer size and packet count as follows:
     *      xfersize = N * maxpacket + short_packet pktcnt = N + (short_packet exist ? 1 : 0)
     */
    packet_count = size / MAX_DATA_PACKET_SIZE(ep) + (size & (MAX_DATA_PACKET_SIZE(ep)-1) ? 1 : 0);

    /* 1. Program the OTG_FS_DIEPTSIZx register for the transfer size and the corresponding packet count. */
    set_reg_value(r_CORTEX_M_USB_FS_DIEPTSIZ(ep), packet_count, USB_FS_DIEPTSIZ_PKTCNT_Msk(ep), USB_FS_DIEPTSIZ_PKTCNT_Pos(ep));
    set_reg_value(r_CORTEX_M_USB_FS_DIEPTSIZ(ep), size, USB_FS_DIEPTSIZ_XFRSIZ_Msk(ep), USB_FS_DIEPTSIZ_XFRSIZ_Pos(ep));

    /* 2. Enable endpoint for transmission. */
    set_reg_bits(r_CORTEX_M_USB_FS_DIEPCTL(ep), USB_FS_DIEPCTL_CNAK_Msk | USB_FS_DIEPCTL_EPENA_Msk);


   /* The application can write multiple packets for the same
    * endpoint into the transmit FIFO, if space is available.
    * /!\ FIXME For periodic IN endpoints, the application must
    *     write packets only for one microframe.
    *     It can write packets for the next periodic transaction
    *     only after getting transfer complete for the previous transaction.
    */
    write_fifo(src, size, ep);
    /* Are we suspended */
    if(get_reg(r_CORTEX_M_USB_FS_DSTS, USB_FS_DSTS_SUSPSTS)){
        return;
    }
    //FIXME activate TXEMPTY intrrupt in DIEPEMPMSK register
}

void usb_fs_driver_send(const void *src, uint32_t size, uint8_t ep)
{
        /* FIXME/TODO: it would be cleaner to monitor TXEMPTY here */
        if((ep == USB_FS_DXEPCTL_EP0) && (src != NULL)){
                /* Special handling for EP0 to split sending across multiple packets */
                unsigned int i;
                uint32_t packet_count = 0;
                packet_count = size / MAX_DATA_PACKET_SIZE(ep);
                ep0_last_packet_sent = 0;
                for(i = 0; i < packet_count; i++){
                        if((size == (packet_count * MAX_DATA_PACKET_SIZE(ep))) && (i == packet_count-1)){
                                ep0_last_packet_sent = 1;
                        }
                        ep0_packets_sent = 0;
                        _usb_fs_driver_send(src+(i*MAX_DATA_PACKET_SIZE(ep)), MAX_DATA_PACKET_SIZE(ep), ep);
                        if(((size == (packet_count * MAX_DATA_PACKET_SIZE(ep))) && (i != packet_count-1)) ||
                            (size != (packet_count * MAX_DATA_PACKET_SIZE(ep)))){
                                /* For all the intermediate packets except the last one:
                                 * wait with timeout (the host might be gone ...)
                                 * FIXME: find a better way to handle timeout
                                 */
                                while(ep0_packets_sent == 0){
                                        /* Are we suspended? */
                                        if(get_reg(r_CORTEX_M_USB_FS_DSTS, USB_FS_DSTS_SUSPSTS)){
                                                break;
                                        }
                                }
                        }
                }
                if(size != (packet_count * MAX_DATA_PACKET_SIZE(ep))){
                        /* Residual data to send */
                        ep0_last_packet_sent = 1;
                        _usb_fs_driver_send(src+(packet_count*MAX_DATA_PACKET_SIZE(ep)), size - (packet_count*MAX_DATA_PACKET_SIZE(ep)), ep);
                }
                if(packet_count > 0){
                        set_reg_bits(r_CORTEX_M_USB_FS_DOEPCTL(USB_FS_DXEPCTL_EP0), USB_FS_DOEPCTL_CNAK_Msk);
                }
        }
        else if((ep == USB_FS_DXEPCTL_EP0) && (src == NULL)){
                ep0_last_packet_sent = 1;
                _usb_fs_driver_send(src, size, ep);
        }
        else{
                _usb_fs_driver_send(src, size, ep);
        }
        /* If we are suspended, call ourself the callback and flush stuff */
        if(get_reg(r_CORTEX_M_USB_FS_DSTS, USB_FS_DSTS_SUSPSTS)){
                usb_fs_driver_TXFIFO_flush_all();
                if (usb_fs_callbacks.data_sent_callback){
 		    /* Sanity check our callback before calling it */
  		    if(handler_sanity_check_with_panic((physaddr_t)usb_fs_callbacks.data_sent_callback)){
			return;
		    }
		    else{
                        usb_fs_callbacks.data_sent_callback();
		    }
                }
        }

}

/**
 * \brief Function to handle read call.
 *
 * Set register correctly for the device to read data from host next.
 * This is NOT where the actual reading is done.
 *
 * @param src Source adress
 * @param size Size of data
 * @param ep Endpoint used
 */
void usb_fs_driver_read(void *dst, uint32_t size, uint8_t ep)
{
        if (ep == USB_FS_DXEPCTL_EP0 && dst != NULL) {
                /* For EP0, we have to monitor each packet since we are limited to 64 bytes for data */
                assert(dst);
                assert(!buffer_ep0);
                buffer_ep0 = dst;
                buffer_ep0_idx = 0;
                buffer_ep0_size = size;
                /* No need to trigger a oepint interrupt here, since we are limited to 64 bytes packets anyways ...
                 * Handling the completion in rxflvl is a better choice.
                 */
        }
        if (ep == USB_FS_DXEPCTL_EP0 && dst == NULL) {
                buffer_ep0 = NULL;
                buffer_ep0_idx = buffer_ep0_size = 0;
                /* No need to trigger a oepint interrupt here, since we are limited to 64 bytes packets anyways ...
                 * Handling the completion in rxflvl is a better choice.
                 */
        }
        if (ep == USB_FS_DXEPCTL_EP1) {
                if(dst != NULL){
                        assert(dst);
                        assert(!buffer_ep1);
                        buffer_ep1 = dst;
                        buffer_ep1_idx = 0;
                        buffer_ep1_size = size;
                }
                else{
                        buffer_ep1 = NULL;
                        buffer_ep1_idx = buffer_ep1_size = 0;
                }
                /* Set the packet count number in the PKTCNT field */
                uint32_t packet_count = size / MAX_DATA_PACKET_SIZE(ep) + (size & (MAX_DATA_PACKET_SIZE(ep)-1) ? 1 : 0);
                set_reg_value(r_CORTEX_M_USB_FS_DOEPTSIZ(ep), packet_count, USB_FS_DOEPTSIZ_PKTCNT_Msk(ep), USB_FS_DOEPTSIZ_PKTCNT_Pos(ep));
                /* Set the size in the XFRSIZ field */
                set_reg_value(r_CORTEX_M_USB_FS_DOEPTSIZ(ep), size, USB_FS_DOEPTSIZ_XFRSIZ_Msk(ep), USB_FS_DOEPTSIZ_XFRSIZ_Pos(ep));

        }
        set_reg_bits(r_CORTEX_M_USB_FS_DOEPCTL(ep), USB_FS_DOEPCTL_CNAK_Msk);
}

/**
 * \brief Set adress.
 */
void usb_fs_driver_set_address(uint16_t addr)
{
	set_reg(r_CORTEX_M_USB_FS_DCFG, addr, USB_FS_DCFG_DAD);
}
#endif
