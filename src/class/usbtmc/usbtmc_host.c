/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 N Conrad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_config.h"

#if (CFG_TUH_ENABLED && CFG_TUH_USBTMC)

#include "host/usbh.h"
#include "host/usbh_pvt.h"

#include "usbtmc_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// Buffer size must be an exact multiple of the max packet size for both
// bulk  (up to 64 bytes for FS, 512 bytes for HS). In addation, this driver
// imposes a minimum buffer size of 32 bytes.
#define USBTMCD_BUFFER_SIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

// Interrupt endpoint buffer size, default to 2 bytes as USB488 specification.
#ifndef CFG_TUD_USBTMC_INT_EP_SIZE
#define CFG_TUD_USBTMC_INT_EP_SIZE 2
#endif

typedef enum
{
    STATE_CLOSED,  // Endpoints have not yet been opened since USB reset
    STATE_NAK,     // Bulk-out endpoint is in NAK state.
    STATE_IDLE,    // Bulk-out endpoint is waiting for CMD.
    STATE_RCV,     // Bulk-out is receiving DEV_DEP message
    STATE_TX_REQUESTED,
    STATE_TX_INITIATED,
    STATE_TX_SHORTED,
    STATE_CLEARING,
    STATE_ABORTING_BULK_IN,
    STATE_ABORTING_BULK_IN_SHORTED, // aborting, and short packet has been queued for transmission
    STATE_ABORTING_BULK_IN_ABORTED, // aborting, and short packet has been transmitted
    STATE_ABORTING_BULK_OUT,
    STATE_NUM_STATES
} usbtmcd_state_enum;

#if (CFG_TUD_USBTMC_ENABLE_488)
typedef usbtmc_response_capabilities_488_t usbtmc_capabilities_specific_t;
#else
typedef usbtmc_response_capabilities_t usbtmc_capabilities_specific_t;
#endif

typedef struct{
    volatile usbtmcd_state_enum state;

    uint8_t itf_num; // Interface ID

    uint8_t ep_bulk_in; // Bulk IN endpoint
    uint8_t ep_bulk_out; // Bulk OUT endpoint
    uint8_t ep_int_in; // Interrupt IN endpoint
    // uint32_t ep_bulk_in_wMaxPacketSize; // Bulk IN endpoint max packet size //TODO: Delete if not needed
    // uint32_t ep_bulk_out_wMaxPacketSize; // Bulk OUT endpoint max packet size //TODO: Delete if not neede
    // IN buffer is only used for first packet, not the remainder
    // in order to deal with prepending header
    CFG_TUSB_MEM_ALIGN uint8_t ep_bulk_in_buf[USBTMCD_BUFFER_SIZE]; // Bulk IN buffer
    // OUT buffer receives one packet at a time
    CFG_TUSB_MEM_ALIGN uint8_t ep_bulk_out_buf[USBTMCD_BUFFER_SIZE]; // Bulk OUT buffer
    //  int msg to ensure alignment and placement correctness
    CFG_TUSB_MEM_ALIGN uint8_t ep_int_in_buf[CFG_TUD_USBTMC_INT_EP_SIZE]; // Interrupt IN buffer

    uint32_t transfer_size_remaining; // also used for requested length for bulk IN.
    uint32_t transfer_size_sent;      // To keep track of data bytes that have been queued in FIFO (not header bytes)

    uint8_t lastBulkOutTag; // used for aborts (mostly)
    uint8_t lastBulkInTag; // used for aborts (mostly)

    uint8_t const *devInBuffer; // pointer to application-layer used for transmissions

    bool configured;
    bool mounted;

    usbtmc_capabilities_specific_t *capabilities;
} usbtmc_interface_t;

CFG_TUH_MEM_SECTION static usbtmc_interface_t _usbtmc_itf[CFG_TUH_DEVICE_MAX];

CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN
static uint8_t _usbtmc_buffer[sizeof(usbtmc_response_capabilities_t)];

TU_ATTR_ALWAYS_INLINE
static inline usbtmc_interface_t* get_itf(uint8_t dev_addr) {
    return &_usbtmc_itf[dev_addr - 1];
}

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+
bool usbtmch_init(void) {
    tu_memclr(_usbtmc_itf, sizeof(_usbtmc_itf));
    return true;
}

bool usbtmch_deinit(void) {
    return true;
}

bool usbtmch_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes) {
    (void) dev_addr;
    (void) ep_addr;
    (void) event;
    (void) xferred_bytes;
    //usbtmc_interface_t * p_usbtmc = get_itf(dev_addr);

    // TODO: Implement this

    return true;
}

void usbtmch_close(uint8_t dev_addr) {
    TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX,);
    usbtmc_interface_t *p_usbtmc = get_itf(dev_addr);
    TU_VERIFY(p_usbtmc->configured,);

    if (p_usbtmc->mounted) {
        if (tuh_usbtmc_umount_cb) tuh_usbtmc_umount_cb(dev_addr);
    }

    tu_memclr(p_usbtmc, sizeof(usbtmc_interface_t));
}

//--------------------------------------------------------------------+
// Enumeration
//--------------------------------------------------------------------+

static void config_get_capabilities_complete(tuh_xfer_t* xfer);

bool usbtmch_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const* desc_itf, uint16_t max_len) {
    (void) rhport;

    /*char str_buffer[0xFF];
    int count = sprintf(str_buffer, "bInterfaceClass: %X, bInterfaceSubClass: %X", desc_itf->bInterfaceClass, desc_itf->bInterfaceSubClass);
    tuh_usbtmc_debug_cb(str_buffer, count);*/

    TU_VERIFY(TUSB_CLASS_APPLICATION_SPECIFIC == desc_itf->bInterfaceClass &&
                USBTMC_SUBCLASS_USB488 == desc_itf->bInterfaceSubClass);
    //TU_LOG_DRV("[%u] HID opening Interface %u\r\n", daddr, desc_itf->bInterfaceNumber);

    //TODO: Add USB488 protocol support
    /*if (desc_itf->bInterfaceProtocol == 1) {
       // Enable USB488 Protocol
    }*/

    // len = interface + n*endpoints
    uint16_t const drv_len = (uint16_t) (sizeof(tusb_desc_interface_t) +
                                         desc_itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    TU_ASSERT(max_len >= drv_len);
    usbtmc_interface_t *p_usbtmc = get_itf(dev_addr);


    //------------- Endpoint Descriptors -------------//
    tusb_desc_endpoint_t *ep_desc = (tusb_desc_endpoint_t *) tu_desc_next(desc_itf);

    for (int i = 0; i < desc_itf->bNumEndpoints; i++) {
        TU_ASSERT(TUSB_DESC_ENDPOINT == ep_desc->bDescriptorType);
        TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));


        if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN) {
            if (ep_desc->bmAttributes.xfer == TUSB_XFER_BULK)   p_usbtmc->ep_int_in = ep_desc->bEndpointAddress;
            else                                                p_usbtmc->ep_bulk_in = ep_desc->bEndpointAddress;
        } else {
            p_usbtmc->ep_bulk_out = ep_desc->bEndpointAddress;
        }

        ep_desc = (tusb_desc_endpoint_t *) tu_desc_next(desc_itf);
    }

    p_usbtmc->itf_num = desc_itf->bInterfaceNumber;

    return true;
}

bool usbtmch_set_config(uint8_t dev_addr, uint8_t itf_num) {
    usbtmc_interface_t *p_usbtmc = get_itf(dev_addr);
    TU_ASSERT(p_usbtmc->itf_num == itf_num);

    p_usbtmc->configured = true;

    //--------- GET CAPABILITIES ---------//
    tusb_control_request_t const request = {
            .bmRequestType_bit = {
                    .recipient = TUSB_REQ_RCPT_INTERFACE,
                    .type      = TUSB_REQ_TYPE_CLASS,
                    .direction = TUSB_DIR_IN
            },
            .bRequest = USBTMC_bREQUEST_GET_CAPABILITIES,
            .wValue   = 0,
            .wIndex   = itf_num,
            .wLength  = 0x0018
    };

    tuh_xfer_t xfer = {
            .daddr       = dev_addr,
            .ep_addr     = 0,
            .setup       = &request,
            .buffer      = _usbtmc_buffer,
            .complete_cb = config_get_capabilities_complete,
            .user_data   = 0
    };
    TU_ASSERT(tuh_control_xfer(&xfer));

    return true;
}

static void config_get_capabilities_complete(tuh_xfer_t *xfer) {
    uint8_t  const dev_addr = xfer->daddr;
    usbtmc_interface_t *p_usbtmc = get_itf(dev_addr);

    p_usbtmc->capabilities->bmIntfcCapabilities.listenOnly             = (_usbtmc_buffer[4] >> 0) & 0x01;
    p_usbtmc->capabilities->bmIntfcCapabilities.talkOnly               = (_usbtmc_buffer[4] >> 1) & 0x01;
    p_usbtmc->capabilities->bmIntfcCapabilities.supportsIndicatorPulse = (_usbtmc_buffer[4] >> 2) & 0x01;

    p_usbtmc->capabilities->bmDevCapabilities.canEndBulkInOnTermChar   = (_usbtmc_buffer[5] >> 0) & 0x01;

    p_usbtmc->mounted = true;
    if (tuh_usbtmc_mount_cb) tuh_usbtmc_mount_cb(dev_addr);

    usbh_driver_set_config_complete(dev_addr, p_usbtmc->itf_num);
}

#endif