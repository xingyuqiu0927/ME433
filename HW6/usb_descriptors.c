#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

#define USB_VID 0xCafe
#define USB_PID 0x4004
#define USB_BCD 0x0200

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE)),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void) instance;
    return desc_hid_report;
}

enum {
    ITF_NUM_HID,
    ITF_NUM_TOTAL,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define EPNUM_HID 0x81

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(
        ITF_NUM_HID,
        0,
        HID_ITF_PROTOCOL_MOUSE,
        sizeof(desc_hid_report),
        EPNUM_HID,
        CFG_TUD_HID_EP_BUFSIZE,
        5
    ),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

static char const *string_desc_arr[] = {
    (const char[]) {0x09, 0x04},
    "ME433",
    "Pico 2 IMU Mouse",
    NULL,
};

static uint16_t desc_string[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;

    size_t char_count = 0;

    if (index == STRID_LANGID) {
        memcpy(&desc_string[1], string_desc_arr[0], 2);
        char_count = 1;
    } else if (index == STRID_SERIAL) {
        char_count = board_usb_get_serial(desc_string + 1, 32);
    } else {
        if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        char const *str = string_desc_arr[index];
        if (!str) {
            return NULL;
        }

        char_count = strlen(str);
        if (char_count > 32) {
            char_count = 32;
        }

        for (size_t i = 0; i < char_count; i++) {
            desc_string[1 + i] = str[i];
        }
    }

    desc_string[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * char_count + 2));
    return desc_string;
}
