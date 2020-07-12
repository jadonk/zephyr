/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MANIFECTO_MANIFEST_H_
#define MANIFECTO_MANIFEST_H_

#include <stddef.h>
#include <stdint.h>
#ifdef _MSC_VER
#include <manifecto/_cdefs.h>
#else
#include <sys/cdefs.h>
#endif

__BEGIN_DECLS

typedef void *manifest_t;

// TODO: combine enums with C++ API
typedef enum {
  BUNDLE_CLASS_CONTROL = 0x00,
  BUNDLE_CLASS_AP = 0x01,
  BUNDLE_CLASS_HID = 0x05,
  BUNDLE_CLASS_POWER_SUPPLY = 0x08,
  BUNDLE_CLASS_BRIDGED_PHY = 0x0a,
  BUNDLE_CLASS_DISPLAY = 0x0c,
  BUNDLE_CLASS_CAMERA = 0x0d,
  BUNDLE_CLASS_SENSOR = 0x0e,
  BUNDLE_CLASS_LIGHTS = 0x0f,
  BUNDLE_CLASS_VIBRATOR = 0x10,
  BUNDLE_CLASS_LOOPBACK = 0x11,
  BUNDLE_CLASS_AUDIO = 0x12,
  BUNDLE_CLASS_SVC = 0x14,
  BUNDLE_CLASS_FIRMWARE = 0x15,
  BUNDLE_CLASS_RAW = 0xfe,
  BUNDLE_CLASS_VENDOR_SPECIFIC = 0xff,
} BundleClass;

typedef enum {
  CPORT_PROTOCOL_CONTROL = 0x00,
  CPORT_PROTOCOL_AP = 0x01,
  CPORT_PROTOCOL_GPIO = 0x02,
  CPORT_PROTOCOL_I2C = 0x03,
  CPORT_PROTOCOL_UART = 0x04,
  CPORT_PROTOCOL_HID = 0x05,
  CPORT_PROTOCOL_USB = 0x06,
  CPORT_PROTOCOL_SDIO = 0x07,
  CPORT_PROTOCOL_POWER_SUPPLY = 0x08,
  CPORT_PROTOCOL_PWM = 0x09,
  CPORT_PROTOCOL_SPI = 0x0b,
  CPORT_PROTOCOL_DISPLAY = 0x0c,
  CPORT_PROTOCOL_CAMERA_MANAGEMENT = 0x0d,
  CPORT_PROTOCOL_SENSOR = 0x0e,
  CPORT_PROTOCOL_LIGHTS = 0x0f,
  CPORT_PROTOCOL_VIBRATOR = 0x10,
  CPORT_PROTOCOL_LOOPBACK = 0x11,
  CPORT_PROTOCOL_AUDIO_MANAGEMENT = 0x12,
  CPORT_PROTOCOL_AUDIO_DATA = 0x13,
  CPORT_PROTOCOL_SVC = 0x14,
  CPORT_PROTOCOL_FIRMWARE = 0x15,
  CPORT_PROTOCOL_CAMERA_DATA = 0x16,
  CPORT_PROTOCOL_RAW = 0xfe,
  CPORT_PROTOCOL_VENDOR_SPECIFIC = 0xff,
} CPortProtocol;

typedef enum {
  DESC_TYPE_INTERFACE = 0x01,
  DESC_TYPE_STRING = 0x02,
  DESC_TYPE_BUNDLE = 0x03,
  DESC_TYPE_CPORT = 0x04,
} DescriptorType;

manifest_t manifest_new(void);
void manifest_fini(manifest_t *manifest);

int manifest_mnfs_parse(manifest_t manifest, const char *mnfs,
                        size_t mnfs_size);
int manifest_mnfs_parse_file(manifest_t manifest, const char *file);

int manifest_mnfb_gen(manifest_t manifest);
size_t manifest_mnfb_size(manifest_t manifest);
uint8_t *manifest_mnfb_data(manifest_t manifest);

int manifest_add_section(manifest_t manifest, const char *section_name);
int manifest_add_option(manifest_t manifest, const char *section_name,
                        const char *option_name, const char *option_value);

int manifest_add_header(manifest_t manifest, uint8_t major_, uint8_t minor_);
int manifest_add_interface_desc(manifest_t manifest, uint16_t vendor_id,
                                uint16_t product_id);
int manifest_add_string_desc(manifest_t manifest, uint8_t id,
                             const char *string_);
int manifest_add_bundle_desc(manifest_t manifest, uint8_t id,
                             BundleClass class_);
int manifest_add_cport_desc(manifest_t manifest, uint8_t id, BundleClass class_,
                            CPortProtocol protocol);

__END_DECLS

#endif /* MANIFECTO_MANIFEST_H_ */
