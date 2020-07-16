#ifndef ASDFGHJ_
#define ASDFGHJ_

#include <stdint.h>
#include <device.h>
#include <greybus/manifecto/manifest.h>

struct bus_api {
	int (*add_interface)(struct device *bus, uint16_t vendor_string_id,
            uint16_t product_string_id);
	int (*add_string)(struct device *bus, uint8_t id, const char *string_);
	int (*add_bundle)(struct device *bus, uint8_t id, BundleClass class_);
	int (*add_cport)(struct device *bus, uint8_t id, BundleClass class_, CPortProtocol protocol);
	manifest_t (*get_manifest)(struct device *bus);
};

#endif /* ASDFGHJ_ */
