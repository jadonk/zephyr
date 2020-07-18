#ifndef ASDFGHJ_
#define ASDFGHJ_

#include <stdint.h>
#include <device.h>
#include <greybus/manifecto/manifest.h>

/*
 * Should probably be in greybus/platform.h
 *
 * Used for drivers to add their descriptors to the Greybus Manifest
 * Intermediate Represetntation (IR), extract sufficient information
 * to start the Greybus 
 */

struct bus_api {
	int (*add_interface)(struct device *bus, uint16_t vendor_string_id,
            uint16_t product_string_id);
	int (*add_string)(struct device *bus, uint8_t id, const char *string_);
	int (*add_bundle)(struct device *bus, uint8_t id, BundleClass class_);
	int (*add_cport)(struct device *bus, uint8_t id, BundleClass class_, CPortProtocol protocol);
	int (*num_cports)(struct device *bus);
	int (*gen_mnfb)(struct device *bus, uint8_t **mnfb, size_t *mnfb_size);
	void (*fini)(struct device *bus);
};

#endif /* ASDFGHJ_ */
