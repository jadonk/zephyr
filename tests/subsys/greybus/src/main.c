/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <device.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test);

#include <greybus/manifecto/manifest.h>

#include "bus.h"

/*
 * FIXME: currently, Zephyr does not guarantee that parent DeviceTree nodes are
 * initialized before child DeviceTree nodes. This has the unfortunate side
 * effect of pushing-off device initialization to the application.
 * It's possible that Zephyr may initialize things in the correct order if
 * phandles are used instead, but that isn't clear at this point.
 *
 * E: greybus bundle: failed to get driver_api for device 'GREYBUS_0'
 * E: greybus bundle: failed to get driver_api for device 'GREYBUS_0'
 * I: probed greybus: 0 major: 0 minor: 1
 * I: probed cport 0: class: 0 protocol: 0
 * I: probed cport 0: class: 10 protocol: 2
 * E: failed to get driver_api for device 'GREYBUS_0'
 * E: greybus string: driver_api was NULL
 * E: greybus string: driver_api was NULL
 * E: Cannot create zeth (-1)
 * *** Booting Zephyr OS build zephyr-v2.3.0-733-g5217cd775382  ***
 * I: Initializing network
 * I: IPv4 address: 192.0.2.1
 * I: test_main(): ahoy!
 * I: IPv6 address: fe80::5eff:fe00:533b
 * I: IPv6 address: fe80::5eff:fe00:533b
 * ^C
 *
 * pabigot is working on a PR to address this, which reorders custom
 * elf-sections at link-time in order to create the proper order.
 *
 * See https://github.com/zephyrproject-rtos/zephyr/pull/26616 for details.
 */

struct definit {
	struct device *dev;
	int (*init)(struct device *);
};
static size_t definits_size;
static struct definit *definits;

int defer_init(struct device *dev, int (*init)(struct device *)) {
	size_t n = definits_size;
	struct definit *tmp;

	tmp = realloc(definits, (n + 1) * sizeof(*tmp));
	if (NULL == tmp) {
		LOG_ERR("failed to allocate space for deferred init");
		return -ENOMEM;
	}

	definits = tmp;
	definits[n].dev = dev;
	definits[n].init = init;

	definits_size++;

	return 0;
}

static int deferred_init(void) {
	int r = 0;
	int rr = 0;
	struct device *dev;
	int (*init)(struct device *);

	for(size_t i = 0; i < definits_size; ++i) {

		dev = definits[i].dev;
		init = definits[i].init;

		r = init(dev);
		if (r < 0) {
			if (0 == rr) {
				rr = r;
			}
			LOG_ERR("failed to initialize '%s'", dev->name);
		}
	}

	if (NULL != definits) {
		free(definits);
		definits = NULL;
		definits_size = 0;
	}

	return rr;
}

void test_main(void) {

	int r;
	struct device *greybus;
	struct bus_api *api;
	uint8_t *mnfb;
	uint8_t mnfb_size;
	manifest_t manifest;

	r = deferred_init();
	if (r < 0) {
		LOG_ERR("deferred_init() failed: %d", r);
		goto out;
	}

	greybus = device_get_binding("GREYBUS_0");
	if (NULL == greybus) {
		LOG_ERR("failed to get GREYBUS_0 device");
		goto out;
	}

	api = (struct bus_api *) greybus->driver_api;
	if (NULL == api) {
		LOG_ERR("failed to get GREYBUS_0 driver_api");
		goto out;
	}

	manifest = api->get_manifest(greybus);
	if (NULL == manifest) {
		LOG_ERR("failed to get greybus manifest");
		goto out;
	}

	r = manifest_mnfb_gen(manifest);
	if (r < 0) {
		LOG_ERR("manifest_mnfb_gen() failed: %d", r);
		goto out;
	}

	mnfb = manifest_mnfb_data(manifest);
	if (NULL == mnfb) {
		LOG_ERR("mnfb was NULL");
		goto out;
	}

	mnfb_size = manifest_mnfb_size(manifest);

	LOG_INF("Greybus Manifest Binary (size: %u):", mnfb_size);
	for(size_t i = 0; i < mnfb_size; ++i) {
		if (0 == i % 8 && i != 0) {
			printk("\n");
		}
		printk("%02x ", mnfb[i]);
	}
	printk("\n");

out:
    for(;;) {
    	k_sleep(K_MSEC(500));
    }
}
