#include <device.h>
#include <init.h>
#include <greybus/platform.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_service);

#ifdef CONFIG_GREYBUS_STATIC_MANIFEST
#include <greybus/static-manifest.h>
#endif

/* Currently only one greybus instance is supported */
#define GREYBUS_BUS_NAME "GREYBUS_0"
/* Based on UniPro, from Linux */
#define CPORT_ID_MAX 4095

/* Deferred init of some DT nodes required - see defer_init.c */
extern int gb_service_deferred_init(void);

static size_t num_cports;

static int greybus_service_init(struct device *bus)
{
    int r;
	struct greybus_platform_api *api;
	uint8_t *mnfb;
	size_t mnfb_size;

    LOG_INF("Greybus initializing..");

	r = gb_service_deferred_init();
	if (r < 0) {
		LOG_ERR("gb_service_deferred_init() failed: %d", r);
		goto out;
	}

	bus = device_get_binding(GREYBUS_BUS_NAME);
	if (NULL == bus) {
		LOG_ERR("failed to get " GREYBUS_BUS_NAME " device");
		goto out;
	}

	api = (struct bus_api *) bus->driver_api;
	if (NULL == api) {
		LOG_ERR("failed to get " GREYBUS_BUS_NAME " driver_api");
		goto out;
	}

	r = api->num_cports(bus);
	if (r < 0) {
		LOG_ERR("failed to get the number of cports");
		goto out;
	}
    if (r == 0) {
		LOG_ERR("no cports are defined");
        r = -EINVAL;
		goto out;
    }
    if (r > CPORT_ID_MAX) {
        LOG_ERR("invalid number of cports %u", (unsigned)r);
        r = -EINVAL;
        goto out;
    }
	num_cports = r;

    /* assume ownership of the dynamically allocated mnfb */
    r = api->gen_mnfb(bus, &mnfb, &mnfb_size);
    if (r < 0) {
        LOG_ERR("failed to generate mnfb: %d", r);
        goto out;
    }

    /* ok to release resources from the manifest IR */
    r = api->fini(bus);

	r = manifest_parse(mnfb, mnfb_size);
	if (r != true) {
		LOG_ERR("failed to parse mnfb");
		r = -EINVAL;
        goto free_mnfb;
	}

    /* for some reason, set_manifest_blob() */
    set_manifest_blob(mnfb);

    r = gb_init((struct gb_transport_backend *)NULL);
    if (r < 0) {
        LOG_ERR("gb_init() failed: %d", r);
        goto clear_mnfb;
    }

    enable_cports();

    LOG_INF("Greybus is active");

    r = 0;
    goto out;

clear_mnfb:
    set_manifest_blob(NULL);

free_mnfb:
    free(mnfb);
    mnfb = NULL;
    mnfb_size = 0;

out:
    return r;
}

SYS_INIT(greybus_service_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
