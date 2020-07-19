#include <errno.h>
#include <stdint.h>
#include <zephyr.h>

#define DT_DRV_COMPAT zephyr_greybus_interface
#include <device.h>
#include <devicetree.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_interface);

#include <greybus/platform.h>

struct greybus_interface_config {
	const uint8_t num;
	const uint16_t vendor_string_id;
	const uint16_t product_string_id;
	const char *const bus_name;
};

static int greybus_interface_init(struct device *dev) {

	const struct greybus_interface_config *config =
		(struct greybus_interface_config *)dev->config_info;

	struct device *bus;
	struct greybus_platform_api *api;
	int r;

	bus = device_get_binding(config->bus_name);
	if (NULL == bus) {
		LOG_ERR("failed to get binding for device '%s'", config->bus_name);
		return -ENODEV;
	}

	api = (struct greybus_platform_api *)bus->driver_api;
	if (NULL == api) {
		LOG_ERR("failed to get driver_api for device '%s'", config->bus_name);
		return -EINVAL;
	}

	r = api->add_interface(bus, config->vendor_string_id, config->product_string_id);
	if (r < 0) {
		LOG_ERR("add_interface() failed: %d", r);
		return r;
	}

	LOG_INF("probed greybus interface %u", config->num);

    return 0;
}

extern int gb_service_defer_init(struct device *, int (*init)(struct device *));
static int defer_greybus_interface_init(struct device *dev) {
	return gb_service_defer_init(dev, &greybus_interface_init);
}


#define DEFINE_GREYBUS_INTERFACE(_num)						\
															\
        static const struct greybus_interface_config		\
			greybus_interface_config_##_num = {				\
			.num = (uint8_t) _num,							\
			.vendor_string_id =								\
				DT_PROP(DT_PHANDLE(DT_DRV_INST(_num), 		\
					vendor_string_id), id),					\
			.product_string_id =							\
				DT_PROP(DT_PHANDLE(DT_DRV_INST(_num), 		\
					product_string_id), id),				\
			.bus_name = 									\
				DT_LABEL(DT_PARENT(DT_DRV_INST(_num))),		\
        };													\
        													\
        DEVICE_INIT(greybus_interface_##_num,				\
			"GBINTERFACE_" #_num,							\
			defer_greybus_interface_init,					\
			NULL,											\
			&greybus_interface_config_##_num, POST_KERNEL,	\
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_INTERFACE);
