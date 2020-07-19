#include <stdint.h>
#include <zephyr.h>
#include <dt-bindings/greybus/greybus.h>

#define DT_DRV_COMPAT zephyr_greybus_control
#include <device.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_control);

#include <greybus/platform.h>

struct greybus_control_config {
    const uint8_t id;
    const char *bus_name;
};

static int greybus_control_init(struct device *dev) {

    struct greybus_control_config *config =
        (struct greybus_control_config *)dev->config_info;
    int r;
    struct device *bus;
    struct greybus_platform_api *api;

    bus = device_get_binding(config->bus_name);
    if (NULL == bus) {
		LOG_ERR("control: failed to get binding for device '%s'", config->bus_name);
    	return -ENODEV;
    }

    api = (struct greybus_platform_api *) bus->driver_api;
    if (NULL == api) {
		LOG_ERR("control: failed to get driver_api for '%s'", config->bus_name);
    	return -EINVAL;
    }

    r = api->add_cport(bus, config->id, BUNDLE_CLASS_CONTROL, CPORT_PROTOCOL_CONTROL);
    if (r < 0) {
		LOG_ERR("control: failed to get driver_api for '%s'", config->bus_name);
		return r;
    }

    LOG_INF("probed cport %u: class: %u protocol: %u", config->id,
		BUNDLE_CLASS_CONTROL, CPORT_PROTOCOL_CONTROL);

    return 0;
}

extern int gb_service_defer_init(struct device *, int (*init)(struct device *));
static int defer_greybus_control_init(struct device *dev) {
	return gb_service_defer_init(dev, &greybus_control_init);
}

#define DEFINE_GREYBUS_CONTROL(_num)										\
																			\
		BUILD_ASSERT(DT_PROP(DT_PARENT(DT_DRV_INST(_num)), bundle_class)	\
			== BUNDLE_CLASS_CONTROL, "BUNDLE_CLASS_CONTROL required"); 		\
																			\
		BUILD_ASSERT(DT_PROP(DT_DRV_INST(_num), cport_protocol) 			\
			== CPORT_PROTOCOL_CONTROL, "CPORT_PROTOCOL_CONTROL required"); 	\
																			\
		static const struct greybus_control_config							\
			greybus_control_config_##_num = {								\
                .id = (uint8_t) _num,										\
				.bus_name = 												\
					DT_LABEL(DT_PARENT(DT_PARENT(DT_DRV_INST(_num)))),		\
        };																	\
        																	\
        DEVICE_INIT(gpio_control_##_num, "GBCONTROL_" #_num,				\
                            defer_greybus_control_init,						\
							NULL,											\
                            &greybus_control_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_CONTROL);
