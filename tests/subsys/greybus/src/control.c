#include <stdint.h>
#include <zephyr.h>
#include <dt-bindings/greybus/greybus.h>

#define DT_DRV_COMPAT zephyr_greybus_control
#include <device.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_control);

struct greybus_control_config {
    const uint8_t id;
};

static int greybus_control_init(struct device *dev) {

	const struct greybus_control_config *config =
			(const struct greybus_control_config *)dev->config_info;

    LOG_INF("probed cport %u: class: %u protocol: %u", config->id,
		BUNDLE_CLASS_CONTROL, CPORT_PROTOCOL_CONTROL);

    return 0;
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
        };																	\
        																	\
        DEVICE_INIT(gpio_control_##_num, "GBCONTROL_" #_num,				\
                            greybus_control_init,							\
							NULL,											\
                            &greybus_control_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_CONTROL);
