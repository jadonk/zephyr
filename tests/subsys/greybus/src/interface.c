#include <errno.h>
#include <stdint.h>
#include <zephyr.h>

#define DT_DRV_COMPAT zephyr_greybus_interface
#include <device.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_interface);

struct greybus_interface_config {
    const uint8_t num;
};

static int greybus_interface_init(struct device *dev) {

	ARG_UNUSED(dev);

    return 0;
}

#define DEFINE_GREYBUS_INTERFACE(_num)						\
															\
        static struct greybus_interface_config				\
			greybus_interface_config_##_num = {				\
                .num = (uint8_t)_num,						\
        };													\
        													\
        DEVICE_INIT(greybus_interface_##_num,				\
			"GBINTERFACE_" #_num,							\
			greybus_interface_init,							\
			NULL,											\
			&greybus_interface_config_##_num, POST_KERNEL,	\
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_INTERFACE);
