#define DT_DRV_COMPAT zephyr_greybus_interface

#include <device.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_interface);

struct greybus_interface_config {
    const uint8_t num;
};

struct greybus_interface_data {
};

static int greybus_interface_init(struct device *dev) {

	struct greybus_interface_data *drv_data =
			(struct greybus_interface_data *)dev->driver_data;

    return 0;
}

#define DEFINE_GREYBUS_INTERFACE(_num)										\
																			\
        static struct greybus_interface_config								\
			greybus_interface_config_##_num = {								\
                .num = (uint8_t)_num,										\
        };																	\
        																	\
        static struct greybus_interface_data								\
			greybus_interface_data_##_num;  								\
        																	\
        DEVICE_AND_API_INIT(gpio_interface_##_num, DT_INST_LABEL(_num),		\
                            greybus_interface_init,							\
							&greybus_interface_data_##_num,					\
                            &greybus_interface_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	NULL)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_INTERFACE);
