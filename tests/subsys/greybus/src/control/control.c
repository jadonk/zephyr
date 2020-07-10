#include <stdint.h>

#define DT_DRV_COMPAT zephyr_greybus_control

#include <device.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_control);

struct greybus_control_config {
    const uint8_t num;
    const char *const greybus_interface_name;
    const char *const greybus_bundle_name;
};

struct interface;
struct greybus_control_data {
};

static int greybus_control_init(struct device *dev) {

	struct greybus_control_data *drv_data =
			(struct greybus_control_data *)dev->driver_data;

    ARG_UNUSED(dev);
    ARG_UNUSED(drv_data);

    return 0;
}

#define DEFINE_GREYBUS_CONTROL(_num)										\
																			\
        static struct greybus_control_config								\
			greybus_control_config_##_num = {								\
                .num = (uint8_t) _num,												\
                .greybus_interface_name = \
                    DT_LABEL(DT_PARENT(DT_PARENT(DT_DRV_INST(_num)))), \
                .greybus_bundle_name = \
                    DT_LABEL(DT_PARENT(DT_DRV_INST(_num))), \
        };																	\
        																	\
        static struct greybus_control_data								\
			greybus_control_data_##_num;								\
        																	\
        DEVICE_AND_API_INIT(gpio_control_##_num, DT_INST_LABEL(_num),		\
                            greybus_control_init,							\
							&greybus_control_data_##_num,					\
                            &greybus_control_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	NULL)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_CONTROL);
