#define DT_DRV_COMPAT zephyr_greybus_gpio_controller

#include <device.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_gpio_control);

struct greybus_gpio_control_config {
    const uint8_t num;
    const char *const greybus_interface_name;
    const char *const greybus_bundle_name;
    const char *const greybus_gpio_controller_name;
};

struct greybus_gpio_control_data {
    struct device *greybus_gpio_controller;
};

static int greybus_gpio_control_init(struct device *dev) {

	struct greybus_gpio_control_data *drv_data =
        (struct greybus_gpio_control_data *)dev->driver_data;
    struct greybus_gpio_control_config *config =
        (struct greybus_gpio_control_config *)dev->config_info;

    drv_data->greybus_gpio_controller =
        device_get_binding(config->greybus_gpio_controller_name);

    return 0;
}

#define DEFINE_GREYBUS_GPIO_CONTROL(_num)										\
																			\
        static struct greybus_gpio_control_config								\
			greybus_gpio_control_config_##_num = {								\
                .num = (uint8_t)_num,												\
                .greybus_interface_name = \
                    DT_LABEL(DT_PARENT(DT_PARENT(DT_DRV_INST(_num)))), \
                .greybus_bundle_name = \
                    DT_LABEL(DT_PARENT(DT_DRV_INST(_num))), \
                .greybus_gpio_controller_name = \
                    DT_LABEL(DT_PHANDLE(DT_DRV_INST(_num), greybus_gpio_controller)), \
        };																	\
        																	\
        static struct greybus_gpio_control_data								\
			greybus_gpio_control_data_##_num;								\
        																	\
        DEVICE_AND_API_INIT(gpio_gpio_control_##_num, DT_INST_LABEL(_num),		\
                            greybus_gpio_control_init,							\
							&greybus_gpio_control_data_##_num,					\
                            &greybus_gpio_control_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	NULL)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_GPIO_CONTROL);
