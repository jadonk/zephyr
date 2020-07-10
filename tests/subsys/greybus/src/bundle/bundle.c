#define DT_DRV_COMPAT zephyr_greybus_bundle

#include <device.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_bundle);

struct greybus_bundle_config {
    const uint8_t num;
    const char *const greybus_interface_name;
};

struct interface;
struct greybus_bundle_data {
    struct interface *interface;
};

static int greybus_bundle_init(struct device *dev) {

	struct greybus_bundle_data *drv_data =
			(struct greybus_bundle_data *)dev->driver_data;
	struct greybus_bundle_config *config =
        (struct greybus_bundle_config *)dev->config_info;

    ARG_UNUSED(drv_data);
    ARG_UNUSED(config);

    return 0;
}
    
#define DEFINE_GREYBUS_BUNDLE(_num)										\
																			\
        static struct greybus_bundle_config								\
			greybus_bundle_config_##_num = {								\
                .num = (uint8_t)_num,												\
                .greybus_interface_name = \
                    DT_LABEL(DT_PARENT(DT_DRV_INST(_num))), \
        };																	\
        																	\
        static struct greybus_bundle_data								\
			greybus_bundle_data_##_num;								\
        																	\
        DEVICE_AND_API_INIT(gpio_bundle_##_num, DT_INST_LABEL(_num),		\
                            greybus_bundle_init,							\
							&greybus_bundle_data_##_num,					\
                            &greybus_bundle_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	NULL)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_BUNDLE);
