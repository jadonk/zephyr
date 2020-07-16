#include <stdint.h>
#include <zephyr.h>
#include <greybus/manifecto/manifest.h>

#define DT_DRV_COMPAT zephyr_greybus
#include <device.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_bus);

struct greybus_config {
    const uint8_t id;
};

struct greybus_data {
	manifest_t manifest;
};

static int greybus_init(struct device *dev) {

	const struct greybus_config *const config =
			(const struct greybus_config *)dev->config_info;
	struct greybus_data *const data =
			(struct greybus_data *)dev->driver_data;

	data->manifest = manifest_new();
	if (NULL == data->manifest) {
		LOG_ERR("manifest_new() failed");
		return -ENOMEM;
	}

	LOG_INF("probed bus %u", config->id);

    return 0;
}

#define DEFINE_GREYBUS(_num)                                     	\
																	\
        static const struct greybus_config 							\
			greybus_config_##_num = {      							\
				.id = _num,                                 		\
        };															\
        															\
		static struct greybus_data greybus_data_##_num;				\
																	\
        DEVICE_INIT(greybus_##_num, "GREYBUS_" #_num,				\
                            greybus_init, &greybus_data_##_num,		\
                            &greybus_config_##_num, POST_KERNEL,	\
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS);
