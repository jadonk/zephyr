#define DT_DRV_COMPAT zephyr_greybus_string
#include <stdint.h>
#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_string);

struct greybus_string_config {
    const uint8_t id;
    const char *const string;
};

static int greybus_string_init(struct device *dev) {
	ARG_UNUSED(dev);
	const struct greybus_string_config *config =
			(const struct greybus_string_config *)dev->config_info;

	LOG_INF("probed %u: %s", config->id, config->string);

    return 0;
}

#define DEFINE_GREYBUS_STRING(_num)                                                   \
                                                                                \
        static const struct greybus_string_config greybus_string_config_##_num = {      \
			.id = _num,                                 \
			.string = DT_INST_PROP(_num, greybus_string),                         \
        };                                                                      \
                                                                                \
        DEVICE_INIT(greybus_string_##_num, DT_INST_LABEL(_num),               \
                            greybus_string_init, NULL,               \
                            &greybus_string_config_##_num, POST_KERNEL,               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_STRING);
