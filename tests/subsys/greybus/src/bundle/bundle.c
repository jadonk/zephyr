#define DT_DRV_COMPAT zephyr_greybus_bundle
#include <stdint.h>
#include <device.h>
#include <devicetree.h>
#include <zephyr.h>
#include <greybus/manifecto/manifest.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_bundle);

struct greybus_bundle_config {
    const uint8_t id;
    const BundleClass class;
};

static int greybus_bundle_init(struct device *dev) {
	ARG_UNUSED(dev);
//	const struct greybus_bundle_config *config =
//			(const struct greybus_bundle_config *)dev->config_info;

	//LOG_INF("probed %u %u", config->id, config->class_);

    return 0;
}

#define DEFINE_GREYBUS_BUNDLE(_num)                                                   \
                                                                                \
        static const struct greybus_bundle_config greybus_bundle_config_##_num = {      \
			.id = _num,                                 \
			.class = DT_INST_PROP(_num, bundle_class),                         \
        };                                                                      \
                                                                                \
        DEVICE_AND_API_INIT(greybus_bundle_##_num, DT_INST_LABEL(_num),               \
        					greybus_bundle_init, NULL,               \
                            &greybus_bundle_config_##_num, POST_KERNEL,               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                 \
                            NULL)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS_BUNDLE);
