#include <stdint.h>
#include <zephyr.h>
#include <greybus/manifecto/manifest.h>

#define DT_DRV_COMPAT zephyr_greybus
#include <device.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test_bus);

#include "bus.h"

struct greybus_config {
    const uint8_t id;
    const uint8_t version_major;
    const uint8_t version_minor;
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

	LOG_INF("probed greybus: %u major: %u minor: %u",
		config->id, config->version_major, config->version_minor);

    return 0;
}

static int greybus_add_interface(struct device *dev, uint16_t vendor_string_id,
		uint16_t product_string_id) {

	struct greybus_data *const data =
			(struct greybus_data *)dev->driver_data;

	return manifest_add_interface_desc(data->manifest, vendor_string_id, product_string_id);
}

static int greybus_add_string(struct device *dev, uint8_t id, const char *string_) {

	struct greybus_data *const data =
			(struct greybus_data *)dev->driver_data;

	return manifest_add_string_desc(data->manifest, id, string_);
}

static int greybus_add_bundle(struct device *dev, uint8_t id, BundleClass class_) {

	struct greybus_data *const data =
			(struct greybus_data *)dev->driver_data;

	return manifest_add_bundle_desc(data->manifest, id, class_);
}

static int greybus_add_cport(struct device *dev, uint8_t id, BundleClass class_, CPortProtocol protocol) {

	struct greybus_data *const data =
			(struct greybus_data *)dev->driver_data;

	return manifest_add_cport_desc(data->manifest, id, class_, protocol);
}

static manifest_t greybus_get_manifest(struct device *dev) {

	struct greybus_data *const data =
			(struct greybus_data *)dev->driver_data;

	return data->manifest;
}

static const struct bus_api greybus_api = {
	.add_interface = greybus_add_interface,
	.add_string = greybus_add_string,
	.add_bundle = greybus_add_bundle,
	.add_cport = greybus_add_cport,
	.get_manifest = greybus_get_manifest,
};

#define DEFINE_GREYBUS(_num)                            \
														\
        static const struct greybus_config 				\
			greybus_config_##_num = {      				\
				.id = _num,                             \
				.version_major = 						\
					DT_INST_PROP(_num, version_major),	\
				.version_minor = 						\
					DT_INST_PROP(_num, version_minor),	\
        };												\
        												\
		static struct greybus_data greybus_data_##_num;	\
														\
        DEVICE_AND_API_INIT(greybus_##_num,				\
			"GREYBUS_" #_num,							\
			greybus_init, &greybus_data_##_num,			\
			&greybus_config_##_num, POST_KERNEL,		\
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
			&greybus_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GREYBUS);
