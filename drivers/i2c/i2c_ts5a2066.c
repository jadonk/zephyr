/*
 * Copyright (c) 2021 Jason Kridner, BeagleBoard.org Foundation
 * Copyright (c) 2020 Innoseis BV
 *
 * Based on i2c_tca9656a.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ts5a2066

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(ts5a2066, CONFIG_I2C_LOG_LEVEL);

struct ts5a2066_config {
	const struct device *bus;
	const struct device *gpiodev;
	gpio_pin_t gpio;
};

struct ts5a2066_data {
	struct k_mutex lock;
};


int ts5a2066_configure(const struct device *dev, uint32_t dev_config)
{
	const struct ts5a2066_config * config = dev->config;

	return i2c_configure(config->bus, dev_config);
}

static int ts5a2066_enable(const struct device *dev)
{
	int res = 0;
	const struct ts5a2066_config *config = dev->config;

	res = gpio_pin_set(config->gpiodev, config->gpio, 1);

	return res;
}

static int ts5a2066_disable(const struct device *dev)
{
	int res = 0;
	const struct ts5a2066_config *config = dev->config;

	res = gpio_pin_set(config->gpiodev, config->gpio, 0);

	return res;
}
static int ts5a2066_transfer(const struct device *dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs,
			     uint16_t addr)
{
	struct ts5a2066_data *data = dev->data;
	const struct ts5a2066_config *config = dev->config;
	int res;

	res = k_mutex_lock(&data->lock, K_MSEC(5000));
	if (res != 0) {
		return res;
	}

	ts5a2066_enable(dev);
	res = i2c_transfer(config->bus, msgs, num_msgs, addr);
	ts5a2066_disable(dev);

	k_mutex_unlock(&data->lock);
	return res;
}

const struct i2c_driver_api ts5a2066_api_funcs = {
	.configure = ts5a2066_configure,
	.transfer = ts5a2066_transfer,
};

static int ts5a2066_init(const struct device *dev)
{
	//struct ts5a2066_data * data = dev->data;
	//const struct ts5a2066_config * config = dev->config;

	return 0;
}

#define	DEFINE_TS5A2066(_nodeid)					\
									\
static struct ts5a2066_data ts5a2066_dev_data_##_nodeid;		\
									\
static const struct ts5a2066_config ts5a2066_dev_cfg_##_nodeid = {	\
	.bus		= DEVICE_DT_GET(DT_PARENT(DT_NODE_PATH(_nodeid))),	\
	.gpiodev	= DT_GPIO_CTLR(_nodeid, gpios),			\
	.gpio		= DT_GPIO_PIN(_nodeid, gpios),			\
};									\
									\
DEVICE_DT_INST_DEFINE(_nodeid,						\
	    ts5a2066_init,						\
	    device_pm_control_nop,					\
	    &ts5a2066_dev_data_##_nodeid,				\
	    &ts5a2066_dev_cfg_##_nodeid,				\
	    POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_TS5A2066)
