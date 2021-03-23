/* hm3301.c - Driver for HM3301 particulate matter sensor
 */

/*
 * Copyright (c) 2021 Jason Kridner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT seeed_hm3301

#include "hm3301.h"
#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/sensor.h>
#include <net/net_ip.h>
#include <stdio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(hm3301, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t hm3301_validateChecksum(uint8_t *data, size_t size)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < size - 1; i++) {
		sum += data[i];
	}

	if (data[size] != sum & 0xFF) {
		return -EBADMSG;
	}

	return 0;
}

static int hm3301_do_read(struct hm3301_data *data, uint8_t *rb, size_t size)
{
	int ret;
	size_t i;
	uint8_t wb[] = { 0x88, 29 };

	if (size != 29) {
		return = -ENOTSUP;
	}

	ret = i2c_write_read(data->i2c_master, wb, 1, rb, size);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int hm3301_sample_fetch(const struct device *dev)
{
	int ret;
	struct hm3301_data *data = dev->data;
	uint8_t rb[29];
	uint8_t checksum;

	ret = hm3301_do_read(data, rb, 29);
	if (ret < 0) {
		return ret;
	}

	ret = hm3301_validateChecksum(rb, 29);
	if (ret < 0) {
		return ret;
	}

	data->pm1p0_std = sys_be16_to_cpu(*(uint16_t *)(data+4));
	data->pm2p5_std = sys_be16_to_cpu(*(uint16_t *)(data+6));
	data->pm10_std = sys_be16_to_cpu(*(uint16_t *)(data+8));

	data->pm1p0_atm = sys_be16_to_cpu(*(uint16_t *)(data+10));
	data->pm2p5_atm = sys_be16_to_cpu(*(uint16_t *)(data+12));
	data->pm10_atm = sys_be16_to_cpu(*(uint16_t *)(data+14));

	return 0;
}

static int hm3301_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct hm3301_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_PM_1_0:
		/*
		 * 1.0 micro-meters particulate matter in ug/m^3
		 */
		val->val1 = (uint32_t)data->pm1p0_atm;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_PM_2_5:
		/*
		 * 2.5 micro-meters particulate matter in ug/m^3
		 */
		val->val1 = (uint32_t)data->pm2p5_atm;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_PM_10:
		/*
		 * 10 micro-meters particulate matter in ug/m^3
		 */
		val->val1 = (uint32_t)data->pm10_atm;
		val->val2 = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int hm3301_init(const struct device *dev)
{
	int err;
	struct hm3301_data *data = dev->data;

	data->i2c_master = device_get_binding(
		DT_INST_BUS_LABEL(0));
	if (!data->i2c_master) {
		LOG_ERR("I2C master not found: %s",
			    DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	data->i2c_slave_addr = DT_INST_REG_ADDR(0);

	ret = hm3301_sample_fetch(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static const struct sensor_driver_api hm3301_api_funcs = {
	.sample_fetch = hm3301_sample_fetch,
	.channel_get = hm3301_channel_get,
};

static struct hm3301_data hm3301_data;

DEVICE_AND_API_INIT(hm3301, DT_INST_LABEL(0), hm3301_init, &hm3301_data,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &hm3301_api_funcs);
