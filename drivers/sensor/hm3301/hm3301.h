/*
 * Copyright (c) 2021 Jason Kridner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_HM3301_H__
#define __SENSOR_HM3301_H__

#include <device.h>
#include <zephyr/types.h>

struct hm3301_data {
	const struct device *i2c_master;
	uint16_t i2c_slave_addr;

	uint16_t pm1p0_std;
	uint16_t pm2p5_std;
	uint16_t pm10_std;

	uint16_t pm1p0_atm;
	uint16_t pm2p5_atm;
	uint16_t pm10_atm;
};

#endif /* __SENSOR_HM3301_H__ */
