/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_driver.h"

#include <kernel.h>

struct test_driver_data {
	bool ongoing;
	bool async;
	struct k_sem sync;
};

static int test_driver_action(const struct device *dev,
			      enum pm_device_action action)
{
	struct test_driver_data *data = dev->data;

	data->ongoing = true;

	if (data->async) {
		k_sem_take(&data->sync, K_FOREVER);
		data->async = false;
	}

	data->ongoing = false;

	return 0;
}

void test_driver_pm_async(const struct device *dev)
{
	struct test_driver_data *data = dev->data;

	data->async = true;
}

void test_driver_pm_done(const struct device *dev)
{
	struct test_driver_data *data = dev->data;

	k_sem_give(&data->sync);
}

bool test_driver_pm_ongoing(const struct device *dev)
{
	struct test_driver_data *data = dev->data;

	return data->ongoing;
}

int test_driver_init(const struct device *dev)
{
	struct test_driver_data *data = dev->data;

	k_sem_init(&data->sync, 0, 1);

	return 0;
}

PM_DEVICE_DEFINE(test_driver, test_driver_action);

static struct test_driver_data data;

DEVICE_DEFINE(test_driver, "test_driver", &test_driver_init,
	      PM_DEVICE_REF(test_driver), &data, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
