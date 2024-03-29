/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/printk.h>
#include <zephyr/types.h>
#include <pm/device_runtime.h>
#include "dummy_driver.h"

static int dummy_open(const struct device *dev)
{
	return pm_device_runtime_get(dev);
}

static int dummy_close(const struct device *dev)
{
	return pm_device_runtime_put(dev);
}

static int dummy_device_pm_action(const struct device *dev,
				  enum pm_device_action action)
{
	return 0;
}

static const struct dummy_driver_api funcs = {
	.open = dummy_open,
	.close = dummy_close,
};

int dummy_init(const struct device *dev)
{
	pm_device_runtime_enable(dev);
	return 0;
}

PM_DEVICE_DEFINE(dummy_driver, dummy_device_pm_action);

DEVICE_DEFINE(dummy_driver, DUMMY_DRIVER_NAME, &dummy_init,
	      PM_DEVICE_REF(dummy_driver), NULL, NULL, APPLICATION,
	      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &funcs);
