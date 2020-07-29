/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_sim_i2c

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/i2c/i2c_sim.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_sim, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

struct i2c_sim_config {
};

struct i2c_sim_callback {
	uint16_t addr;
	i2c_sim_callback_t cb;
};

struct i2c_sim_data {
    uint32_t dev_config;
    size_t num_callbacks;
    struct i2c_sim_callback *callbacks;
    struct k_sem sem;
};

int i2c_sim_callback_register(struct device *dev, uint16_t addr, i2c_sim_callback_t callback)
{
    int r;
    struct i2c_sim_callback *tmp;
        struct i2c_sim_data *const data =
        (struct i2c_sim_data *)dev->driver_data; 

    if (dev == NULL || callback == NULL) {
        return -EINVAL;
    }

    k_sem_take(&data->sem, K_FOREVER);

    for(size_t i = 0; i < data->num_callbacks; ++i) {
        struct i2c_sim_callback *cb = &data->callbacks[i];
        if (cb->addr == addr) {
            LOG_ERR("callback already exists for device: %p addr: %04x", dev, addr);
            r = -EALREADY;
            goto unlock;
        }
    }

    tmp = realloc(data->callbacks, (data->num_callbacks + 1) * sizeof(*tmp));
    if (tmp == NULL) {
        LOG_ERR("unable to allocate space for i2c_sim callback");
        r = -ENOMEM;
        goto unlock;
    }

    data->callbacks = tmp;
    data->callbacks[data->num_callbacks].addr = addr;
    data->callbacks[data->num_callbacks].cb = callback;
    data->num_callbacks++;

    LOG_DBG("added i2c-sim callback: device: %p addr: %04x cb: %p", dev, addr, callback);
    r = 0;

unlock:
    k_sem_give(&data->sem);

    return r;
}

static int i2c_sim_configure(struct device *dev,
				   uint32_t dev_config)
{
    struct i2c_sim_data *const data =
        (struct i2c_sim_data *)dev->driver_data; 
    
    k_sem_take(&data->sem, K_FOREVER);

    data->dev_config = dev_config;
    LOG_DBG("%s(): set dev_config to %08x", __func__, dev_config);

    k_sem_give(&data->sem);

    return 0;
}

static int i2c_sim_transfer(struct device *dev,
				 struct i2c_msg *msgs,
				 uint8_t num_msgs,
				 uint16_t addr)
{
    int r;
    struct i2c_sim_data *const data =
        (struct i2c_sim_data *)dev->driver_data; 

    k_sem_take(&data->sem, K_FOREVER);

    for(size_t i = 0; i < data->num_callbacks; ++i) {
        struct i2c_sim_callback *cb = &data->callbacks[i];
        if (cb->addr == addr) {
            LOG_DBG("found dev: %p addr: %04x: callback: %p", dev, addr, cb->cb);
            r = cb->cb(dev, msgs, num_msgs, addr);
            goto unlock;
        }
    }

    LOG_ERR("no i2c_sim callback set up for dev: %p addr: %04x", dev, addr);
    r = -EIO;

unlock:
    k_sem_give(&data->sem);

    return r;
}

static int i2c_sim_peripheral_register(struct device *dev,
					struct i2c_slave_config *cfg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cfg);
    LOG_DBG("%s()", __func__);
    return -ENOSYS;
}

static int i2c_sim_peripheral_unregister(struct device *dev,
					  struct i2c_slave_config *cfg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cfg);
    LOG_DBG("%s()", __func__);
    return -ENOSYS;
}

static int i2c_sim_recover_bus(struct device *dev)
{
    ARG_UNUSED(dev);
    LOG_DBG("%s()", __func__);
    return 0;
}

static struct i2c_driver_api i2c_sim_driver_api = {
    .configure = i2c_sim_configure,
    .transfer = i2c_sim_transfer,
    .slave_register = i2c_sim_peripheral_register,
    .slave_unregister = i2c_sim_peripheral_unregister,
    .recover_bus = i2c_sim_recover_bus,
};

static int i2c_sim_init(struct device *dev) {

    struct i2c_sim_data *const data =
        (struct i2c_sim_data *)dev->driver_data; 
    
    LOG_DBG("%s()", __func__);
    k_sem_init(&data->sem, 1, 1);

    return 0;
}

#define DEFINE_I2C_SIM(n) \
	static struct i2c_sim_config i2c_sim_config_##n; 	\
	static struct i2c_sim_data i2c_sim_data_##n; 	\
	DEVICE_AND_API_INIT(i2c_sim_##n,				\
			    DT_INST_LABEL(n),				\
			    i2c_sim_init,			\
			    &i2c_sim_data_##n,			\
			    &i2c_sim_config_##n, POST_KERNEL,	\
			    CONFIG_I2C_INIT_PRIORITY,			\
			    &i2c_sim_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_I2C_SIM)
