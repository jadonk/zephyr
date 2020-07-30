/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_sim_spi

#include <device.h>
#include <drivers/spi.h>
#include <drivers/spi/spi_sim.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(spi_sim, CONFIG_SPI_LOG_LEVEL);

struct spi_sim_callback {
	uint16_t peripheral;
	spi_sim_callback_t cb;
};

struct spi_sim_config {
};

struct spi_sim_data {
	struct spi_sim_callback *callbacks;
	uint16_t num_callbacks;
	struct k_sem sem;
};

int spi_sim_callback_register(struct device *dev, uint16_t peripheral,
			      spi_sim_callback_t callback)
{
	int r;
	struct spi_sim_callback *tmp;

	if (dev == NULL || callback == NULL) {
		return -EINVAL;
	}

	struct spi_sim_data *const data =
		(struct spi_sim_data *)dev->driver_data;

	k_sem_take(&data->sem, K_FOREVER);

	for (size_t i = 0; i < data->num_callbacks; ++i) {
		if (data->callbacks[i].peripheral == peripheral) {
			r = -EALREADY;
			goto unlock;
		}
	}

	tmp = realloc(data->callbacks,
		      (data->num_callbacks + 1) * sizeof(*tmp));
	if (tmp == NULL) {
		r = -ENOMEM;
		goto unlock;
	}

	data->callbacks = tmp;
	data->callbacks[data->num_callbacks].peripheral = peripheral;
	data->callbacks[data->num_callbacks].cb = callback;
	data->num_callbacks++;

	r = 0;

unlock:
	k_sem_give(&data->sem);

	return r;
}

static int spi_sim_transceive(struct device *dev,
			      const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs,
			      const struct spi_buf_set *rx_bufs)
{
	int r;
	struct spi_sim_callback *cb;

	if (dev == NULL || config == NULL || tx_bufs == NULL) {
		return -EINVAL;
	}

	struct spi_sim_data *const data =
		(struct spi_sim_data *)dev->driver_data;

	k_sem_take(&data->sem, K_FOREVER);

	for (size_t i = 0; i < data->num_callbacks; ++i) {
		cb = &data->callbacks[i];
		if (cb->peripheral == config->slave) {
			r = cb->cb(dev, config, tx_bufs, rx_bufs);
			goto unlock;
		}
	}

	LOG_ERR("no callback present for peripheral %u", config->slave);
	r = -EIO;

unlock:
	k_sem_give(&data->sem);

	return r;
}

static int spi_sim_release(struct device *dev, const struct spi_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);

	if ((config->operation & SPI_LOCK_ON) == 0) {
		return -EINVAL;
	}

	((struct spi_config *)config)->operation &= ~SPI_LOCK_ON;

	return 0;
}

static struct spi_driver_api spi_sim_driver_api = {
	.transceive = spi_sim_transceive,
	.release = spi_sim_release,
};

static int spi_sim_init(struct device *dev)
{
	struct spi_sim_data *const data =
		(struct spi_sim_data *)dev->driver_data;

	k_sem_init(&data->sem, 1, 1);

	LOG_DBG("%s()", __func__);

	return 0;
}

#define DEFINE_SPI_SIM(n)                                                      \
	static struct spi_sim_config spi_sim_config_##n;                       \
	static struct spi_sim_data spi_sim_data_##n;                           \
	DEVICE_AND_API_INIT(spi_sim_##n, DT_INST_LABEL(n), spi_sim_init,       \
			    &spi_sim_data_##n, &spi_sim_config_##n,            \
			    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,             \
			    &spi_sim_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_SPI_SIM)
