/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Backend API for simulated SPI
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_SIM_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_SIM_H_

#include <zephyr/types.h>
#include <drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*spi_sim_callback_t)(struct device *dev,
			  const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs,
			  const struct spi_buf_set *rx_bufs);
int spi_sim_callback_register(struct device *dev, uint16_t peripheral, spi_sim_callback_t cb);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_SIM_H_ */
