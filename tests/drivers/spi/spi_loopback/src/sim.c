/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_SPI_SIM

#include <drivers/spi.h>
#include <drivers/spi/spi_sim.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>
#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(test_spi_sim, CONFIG_SPI_LOG_LEVEL);

#ifndef SPI_DEV_NAME
#define SPI_DEV_NAME "SPI_0"
#endif

static char *to_string(uint8_t *data, size_t len)
{
	static char buf[256];
	char *p = buf;

	memset(buf, '\0', sizeof(buf));

	for (size_t i = 0; NULL != data && i < len && p < buf + sizeof(buf) - 4;
	     ++i) {
		sprintf(p, "%02x", data[i]);
		p += 2;
		if (i < len - 1) {
			*p++ = ',';
			*p++ = ' ';
		}
	}

	return buf;
}

static size_t spi_partial_transfer(const struct spi_config *config,
				   struct spi_buf *tx, struct spi_buf *rx)
{
	if (NULL == tx || NULL == rx) {
		return 0;
	}

	const size_t len = MIN(tx->len, rx->len);

	LOG_DBG("%s(): "
		"config: {freq: %u op: %04x periph: %04x} "
		"len: %u data: [%s]",
		__func__, config->frequency, config->operation, config->slave,
		(unsigned int)len, log_strdup(to_string(tx->buf, len)));

	if (!(NULL == rx->buf || NULL == tx->buf)) {
		memcpy(rx->buf, tx->buf, len);
	}

	if (rx->buf != NULL) {
		rx->buf = (uint8_t *)rx->buf + len;
	}

	if (tx->buf != NULL) {
		tx->buf = (uint8_t *)tx->buf + len;
	}

	rx->len -= len;
	tx->len -= len;

	return len;
}

static int spi_sim_callback(struct device *dev, const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs,
			    const struct spi_buf_set *rx_bufs)
{
	ARG_UNUSED(dev);

	struct spi_buf tx;
	struct spi_buf rx;
	size_t len;
	size_t tx_idx = 0;
	size_t rx_idx = 0;

	if (tx_idx < tx_bufs->count) {
		memcpy(&tx, &tx_bufs->buffers[tx_idx], sizeof(tx));
	}

	if (rx_idx < rx_bufs->count) {
		memcpy(&rx, &rx_bufs->buffers[rx_idx], sizeof(rx));
		if (NULL != rx.buf) {
			memset(rx.buf, 0, rx.len);
		}
	}

	for (; tx_idx < tx_bufs->count && rx_idx < rx_bufs->count;) {
		len = spi_partial_transfer(config, &tx, &rx);

		if (tx.len == 0) {
			++tx_idx;
			if (tx_idx < tx_bufs->count) {
				memcpy(&tx, &tx_bufs->buffers[tx_idx],
				       sizeof(tx));
			}
		}

		if (rx.len == 0) {
			++rx_idx;
			if (rx_idx < rx_bufs->count) {
				memcpy(&rx, &rx_bufs->buffers[rx_idx],
				       sizeof(rx));
				if (NULL != rx.buf) {
					memset(rx.buf, 0, rx.len);
				}
			}
		}
	}

	return 0;
}

void sim_setup(void)
{
	int r;
	struct device *dev;

	dev = device_get_binding(SPI_DEV_NAME);
	__ASSERT(dev != NULL, "failed to get binding for " SPI_DEV_NAME);

	r = spi_sim_callback_register(dev, 0, spi_sim_callback);
	__ASSERT(r == 0, "failed to register spi_sim callback: %d", r);
}

#endif /* CONFIG_SPI_SIM */
