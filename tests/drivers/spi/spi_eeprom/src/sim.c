/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_SPI_SIM

#include <drivers/spi.h>
#include <drivers/spi/spi_sim.h>
#include <drivers/eeprom.h>
#include <stdio.h>
#include <sys/util.h>
#include <zephyr.h>

#include <logging/log.h>
//LOG_MODULE_REGISTER(test_spi_sim, CONFIG_SPI_LOG_LEVEL);
LOG_MODULE_REGISTER(test_spi_sim, LOG_LEVEL_DBG);

#ifndef SPI_DEV_NAME
#define SPI_DEV_NAME "SPI_0"
#endif

static char *to_string(uint8_t *data, size_t len)
{
	static char buf[65536];
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

/* AT25 instruction set (least-significant nibble) */
typedef enum {
	AT25_WRSR = 1, /* Write STATUS register        */
	AT25_WRITE = 2, /* Write data to memory array   */
	AT25_READ = 3, /* Read data from memory array  */
	AT25_WRDI = 4, /* Reset the write enable latch */
	AT25_RDSR = 5, /* Read STATUS register         */
	AT25_WREN = 6, /* Set the write enable latch   */
	AT25_EONE = 0x52, /* Erase One Sector in Memory Array */
	AT25_EALL = 0x62, /* Erase All Sectors in Memory Array */
	AT25_RDR = 0x15, /* Read Manufacturer and Product ID */
} at25_op_t;

static const char *at25_op_to_string(at25_op_t op) {
	switch(op) {
	case AT25_WRSR:  return "WRSR ";
	case AT25_WRITE: return "WRITE";
	case AT25_READ:  return "READ ";
	case AT25_WRDI:  return "WRDI ";
	case AT25_RDSR:  return "RDSR ";
	case AT25_WREN:  return "WREN ";
	case AT25_EONE:  return "EONE ";
	case AT25_EALL:  return "EALL ";
	case AT25_RDR:   return "RDR  ";
	default:         return "     ";
	}
}

/* AT25 status register bits */
typedef enum {
	AT25_STATUS_WIP = BIT(0), /* Write-In-Process   (RO) */
	AT25_STATUS_WEL = BIT(1), /* Write Enable Latch (RO) */
	AT25_STATUS_BP0 = BIT(2), /* Block Protection 0 (RW) */
	AT25_STATUS_BP1 = BIT(3), /* Block Protection 1 (RW) */
} at25_status_t;

static int spi_sim_callback(struct device *dev, const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs,
			    const struct spi_buf_set *rx_bufs)
{
	static uint8_t status;
	static uint8_t data[256];
	static bool write;
	static uint32_t addr;
	const struct spi_buf *tx = &tx_bufs->buffers[0];
	const struct spi_buf *rx;

	at25_op_t op = ((uint8_t *)tx->buf)[0] & (~0x08);

	switch(op) {
	case AT25_WRSR:
		break;
	case AT25_WRITE:

		write = true;

		addr = 0;
		/* A[23..17] are don't care bits for the 512k version */
		/* addr |= (uint32_t)(((uint8_t *)tx->buf)[0]) << 16; */
		/* A[16] should be a zero bit for the 512k version */
		addr |= ((uint32_t)(((uint8_t *)tx->buf)[1]) << 8) & 7;
		addr |= (uint32_t)(((uint8_t *)tx->buf)[2]) << 0;

		tx = &tx_bufs->buffers[1];

		LOG_DBG("%s(): %s "
			"len: %u data: [%s]",
			__func__, at25_op_to_string(op),
			(unsigned int)tx->len, log_strdup(to_string(tx->buf, tx->len)));

		memcpy(&data[addr], (uint8_t *)tx->buf, tx->len);

		write = false;

		break;

	case AT25_READ:

		write = false;

		addr = 0;
		/* A[23..17] are don't care bits for the 512k version */
		/* addr |= (uint32_t)(((uint8_t *)tx->buf)[0]) << 16; */
		/* A[16] should be a zero bit for the 512k version */
		addr |= ((uint32_t)(((uint8_t *)tx->buf)[1]) << 8) & 7;
		addr |= (uint32_t)(((uint8_t *)tx->buf)[2]) << 0;

		rx = &rx_bufs->buffers[1];

		memcpy(rx->buf, &data[addr], rx->len);

		LOG_DBG("%s(): %s "
			"len: %u data: [%s]",
			__func__, at25_op_to_string(op),
			(unsigned int)rx->len, log_strdup(to_string(rx->buf, rx->len)));

		break;

	case AT25_WRDI:
		break;

	case AT25_RDSR:

		LOG_DBG("%s(): %s "
			"len: %u data: [%s]",
			__func__, at25_op_to_string(op),
			(unsigned int)tx->len, log_strdup(to_string(tx->buf, tx->len)));

		rx = &rx_bufs->buffers[0];
		((uint8_t *)rx->buf)[1] = status;
		break;

	case AT25_WREN:

		LOG_DBG("%s(): %s "
			"len: %u data: [%s]",
			__func__, at25_op_to_string(op),
			(unsigned int)tx->len, log_strdup(to_string(tx->buf, tx->len)));

		status |= AT25_STATUS_WEL;
		break;
	case AT25_EONE:
		break;
	case AT25_EALL:
		break;
	case AT25_RDR:
		break;
	default:

		__ASSERT(1 == 0, "invalid opcode %u", op);

		break;
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
