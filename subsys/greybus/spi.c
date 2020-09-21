/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <device.h>
#include <drivers/spi.h>
#include <errno.h>
#include <greybus/greybus.h>
#include <greybus/platform.h>
#include <stdlib.h>
#include <string.h>
#include <greybus-utils/utils.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#if defined(CONFIG_BOARD_NATIVE_POSIX_64BIT) \
    || defined(CONFIG_BOARD_NATIVE_POSIX_32BIT) \
    || defined(CONFIG_BOARD_NRF52_BSIM)
#include <unistd.h>
extern int usleep(useconds_t usec);
#else
#include <posix/unistd.h>
#endif

#include "spi-gb.h"

#define GB_SPI_VERSION_MAJOR 0
#define GB_SPI_VERSION_MINOR 1

/**
 * @brief Returns the major and minor Greybus SPI protocol version number
 *        supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_version(struct gb_operation *operation)
{
    struct gb_spi_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SPI_VERSION_MAJOR;
    response->minor = GB_SPI_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static int device_spi_get_master_config(struct device *dev, struct gb_spi_master_config_response *response)
{
    int ret;

    struct device *const gb_spidev = gb_spidev_from_zephyr_spidev(dev);
    if (gb_spidev == NULL) {
    	return -ENODEV;
    }

    const struct gb_platform_spi_api *const api = gb_spidev->api;
    __ASSERT_NO_MSG(api != NULL);

    ret = api->controller_config_response(gb_spidev, response);
    __ASSERT_NO_MSG(ret == 0);

    return 0;
}

/**
 * @brief Returns a set of configuration parameters related to SPI master.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_master_config(struct gb_operation *operation)
{
    struct gb_spi_master_config_response *response;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_spi_get_master_config(gb_cport_to_device(operation->cport), response);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    /* TODO: use compile-time byte swap operators in platform/spi.c
     * so byteswapping is eliminated here */
    response->bpw_mask = sys_cpu_to_le32(response->bpw_mask);
    response->min_speed_hz = sys_cpu_to_le32(response->min_speed_hz);
    response->max_speed_hz = sys_cpu_to_le32(response->max_speed_hz);
    response->mode = sys_cpu_to_le16(response->mode);
    response->flags = sys_cpu_to_le16(response->flags);
    response->num_chipselect = sys_cpu_to_le16(response->num_chipselect);

    return GB_OP_SUCCESS;
}

static int device_spi_get_device_config(struct device *dev, uint8_t cs, struct gb_spi_device_config_response *response)
{
    int ret;

    struct device *const gb_spidev = gb_spidev_from_zephyr_spidev(dev);
    struct gb_platform_spi_api *const api = (struct gb_platform_spi_api *)gb_spidev->api;
    __ASSERT_NO_MSG(api != NULL);

    ret = api->peripheral_config_response(gb_spidev, cs, response);
    __ASSERT_NO_MSG(ret == 0);

    return 0;
}

/**
 * @brief Get configuration parameters from chip
 *
 * Returns a set of configuration parameters taht related to SPI device is
 * selected.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_device_config(struct gb_operation *operation)
{
    struct gb_spi_device_config_request *request;
    struct gb_spi_device_config_response *response;
    size_t request_size;
    uint8_t cs;
    int ret = 0;

    request_size = gb_operation_get_request_payload_size(operation);
    if (request_size < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    cs = request->chip_select;
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get selected chip of configuration */
    ret = device_spi_get_device_config(gb_cport_to_device(operation->cport), cs, response);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    /* TODO: use compile-time byte swap operators in platform/spi.c
     * so byteswapping is eliminated here */
    response->mode = sys_cpu_to_le16(response->mode);
    response->max_speed_hz = sys_cpu_to_le32(response->max_speed_hz);

    return GB_OP_SUCCESS;
}

static inline int device_spi_lock(struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

static inline int device_spi_unlock(struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

static inline int device_spi_select(struct device *dev, uint8_t chip_select)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chip_select);
    return 0;
}

static int device_spi_deselect(struct device *dev, uint8_t chip_select)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chip_select);
    return 0;
}

static int request_to_spi_config(const struct gb_spi_transfer_request *const request,
	const size_t freq, const uint8_t bits_per_word, struct device *const spi_dev, struct spi_config *const spi_config, struct spi_cs_control *ctrl)
{
    struct device *const gb_spidev = gb_spidev_from_zephyr_spidev(spi_dev);
    if (gb_spidev == NULL) {
    	return -ENODEV;
    }

    const struct gb_platform_spi_api *const api = gb_spidev->api;
    __ASSERT_NO_MSG(api != NULL);

    spi_config->frequency = freq;
    spi_config->slave = request->chip_select;
    spi_config->operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(bits_per_word);

    if (request->mode & GB_SPI_MODE_CPHA) {
		spi_config->operation |= SPI_MODE_CPHA;
    }

    if (request->mode & GB_SPI_MODE_CPOL) {
		spi_config->operation |= SPI_MODE_CPOL;
    }

    if (request->mode & GB_SPI_MODE_CS_HIGH) {
		spi_config->operation |= SPI_CS_ACTIVE_HIGH;
    }

    if (request->mode & GB_SPI_MODE_CS_HIGH) {
		spi_config->operation |= SPI_CS_ACTIVE_HIGH;
    }

    if (request->mode & GB_SPI_MODE_LSB_FIRST) {
		spi_config->operation |= SPI_TRANSFER_LSB;
    }

    if (request->mode & GB_SPI_MODE_3WIRE) {
		/* Do nothing. I think this is the presumptive case */
    }

    if (request->mode & GB_SPI_MODE_LOOP) {
		spi_config->operation |= SPI_MODE_LOOP;
    }

    if (request->mode & GB_SPI_MODE_NO_CS) {
		if (spi_config->cs != NULL) {
			/* LOG_DBG("GB_SPI_MODE_NO_CS flag given but spi_config->cs is "
				"non-NULL (%p)", spi_config->cs); */
		}
	}

    if (request->mode & GB_SPI_MODE_READY) {
		/* LOG_DBG("GB_SPI_MODE_READY not handled"); */
    }

    if (
		false
		|| ((request->mode & GB_SPI_FLAG_HALF_DUPLEX) != 0)
		|| ((request->mode & GB_SPI_FLAG_NO_RX) != 0)
		|| ((request->mode & GB_SPI_FLAG_NO_TX) != 0)
	) {
		/* LOG_DBG("GB_SPI_FLAG_{HALF_DUPLEX,NO_{RX,TX}} not handled"); */
    }

    if (!api->get_cs_control(gb_spidev, request->chip_select, ctrl)) {
        spi_config->cs = ctrl;
    } else {
        spi_config->cs = NULL;
    }

    return 0;
}

/**
 * @brief Performs a SPI transaction as one or more SPI transfers, defined
 *        in the supplied array.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_transfer(struct gb_operation *operation)
{
    /*
     * FIXME: This function needs to be rewritten to properly use Zephyr's
     * SPI API. Specifically, Zephyr's API already handles batch transfers
     * and chip selects properly, while this code does not (e.g. 'selected'
     * only accounts for the possibility of 1 chip_select).
     *
     * Actually, on review, it does not seem that Zephyr's SPI API works
     * for batched transfers when transfers might use different GPIO lines.
     * We are forced to perform one transfer at a time until that is fixed.
     *
     * Will need to malloc
     * 1) struct spi_buf[] for tx
     * 2) struct spi_buf[] for rx
     * 3) struct spi_cs_control[]
     */

    struct gb_spi_transfer_desc *desc;
    struct gb_spi_transfer_request *request;
    struct gb_spi_transfer_response *response;
    struct spi_config spi_config;
    struct spi_cs_control spi_cs_control;
    struct spi_buf tx_buf;
    struct spi_buf rx_buf;
    const struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1,
    };
    struct spi_buf_set rx_bufs;

    uint32_t size = 0, freq = 0;
    uint8_t *write_data, *read_buf;
    bool selected = false;
    int i, op_count;
    int ret = 0, errcode = GB_OP_SUCCESS;
    size_t request_size;
    size_t expected_size;

    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    __ASSERT_NO_MSG(bundle != NULL);

    request_size = gb_operation_get_request_payload_size(operation);
    if (request_size < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    op_count = sys_le16_to_cpu(request->count);

    expected_size = sizeof(*request) +
                    op_count * sizeof(request->transfers[0]);
    if (request_size < expected_size) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    write_data = (uint8_t *)&request->transfers[op_count];

    for (i = 0, size = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        if (desc->rdwr & GB_SPI_XFER_READ) {
            size += sys_le32_to_cpu(desc->len);
        }
    }

    response = gb_operation_alloc_response(operation, size);
    if (!response) {
        return GB_OP_NO_MEMORY;
    }
    read_buf = response->data;

    /* lock SPI bus */
    ret = device_spi_lock(bundle->dev);
    if (ret) {
        return (ret == -EINVAL)? GB_OP_INVALID : GB_OP_UNKNOWN_ERROR;
    }

    /* parse all transfer request from AP host side */
    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        freq = sys_le32_to_cpu(desc->speed_hz);

        /* assert chip-select pin */
        if (!selected) {
            ret = device_spi_select(bundle->dev, request->chip_select);
            if (ret) {
                goto spi_err;
            }
            selected = true;
        }

        /* setup SPI transfer */
        tx_buf.buf = write_data;
        tx_buf.len = sys_le32_to_cpu(desc->len);
        /* If rdwr without GB_SPI_XFER_READ flag, not need to pass read buffer */
        if (desc->rdwr & GB_SPI_XFER_READ) {
        	rx_buf.buf = read_buf;
        	rx_buf.len = tx_buf.len;
        	rx_bufs.buffers = &rx_buf;
            rx_bufs.count = 1;
        } else {
            rx_bufs.buffers = NULL;
            rx_bufs.count = 0;
        }

        /* set SPI configuration */
        ret = request_to_spi_config(request, freq, desc->bits_per_word, bundle->dev, &spi_config, &spi_cs_control);
        if (ret) {
			goto spi_err;
        }
        /* In Zephyr, spi lock is on a per-call basis */
        spi_config.operation |= SPI_LOCK_ON;

        /* start SPI transfer */
        ret = spi_transceive(bundle->dev, &spi_config, &tx_bufs, &rx_bufs);
        if (ret) {
            goto spi_err;
        }
        /* move to next gb_spi_transfer data buffer */
        write_data += sys_le32_to_cpu(desc->len);

        /* If rdwr without GB_SPI_XFER_READ flag, not need to resize
         * read buffer
         */
        if (desc->rdwr & GB_SPI_XFER_READ) {
            read_buf += sys_le32_to_cpu(desc->len);
            rx_buf.buf = (uint8_t *)rx_buf.buf + sys_le32_to_cpu(desc->len);
        }

        if (sys_le16_to_cpu(desc->delay_usecs) > 0) {
            usleep(sys_le16_to_cpu(desc->delay_usecs));
        }

        /* if cs_change enable, change the chip-select pin signal */
        if (desc->cs_change) {
            /* force deassert chip-select pin */
            ret = device_spi_deselect(bundle->dev, request->chip_select);
            if (ret) {
                goto spi_err;
            }
            selected = false;
        }
    }

spi_err:
    errcode = ret;

    if (selected) {
        /* deassert chip-select pin */
        ret = device_spi_deselect(bundle->dev, request->chip_select);
        if (ret) {
            errcode = ret;
        }
    }

    /* unlock SPI bus*/
    ret = device_spi_unlock(bundle->dev);
    if (ret) {
        errcode = ret;
    }

    if (errcode) {
        /* get error code */
        errcode = (errcode == -EINVAL)? GB_OP_INVALID : GB_OP_UNKNOWN_ERROR;
    }
    return errcode;
}

/**
 * @brief Greybus SPI protocol initialize function
 *
 * @param cport CPort number
 * @param bundle Greybus bundle handle
 * @return 0 on success, negative errno on error
 */
static int gb_spi_init(unsigned int cport, struct gb_bundle *bundle)
{
    bundle->dev = gb_cport_to_device(cport);
    if (!bundle->dev) {
        return -EIO;
    }
    return 0;
}

/**
 * @brief Greybus SPI protocol deinitialize function
 *
 * @param cport CPort number
 * @param bundle Greybus bundle handle
 */
static void gb_spi_exit(unsigned int cport, struct gb_bundle *bundle)
{
}

/**
 * @brief Greybus SPI protocol operation handler
 */
static struct gb_operation_handler gb_spi_handlers[] = {
    GB_HANDLER(GB_SPI_PROTOCOL_VERSION, gb_spi_protocol_version),
    GB_HANDLER(GB_SPI_TYPE_MASTER_CONFIG, gb_spi_protocol_master_config),
    GB_HANDLER(GB_SPI_TYPE_DEVICE_CONFIG, gb_spi_protocol_device_config),
    GB_HANDLER(GB_SPI_PROTOCOL_TRANSFER, gb_spi_protocol_transfer),
};

static struct gb_driver gb_spi_driver = {
    .init = gb_spi_init,
    .exit = gb_spi_exit,
    .op_handlers = gb_spi_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_spi_handlers),
};

/**
 * @brief Register Greybus SPI protocol
 *
 * @param cport CPort number
 * @param bundle Bundle number.
 */
void gb_spi_register(int cport, int bundle)
{
    gb_register_driver(cport, bundle, &gb_spi_driver);
}

