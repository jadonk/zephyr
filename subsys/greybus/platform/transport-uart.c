#include <device.h>
#include <drivers/uart.h>
#include <errno.h>
#include <logging/log.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include "transport.h"

LOG_MODULE_REGISTER(greybus_transport_uart, LOG_LEVEL_DBG);

/* Based on UniPro, from Linux */
#define CPORT_ID_MAX 4095

static int getMessage(struct device *dev, struct gb_operation_hdr **msg);
static int sendMessage(struct device *dev, struct gb_operation_hdr *msg);

static struct device *uart_dev;
static pthread_t receive_thread;

K_PIPE_DEFINE(gb_xport_uart_pipe, 64, 1);

static void *thread_fun(void *arg)
{
	int r;
	int i = 0;
	struct gb_operation_hdr *msg = NULL;
    unsigned int cport = -1;

	for (;;) {
		r = getMessage(uart_dev, &msg);
		if (r < 0) {
			LOG_DBG("failed to receive message (%d)", r);
			break;
		}

		LOG_HEXDUMP_DBG(msg, sys_le16_to_cpu(msg->size), "RX:");

		cport = sys_le16_to_cpu(*((uint16_t *)msg->pad));

		r = greybus_rx_handler(cport, msg, sys_le16_to_cpu(msg->size));
		if (r < 0) {
			LOG_DBG("failed to handle message %u: size: %u, id: %u, type: %u",
				i, sys_le16_to_cpu(msg->size), sys_le16_to_cpu(msg->id),
				msg->type);
		}
	}

	return NULL;
}

static int uart_rx(const struct device *dev, uint8_t *buf, size_t len, int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);
	size_t nread;

	for( ;len > 0; ) {
		int r = k_pipe_get(&gb_xport_uart_pipe, buf, len, &nread, len, K_FOREVER);
		if (r < 0) {
			LOG_ERR("k_pipe_get() returned %d", r);
			continue;
		}
		buf += nread;
		len -= nread;
	}

	return 0;
}

static int getMessage(struct device *dev, struct gb_operation_hdr **msg)
{
	int r;
	void *tmp;
	size_t msg_size;
	size_t payload_size;
	size_t remaining;
	size_t offset;
	size_t recvd;

	if (NULL == msg) {
		LOG_DBG("One or more arguments were NULL or invalid");
		r = -EINVAL;
		goto out;
	}

	tmp = realloc(*msg, sizeof(**msg));
	if (NULL == tmp) {
		LOG_DBG("Failed to allocate memory");
		r = -ENOMEM;
		goto out;
	}
	*msg = tmp;

read_header:
	for (remaining = sizeof(**msg), offset = 0, recvd = 0; remaining;
	     remaining -= recvd, offset += recvd, recvd = 0) {

		r = uart_rx(dev, &((uint8_t *)*msg)[offset], remaining, SYS_FOREVER_MS);
		if (r < 0) {
			LOG_DBG("uart_rx failed (%d)", r);
			k_usleep(100);
			continue;
		}
		recvd = remaining;
	}

	msg_size = sys_le16_to_cpu((*msg)->size);
	if (msg_size < sizeof(struct gb_operation_hdr)) {
		LOG_DBG("invalid message size %u", (unsigned)msg_size);
		goto read_header;
	}

	payload_size = msg_size - sizeof(**msg);
	if (payload_size > GB_MAX_PAYLOAD_SIZE) {
		LOG_DBG("invalid payload size %u", (unsigned)payload_size);
		goto read_header;
	}

	if (payload_size > 0) {
		tmp = realloc(*msg, msg_size);
		if (NULL == tmp) {
			LOG_DBG("Failed to allocate memory");
			r = -ENOMEM;
			goto freemsg;
		}
		*msg = tmp;

		for (remaining = payload_size, offset = sizeof(**msg),
		    recvd = 0;
		     remaining;
		     remaining -= recvd, offset += recvd, recvd = 0) {

			r = uart_rx(dev, &((uint8_t *)*msg)[offset], remaining, SYS_FOREVER_MS);
			if (r < 0) {
				LOG_DBG("uart_rx failed (%d)", r);
				k_usleep(100);
				continue;
			}
			recvd = remaining;
		}
	}

	r = msg_size;
	goto out;

freemsg:
	free(*msg);
	*msg = NULL;

out:
	return r;
}

static int sendMessage(struct device *dev, struct gb_operation_hdr *msg)
{
	int r;
	size_t offset;
	size_t remaining;
	size_t written;

	for (remaining = sys_le16_to_cpu(msg->size), offset = 0, written = 0;
	     remaining; remaining -= written, offset += written, written = 0) {

		uart_poll_out(dev, &((uint8_t *)msg)[offset]);
		written = 1;
	}

	r = 0;

	return r;
}

static void gb_xport_init(void)
{
}
static void gb_xport_exit(void)
{
}
static int gb_xport_listen(unsigned int cport)
{
	return 0;
}
static int gb_xport_stop_listening(unsigned int cport)
{
	return 0;
}
static int gb_xport_send(unsigned int cport, const void *buf, size_t len)
{
	int r;
    uint16_t *cportp;
	struct gb_operation_hdr *msg;
	struct gb_transport_tcpip_context *ctx;

	msg = (struct gb_operation_hdr *)buf;
    if (NULL == msg) {
		LOG_ERR("message is NULL");
	    return -EINVAL;
	}

    msg->pad[0] = cport;

    LOG_HEXDUMP_DBG(msg, sys_le16_to_cpu(msg->size), "TX:");

    if (sys_le16_to_cpu(msg->size) != len || len < sizeof(*msg)) {
		LOG_ERR("invalid message size %u (len: %u)",
			(unsigned)sys_le16_to_cpu(msg->size), (unsigned)len);
        return -EINVAL;
    }

	r = sendMessage(uart_dev, msg);

	return r;
}
static void *gb_xport_alloc_buf(size_t size)
{
	return malloc(size);
}
static void gb_xport_free_buf(void *ptr)
{
	free(ptr);
}

static const struct gb_transport_backend gb_xport = {
	.init = gb_xport_init,
	.exit = gb_xport_exit,
	.listen = gb_xport_listen,
	.stop_listening = gb_xport_stop_listening,
	.send = gb_xport_send,
	.send_async = NULL,
	.alloc_buf = gb_xport_alloc_buf,
	.free_buf = gb_xport_free_buf,
};

static void gb_xport_uart_isr(const struct device *dev, void *user_data)
{
	int r;
	uint8_t byte;
	uint8_t ovflw;
	size_t count = 0;
	size_t nwritten;

	while (uart_irq_update(dev) &&
	       uart_irq_is_pending(dev)) {

		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		r = uart_fifo_read(dev, &byte, 1);
		if (r < 0) {
			LOG_ERR("uart_fifo_read() failed (%d)", r);
			uart_irq_rx_disable(dev);
			return;
		}

		if (0 != k_pipe_put(&gb_xport_uart_pipe, &byte, 1, &nwritten, 1, K_NO_WAIT)) {
			LOG_ERR("k_pipe_put() failed");
			uart_irq_rx_disable(dev);
			return;
		}
	}
}

static int gb_xport_uart_init(void)
{
	int r;
	uint8_t c;

	LOG_INF("binding %s", CONFIG_GREYBUS_XPORT_UART_DEV);
	uart_dev = device_get_binding(CONFIG_GREYBUS_XPORT_UART_DEV);
    if (uart_dev == NULL) {
    	LOG_ERR("unable to bind device named %s!", CONFIG_GREYBUS_XPORT_UART_DEV);
    	r = -ENODEV;
    	goto out;
    }

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	uart_irq_callback_set(uart_dev, gb_xport_uart_isr);

	/* Drain the fifo */
	while (uart_irq_rx_ready(uart_dev)) {
		uart_fifo_read(uart_dev, &c, 1);
	}

	uart_irq_rx_enable(uart_dev);

	r = 0;

out:
	return r;
}

struct gb_transport_backend *gb_transport_backend_init(unsigned int *cports, size_t num_cports) {

    int r;
    struct gb_transport_backend *ret = NULL;

	LOG_DBG("Greybus UART Transport initializing..");

    if (num_cports >= CPORT_ID_MAX) {
        LOG_ERR("invalid number of cports %u", (unsigned)num_cports);
        goto out;
    }

    r = gb_xport_uart_init();
    if (r < 0) {
    	goto out;
    }

	r = pthread_create(&receive_thread, NULL, thread_fun, NULL);
	if (r != 0) {
		LOG_ERR("pthread_create: %d", r);
		goto out;
	}

    ret = (struct gb_transport_backend *)&gb_xport;

	LOG_INF("Greybus UART Transport initialized");
	goto out;

out:
    return ret;
}
