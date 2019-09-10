/*
 * Copyright (c) 2109 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <console/console.h>
#include <device.h>
#include <drivers/gpio.h>
#include <errno.h>
#include <greybus/debug.h>
#include <greybus/greybus.h>
#include <greybus-utils/manifest.h>
#include <greybus-utils/platform.h>
#include <posix/unistd.h>
//#include <syscalls/uart.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#ifdef CONFIG_GREYBUS_STATIC_MANIFEST
#include <greybus/static-manifest.h>
#endif

enum {
	GB_TYPE_MANIFEST_SET = 0x42,
	GB_TYPE_ANY = (uint8_t)-1,
};

static int getMessage( struct gb_operation_hdr **msg, const u8_t expected_msg_type) {

	int r;
	void *tmp;
	size_t msg_size;
	size_t payload_size;
	size_t remaining;
	size_t offset;
	size_t recvd;

	if (NULL == msg) {
		r = -EINVAL;
		goto out;
	}

	tmp = realloc(*msg, sizeof(**msg));
	if (NULL == tmp) {
		r = -ENOMEM;
		goto out;
	}
	*msg = tmp;

	for( remaining = sizeof(**msg), offset = 0; remaining; remaining -= recvd, offset += recvd) {
		r = console_read(NULL, *msg, sizeof(**msg));
		if (r <= 0) {
			goto freemsg;
		}
		recvd = r;
	}

	msg_size = sys_le16_to_cpu((*msg)->size);
	payload_size = msg_size - sizeof(**msg);

	if (payload_size > 0) {
		tmp = realloc(*msg, msg_size);
		if (NULL == tmp) {
			r = -ENOMEM;
			goto freemsg;
		}
		*msg = tmp;

		for( remaining = payload_size, offset = sizeof(**msg); remaining; remaining -= recvd, offset += recvd) {
			r = console_read(NULL, &((uint8_t *)*msg)[offset], remaining);
			if (r <= 0) {
				goto freemsg;
			}
			recvd = r;
		}
	}

	if (!(GB_TYPE_ANY == expected_msg_type || expected_msg_type == (*msg)->type)) {
		r = -EPROTO;
		goto freemsg;
	}

	r = 0;
	goto out;

freemsg:
	free(*msg);
	*msg = NULL;

out:
	return r;
}

static int sendMessage(struct gb_operation_hdr *msg) {

	int r;
	size_t offset;
	size_t remaining;
	size_t written;

	if (NULL == msg) {
		r = -EINVAL;
		goto out;
	}

	for(remaining = sys_le16_to_cpu(msg->size), offset = 0; remaining; remaining -= written, offset += written) {
		r = console_write(NULL, &((uint8_t *)msg)[offset], remaining);
		if (r < 0) {
			goto out;
		}
		written = r;
	}

	r = 0;

out:
	return r;
}

static void assert( bool val ) {
	for(;!val;);
}

unsigned int unipro_cport_count(void) {
	// limits us to 1 control port and 1 device
	return 2;
}

static void gb_xport_init(void) {}
static void gb_xport_exit(void) {}
static int gb_xport_listen(unsigned int cport) { return 0; }
static int gb_xport_stop_listening(unsigned int cport) { return 0; }
static int gb_xport_send(unsigned int cport, const void *buf, size_t len) {
	int r;
	struct gb_operation_hdr *msg;
	msg = (struct gb_operation_hdr *)buf;
	msg->pad[0] = cport & 0xff;
	msg->pad[1] = (cport >> 8) & 0xff;
	r = console_write(NULL, buf, len);
	if (r < 0) {
		return r;
	}
	if (r != len) {
		return -EIO;
	}
	return 0;
}
static void *gb_xport_alloc_buf(size_t size) {
	return calloc(1,size);
}
static void gb_xport_free_buf(void *ptr) {
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

static void register_gb_platform_drivers() {
	extern const struct gb_gpio_platform_driver gb_gpio_cc13x2r;
	gb_gpio_register_platform_driver((struct gb_gpio_platform_driver *)&gb_gpio_cc13x2r);
}

static void blink(struct device *dev, unsigned pin, size_t count, size_t delay) {
	u32_t state;
	gpio_pin_read(dev, pin, &state);
	for(size_t i = 0; i < count; i++) {
		gpio_pin_write(dev, pin, !state);
		usleep(delay);
		gpio_pin_write(dev, pin, state);
		usleep(delay);
	}
}

// TODO: First, we should upload the private key, then upload authorized keys, then the manifest, and then hand it off to Greybus
void main(void)
{
	int r;
	void *manifest;
	size_t manifest_size;
	struct gb_operation_hdr *msg = NULL;
	u16_t msg_size;
	struct gb_operation_hdr resp;

	struct device *red_led_dev;
	const unsigned red_led_pin = DT_ALIAS_LED1_GPIOS_PIN;
	red_led_dev = device_get_binding(DT_ALIAS_LED1_GPIOS_CONTROLLER);
	assert(NULL != red_led_dev);
	gpio_pin_configure(red_led_dev, red_led_pin, GPIO_DIR_OUT);

	struct device *green_led_dev;
	const unsigned green_led_pin = DT_ALIAS_LED0_GPIOS_PIN;
	green_led_dev = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	assert(NULL != green_led_dev);
	gpio_pin_configure(green_led_dev, green_led_pin, GPIO_DIR_IN);

	register_gb_platform_drivers();

	console_init();

	r = -EIO;

start_over:
#ifdef CONFIG_GREYBUS_STATIC_MANIFEST

	// blink an led twice to indicate that the manifest was received
	blink(red_led_dev, red_led_pin, 2, 100000);

	manifest = get_manifest_blob();
	manifest_size = (size_t)manifest_mnfb_len;
	r = manifest_parse(manifest, manifest_size);
	if (true != r) {
		gb_error("failed to parse manifest\n");
		resp.result = gb_errno_to_op_result(-EINVAL);
		sendMessage(&resp);
		goto start_over;
	}
#else
	// blink an led once to indicate that the manifest is expected
	blink(red_led_dev, red_led_pin, 1, 100000);

	r = getMessage(&msg, GB_TYPE_MANIFEST_SET);
	if (r < 0) {
		goto start_over;
	}

	resp = *msg;
	resp.type |= GB_TYPE_RESPONSE_FLAG;
	resp.size = sys_cpu_to_le16(sizeof(resp));

	msg_size = sys_le16_to_cpu(msg->size);
	manifest_size = msg_size - sizeof(*msg);

	memmove(msg, (uint8_t *)msg + sizeof(*msg), manifest_size);
	manifest = (uint8_t *)msg;
	memset((uint8_t *)msg + manifest_size, 0, sizeof(*msg));
	msg = NULL; // give memory reference to manifest

	// blink an led twice to indicate that the manifest was received
	blink(red_led_dev, red_led_pin, 2, 100000);

	r = manifest_parse(manifest, manifest_size);
	if (true != r) {
		gb_error("failed to parse manifest\n");
		resp.result = gb_errno_to_op_result(-EINVAL);
		sendMessage(&resp);
		goto start_over;
	}

	set_manifest_blob(manifest);
#endif

	r = gb_init((struct gb_transport_backend *)&gb_xport);
	if (0 != r) {
		gb_error("gb_init() failed (%d)\n", r);
		resp.result = gb_errno_to_op_result(r);
		sendMessage(&resp);
		goto start_over;
	}

	enable_cports();

#ifndef CONFIG_GREYBUS_STATIC_MANIFEST
	resp.result = GB_OP_SUCCESS;
	sendMessage(&resp);
#endif

	// Turn on the red LED to indicate the device is 'online'
	gpio_pin_write(red_led_dev, red_led_pin, 1);

	msg = NULL;
	for( size_t i = 0;; i++) {
		// get messages and pass them to greybus
		r = getMessage(&msg, GB_TYPE_ANY);
		if (0 != r) {
			gb_error("failed to receive message (%d)\n", r);
			continue;
		}

		unsigned int cport = ((uint16_t)msg->pad[1] << 8) | msg->pad[0];
		//blink(red_led_dev, red_led_pin, 1, 10000);

		r = greybus_rx_handler(cport, msg, sys_le16_to_cpu(msg->size));
		if (0 != r) {
			gb_error("failed to handle message %u: size: %u, id: %u, type: %u\n", i, sys_le16_to_cpu(msg->size), sys_le16_to_cpu(msg->id), msg->type);
		}
	}
}
