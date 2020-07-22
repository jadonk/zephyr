/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <greybus/greybus.h>
#include <string.h>
#include <sys/util.h>
#include <ztest.h>
#include <zephyr.h>

#if defined(CONFIG_BOARD_NATIVE_POSIX_64BIT) \
    || defined(CONFIG_BOARD_NATIVE_POSIX_32BIT) \
    || defined(CONFIG_BOARD_NRF52_BSIM)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/byteorder.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#else

#include <net/socket.h>
#include <posix/unistd.h>

#undef read
#define read(fd, buf, len) zsock_recv(fd, buf, len, 0)

#undef write
#define write(fd, buf, len) zsock_send(fd, buf, len, 0)

#undef perror
#define perror(s) LOG_ERR("%s", s) 

#endif

#include "../../../../subsys/greybus/gpio-gb.h"

#include "test-greybus-gpio.h"

#define TIMEOUT_MS 1000
#define PORT 4243

static struct device *gpio_dev;

static void tx_rx(const struct gb_operation_hdr *req, struct gb_operation_hdr *rsp, size_t rsp_size)
{

    int r;
    struct sockaddr_in6 addr = {
        .sin6_family = AF_INET6,
        .sin6_addr = in6addr_loopback,
        .sin6_port = htons(PORT),
    };
    socklen_t addrlen = sizeof(addr);
    int size;
    int fd;
    struct pollfd pollfd;

    if (gpio_dev == NULL) {
        gpio_dev = device_get_binding(GPIO_DEV_NAME);
        zassert_not_equal(gpio_dev, NULL, "failed to get device binding for " GPIO_DEV_NAME);
    }

    fd = socket(AF_INET6, SOCK_STREAM, 0);
    zassert_not_equal(fd, -1, "socket: %s", strerror(errno));

    r = connect(fd, (const struct sockaddr *)&addr, addrlen);
    zassert_not_equal(r, -1, "connect: %s", strerror(errno));

    size = sys_le16_to_cpu(req->size);
    r = write(fd, req, size);
    zassert_not_equal(r, -1, "write: %s", strerror(errno));
    zassert_equal(r, size, "write: expected: %d actual: %d", size, r);

    pollfd.fd = fd;
    pollfd.events = POLLIN;

    r = poll(&pollfd, 1, TIMEOUT_MS);
    zassert_not_equal(r, -1, "poll: %s", strerror(errno));
    zassert_not_equal(r, 0, "timeout waiting for response");
    zassert_equal(r, 1, "invalid number of pollfds with data: %d", r);

    r = read(fd, rsp, rsp_size);
    zassert_not_equal(r, -1, "read: %s", strerror(errno));
    zassert_equal(r, rsp_size, "expected: %u actual: %d", rsp_size, r);

    zassert_equal(rsp->id, req->id, "expected: 0x%04x actual: 0x%04x", sys_le16_to_cpu(req->id), sys_le16_to_cpu(rsp->id));
    zassert_equal(rsp->type, GB_TYPE_RESPONSE_FLAG | req->type, "expected: %u actual: %u", GB_TYPE_RESPONSE_FLAG | req->type, rsp->type);

    close(fd);
}

void test_greybus_gpio_protocol_version(void) {
    const struct gb_operation_hdr req = {
        .size = sys_cpu_to_le16(sizeof(struct gb_operation_hdr)),
        .id = sys_cpu_to_le16(0xabcd),
        .type = GB_GPIO_TYPE_PROTOCOL_VERSION,
    };
    uint8_t rsp_[
        0
        + sizeof(struct gb_operation_hdr)
        + sizeof(struct gb_gpio_proto_version_response)
        ];
    const size_t rsp_size = sizeof(rsp_);
    struct gb_gpio_proto_version_response *const rsp =
        (struct gb_gpio_proto_version_response *)
        (rsp_ + sizeof(struct gb_operation_hdr));

    tx_rx(&req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);

    /* GB_GPIO_VERSION_MAJOR (0) is buried in subsys/greybus/gpio.c */
	zassert_equal(rsp->major, 0, "expected: %u actual: %u",
        0, rsp->major);

    /* GB_GPIO_VERSION_MINOR (1) is buried in subsys/greybus/gpio.c */
	zassert_equal(rsp->minor, 1, "expected: %u actual: %u",
        1, rsp->minor);
}

void test_greybus_gpio_cport_shutdown(void)
{
	/*
	 * looks like the original NuttX implementation was missing any kind
	 * of cport shutdown request
	 */
}

void test_greybus_gpio_line_count(void)
{
    const struct gb_operation_hdr req = {
        .size = sys_cpu_to_le16(sizeof(struct gb_operation_hdr)),
        .id = sys_cpu_to_le16(0xabcd),
        .type = GB_GPIO_TYPE_LINE_COUNT,
    };
    uint8_t rsp_[
        0
        + sizeof(struct gb_operation_hdr)
        + sizeof(struct gb_gpio_line_count_response)
        ];
    const size_t rsp_size = sizeof(rsp_);
    struct gb_gpio_line_count_response *const rsp =
        (struct gb_gpio_line_count_response *)
        (rsp_ + sizeof(struct gb_operation_hdr));
    size_t expected_count;
    size_t actual_count;

    tx_rx(&req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);

    zassert_not_equal(gpio_dev->config_info, NULL, "gpio_dev->config invalid");

    expected_count =
        popcount(((const struct gpio_driver_config *)gpio_dev->config_info)->port_pin_mask);
    actual_count = rsp->count + 1;

    zassert_equal(expected_count, actual_count, "expected: %u actual: %u",
        expected_count, actual_count);
}

void test_greybus_gpio_activate(void)
{
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_activate_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_activate_request *const activate =
		(struct gb_gpio_activate_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_ACTIVATE;
    activate->which = GPIO_PIN_IN;

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 ];
    size_t rsp_size = sizeof(rsp_);

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);
}

void test_greybus_gpio_deactivate(void)
{
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_activate_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_activate_request *const activate =
		(struct gb_gpio_activate_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_DEACTIVATE;
    activate->which = GPIO_PIN_IN;

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 ];
    size_t rsp_size = sizeof(rsp_);

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);
}

void test_greybus_gpio_get_direction(void)
{
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_get_direction_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_get_direction_request *const dir_req =
		(struct gb_gpio_get_direction_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_GET_DIRECTION;
    dir_req->which = GPIO_PIN_IN;

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_get_direction_response)
		 ];
    size_t rsp_size = sizeof(rsp_);
    struct gb_gpio_get_direction_response *const dir_rsp =
		(struct gb_gpio_get_direction_response *)
		(rsp_ + sizeof(struct gb_operation_hdr));

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);

    /* in greybus, 1 means input */
    zassert_equal(1, dir_rsp->direction, "expected: %u actual: %u", 1, dir_rsp->direction);
}

void test_greybus_gpio_direction_input(void)
{
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_direction_in_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_direction_in_request *const dir_req =
		(struct gb_gpio_direction_in_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_DIRECTION_IN;
    dir_req->which = GPIO_PIN_IN;

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 ];
    size_t rsp_size = sizeof(rsp_);

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);
}

void test_greybus_gpio_direction_output(void)
{
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_direction_out_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_direction_out_request *const dir_req =
		(struct gb_gpio_direction_out_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_DIRECTION_OUT;
    dir_req->which = GPIO_PIN_OUT;

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 ];
    size_t rsp_size = sizeof(rsp_);

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(GB_OP_SUCCESS, ((struct gb_operation_hdr *)rsp_)->result,
        "expected: %u actual: %u", GB_OP_SUCCESS,
        ((struct gb_operation_hdr *)rsp_)->result);
}

void test_greybus_gpio_get_value(void)
{
	int r;
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_get_value_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_get_value_request *const get_req =
		(struct gb_gpio_get_value_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_GET_VALUE;
    get_req->which = GPIO_PIN_IN;

    /*
     * set PIN_OUT (1), since it is an output and is connected to PIN_IN, which
     * we will subsequently read
     */
    if (gpio_dev == NULL) {
        gpio_dev = device_get_binding(GPIO_DEV_NAME);
        zassert_not_equal(gpio_dev, NULL, "failed to get device binding for " GPIO_DEV_NAME);
    }
    r = gpio_pin_set(gpio_dev, GPIO_PIN_OUT, 1);
    zassert_equal(0, r, "gpio_pin_set() failed: %d", r);

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_get_value_response)
		 ];
    size_t rsp_size = sizeof(rsp_);
    struct gb_gpio_get_value_response *const get_rsp =
		(struct gb_gpio_get_value_response *)
		(rsp_ + sizeof(struct gb_operation_hdr));

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(GB_OP_SUCCESS, ((struct gb_operation_hdr *)rsp_)->result,
        "expected: %u actual: %u", GB_OP_SUCCESS,
        ((struct gb_operation_hdr *)rsp_)->result);

    /*
     * PIN_IN should be set (1), since it is an input and is
     * connected to PIN_OUT
     */
    zassert_equal(1, get_rsp->value, "expected: %u actual: %u", 1, get_rsp->value);
}

void test_greybus_gpio_set_value(void)
{
	int r;
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_set_value_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_set_value_request *const set_req =
		(struct gb_gpio_set_value_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_SET_VALUE;
    /* pin 1 is configured as output */
    set_req->which = GPIO_PIN_OUT;
    set_req->value = 1;

    /*
     * set PIN_OUT (1), since it is an output and is connected to PIN_IN, which
     * we will subsequently read
     */
    if (gpio_dev == NULL) {
        gpio_dev = device_get_binding(GPIO_DEV_NAME);
        zassert_not_equal(gpio_dev, NULL, "failed to get device binding for " GPIO_DEV_NAME);
    }

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 ];
    size_t rsp_size = sizeof(rsp_);

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(GB_OP_SUCCESS, ((struct gb_operation_hdr *)rsp_)->result,
        "expected: %u actual: %u", GB_OP_SUCCESS,
        ((struct gb_operation_hdr *)rsp_)->result);

    /*
     * PIN_IN should be set (1), since it is an input and is
     * connected to PIN_OUT
     */
    r = gpio_pin_get(gpio_dev, GPIO_PIN_IN);
    zassert_true(r >= 0, "gpio_pin_get() failed: %d", r);
    zassert_equal(1, r, "expected: 1 actual: %d", 1, r);
}

void test_greybus_gpio_set_debounce(void)
{
	uint8_t req_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 + sizeof(struct gb_gpio_set_debounce_request)
		 ] = {};
    struct gb_operation_hdr *const req = (struct gb_operation_hdr *)req_;
    struct gb_gpio_set_debounce_request *const dir_req =
		(struct gb_gpio_set_debounce_request *)
		(req_ + sizeof(struct gb_operation_hdr));

    req->size = sys_cpu_to_le16(sizeof(req_));
    req->id = sys_cpu_to_le16(0xabcd);
    req->type = GB_GPIO_TYPE_SET_DEBOUNCE;
    dir_req->which = GPIO_PIN_IN;
    dir_req->usec = 11;

    uint8_t rsp_[
		 0
		 + sizeof(struct gb_operation_hdr)
		 ];
    size_t rsp_size = sizeof(rsp_);

    tx_rx(req, (struct gb_operation_hdr *)rsp_, rsp_size);

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);
}

void test_greybus_gpio_irq_type(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_irq_mask(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_irq_unmask(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_irq_event(void)
{
    zassert_true(false, "untested");
}
