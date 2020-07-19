/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <greybus/greybus.h>
#include <string.h>
#include <sys/util.h>
#include <ztest.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_gpio_test, CONFIG_GB_LOG_LEVEL);

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

#define TIMEOUT_MS 1000
#define PORT 4243

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

    fd = socket(AF_INET6, SOCK_STREAM, 0);
    zassert_not_equal(fd, -1, "socket: %s", strerror(errno));

    r = connect(fd, (const struct sockaddr *)&addr, addrlen);
    zassert_not_equal(r, -1, "connect: %s", strerror(errno));

    size = sys_le16_to_cpu(req->size);
    r = write(fd, req, size);
    zassert_not_equal(r, -1, "write: %s", strerror(errno));
    zassert_equal(r, size, "expected: %d actual: %d", size, r);

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
    zassert_true(false, "untested");
}

void test_greybus_gpio_cport_shutdown(void)
{
    zassert_true(false, "untested");
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

    zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
        "expected: GB_OP_SUCCESS actual: %u",
        ((struct gb_operation_hdr *)rsp_)->result);

    tx_rx(&req, (struct gb_operation_hdr *)rsp_, rsp_size);

    /* default number of pins on a controller is 32 */
    zassert_equal(rsp->count + 1, 32, "expected: %u actual: %u", 32, rsp->count + 1);
}

void test_greybus_gpio_activate(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_deactivate(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_get_direction(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_direction_input(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_direction_output(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_get_value(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_set_value(void)
{
    zassert_true(false, "untested");
}

void test_greybus_gpio_set_debounce(void)
{
    zassert_true(false, "untested");
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
