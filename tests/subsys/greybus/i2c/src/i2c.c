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

#if defined(CONFIG_BOARD_NATIVE_POSIX_64BIT) ||                                \
	defined(CONFIG_BOARD_NATIVE_POSIX_32BIT) ||                            \
	defined(CONFIG_BOARD_NRF52_BSIM)

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

#endif

#include "../../../../subsys/greybus/i2c-gb.h"

#include "test-greybus-i2c.h"

#define TIMEOUT_MS (-1)
#define PORT 4243

static struct device *i2c_dev;

static int fd = -1;

static char *to_string(uint8_t *data, size_t len)
{
    static char buf[256];
    char *p = buf;

    memset(buf, '\0', sizeof(buf));
    for(size_t i = 0; i < len && p < buf + sizeof(buf) - 4; ++i) {
        sprintf(p, "%02x", data[i]);
        p += 2;
        if (i < len - 1) {
            *p++ = ',';
            *p++ = ' ';
        }
    }

    return buf;
}

void test_greybus_setup(void)
{
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_addr = in6addr_loopback,
		.sin6_port = htons(PORT),
	};
	socklen_t addrlen = sizeof(addr);
	int r;

	i2c_dev = device_get_binding(I2C_DEV_NAME);
	zassert_not_equal(i2c_dev, NULL,
			  "failed to get device binding for " I2C_DEV_NAME);

	r = socket(AF_INET6, SOCK_STREAM, 0);
	__ASSERT(r >= 0, "connect: %d", errno);
	fd = r;

	r = connect(fd, (const struct sockaddr *)&addr, addrlen);
	__ASSERT(r == 0, "connect: %d", errno);
}

void test_greybus_teardown(void)
{
	if (fd != -1) {
		close(fd);
		fd = -1;
	}
}

static void tx_rx(const struct gb_operation_hdr *req,
		  struct gb_operation_hdr *rsp, size_t rsp_size)
{
	int r;
	int size;
	struct pollfd pollfd;

	size = sys_le16_to_cpu(req->size);
	r = send(fd, req, size, 0);
	zassert_not_equal(r, -1, "send: %s", errno);
	zassert_equal(r, size, "send: expected: %d actual: %d", size, r);

	pollfd.fd = fd;
	pollfd.events = POLLIN;

	r = poll(&pollfd, 1, TIMEOUT_MS);
	zassert_not_equal(r, -1, "poll: %s", errno);
	zassert_not_equal(r, 0, "timeout waiting for response");
	zassert_equal(r, 1, "invalid number of pollfds with data: %d", r);

	r = recv(fd, rsp, rsp_size, 0);
	zassert_not_equal(r, -1, "recv: %s", errno);
	if (r != rsp_size) {
		printk("rsp: [%s]", to_string((uint8_t *)rsp, r));
	}
	zassert_equal(rsp_size, r, "recv: expected: %u actual: %u",
		      (unsigned)rsp_size, r);

	zassert_equal(rsp->id, req->id,
			  "expected: 0x%04x actual: 0x%04x",
			  sys_le16_to_cpu(req->id),
			  sys_le16_to_cpu(rsp->id));
	zassert_equal(rsp->type, GB_TYPE_RESPONSE_FLAG | req->type,
			  "expected: %u actual: %u",
			  GB_TYPE_RESPONSE_FLAG | req->type, rsp->type);
}

void test_greybus_i2c_protocol_version(void)
{
	const struct gb_operation_hdr req = {
		.size = sys_cpu_to_le16(sizeof(struct gb_operation_hdr)),
		.id = sys_cpu_to_le16(0xabcd),
		.type = GB_I2C_PROTOCOL_VERSION,
	};
	uint8_t rsp_[0 + sizeof(struct gb_operation_hdr) +
		     sizeof(struct gb_i2c_proto_version_response)];
	const size_t rsp_size = sizeof(rsp_);
	struct gb_operation_hdr *const rsp =
		(struct gb_operation_hdr *)rsp_;
	struct gb_i2c_proto_version_response *const pv_rsp =
		(struct gb_i2c_proto_version_response
			 *)(rsp_ + sizeof(struct gb_operation_hdr));

	/* add some "noise" values that should be overwritten */
	pv_rsp->major = 0xf0;
	pv_rsp->minor = 0x0d;

	tx_rx(&req, rsp, rsp_size);

	zassert_equal(rsp->result, GB_OP_SUCCESS,
		      "expected: GB_OP_SUCCESS actual: %u",
		      rsp->result);

	/* GB_I2C_VERSION_MAJOR (0) is buried in subsys/greybus/i2c.c */
	zassert_equal(pv_rsp->major, 0, "expected: %u actual: %u", 0, pv_rsp->major);

	/* GB_I2C_VERSION_MINOR (1) is buried in subsys/greybus/i2c.c */
	zassert_equal(pv_rsp->minor, 1, "expected: %u actual: %u", 1, pv_rsp->minor);
}

void test_greybus_i2c_cport_shutdown(void)
{
	/*
	 * looks like the original NuttX implementation was missing any kind
	 * of cport shutdown request
	 */
}

void test_greybus_i2c_functionality(void)
{
	const struct gb_operation_hdr req = {
		.size = sys_cpu_to_le16(sizeof(struct gb_operation_hdr)),
		.id = sys_cpu_to_le16(0xabcd),
		.type = GB_I2C_PROTOCOL_FUNCTIONALITY,
	};
	uint8_t rsp_[0 + sizeof(struct gb_operation_hdr) +
		     sizeof(struct gb_i2c_functionality_rsp)];
	const size_t rsp_size = sizeof(rsp_);
	struct gb_i2c_functionality_rsp *const rsp =
		(struct gb_i2c_functionality_rsp
			 *)(rsp_ + sizeof(struct gb_operation_hdr));
	uint32_t expected_uint32;
	uint32_t actual_uint32;

	tx_rx(&req, (struct gb_operation_hdr *)rsp_, rsp_size);

	zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
		      "expected: GB_OP_SUCCESS actual: %u",
		      ((struct gb_operation_hdr *)rsp_)->result);

	expected_uint32 = GB_I2C_FUNC_I2C | GB_I2C_FUNC_SMBUS_READ_BYTE |
			  GB_I2C_FUNC_SMBUS_WRITE_BYTE |
			  GB_I2C_FUNC_SMBUS_READ_BYTE_DATA |
			  GB_I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
			  GB_I2C_FUNC_SMBUS_READ_WORD_DATA |
			  GB_I2C_FUNC_SMBUS_WRITE_WORD_DATA |
			  GB_I2C_FUNC_SMBUS_READ_I2C_BLOCK |
			  GB_I2C_FUNC_SMBUS_WRITE_I2C_BLOCK;

	actual_uint32 = sys_le32_to_cpu(rsp->functionality);

	zassert_equal(expected_uint32, actual_uint32,
		      "functionality: expected: %08x actual: %08x",
		      expected_uint32, actual_uint32);
}

void test_greybus_i2c_transfer(void)
{
	/* this test simulates some operations on an hmc5883l 3-axis accelerometer */

	/*
	 * D: i2c_sim_hmc_callback(): W: addr: 001e: flags: 02 len: 2 buf: [01, 20]
	 * D: i2c_sim_hmc_callback(): W: addr: 001e: flags: 02 len: 2 buf: [02, 00]
	 * D: i2c_sim_hmc_callback(): W: addr: 001e: flags: 02 len: 1 buf: [03]
	 * D: i2c_sim_hmc_callback(): R: addr: 001e: flags: 03 len: 6 buf: [00, 00, 00, 00, 00, 00]
	 */ 

#define DESC(_addr, _flags, _size) \
	{ \
		.addr = sys_cpu_to_le16(_addr), \
		.flags =  sys_cpu_to_le16(_flags), \
		.size =  sys_cpu_to_le16(_size), \
	}

	const uint8_t write_data[] = {
		0x01, 0x20,
		0x02, 0x00,
		0x03,
	};

	const struct gb_i2c_transfer_desc descs[] = {
		DESC(0x1e, 0, 2),
		DESC(0x1e, 0, 2),
		DESC(0x1e, 0, 1),
		DESC(0x1e, GB_I2C_M_RD, 6),	
	};

	uint8_t req_[
		0
		+ sizeof(struct gb_operation_hdr)
		+ sizeof(struct gb_i2c_transfer_req)
		+ ARRAY_SIZE(descs) * sizeof(struct gb_i2c_transfer_desc)
		/* Would be nice if constexpr existed in C ... oh well */
		+ (2 + 2 + 1 /* request size only counts write ops */)
		] = {};
	const size_t req_size = sizeof(req_);

	struct gb_operation_hdr *const req =
		(struct gb_operation_hdr *)req_;
	
	req->size = sys_cpu_to_le16(req_size);
	req->id = sys_cpu_to_le16(0xabcd);
	req->type = GB_I2C_PROTOCOL_TRANSFER;

	struct gb_i2c_transfer_req *const xfer_req =
		(struct gb_i2c_transfer_req *)
		(req_ + sizeof(*req));
	xfer_req->op_count = sys_cpu_to_le16(ARRAY_SIZE(descs));

	uint8_t rsp_[
		0
		+ sizeof(struct gb_operation_hdr)
		/* Would be nice if constexpr existed in C ... oh well */
		+ (6 /* response size only counts read ops */)
		];
	const size_t rsp_size = sizeof(rsp_);
	struct gb_operation_hdr *const rsp =
		(struct gb_operation_hdr *)rsp_;

	struct gb_i2c_transfer_rsp *const xfer_rsp =
		(struct gb_i2c_transfer_rsp *)
		(rsp_ + sizeof(struct gb_operation_hdr));

	/* the assertion is that the rsp is *not* equal to this */
	const uint8_t expected_data[] = {
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab,
	};
	uint8_t *const actual_data = xfer_rsp->data;

	memcpy(xfer_rsp->data, expected_data, sizeof(expected_data));

	uint8_t *payload = (uint8_t *)xfer_req->desc;
	memcpy(payload, descs, sizeof(descs));
	payload += sizeof(descs);
	memcpy(payload, write_data, sizeof(write_data)); 

	tx_rx(req, rsp, rsp_size);

	zassert_equal(((struct gb_operation_hdr *)rsp_)->result, GB_OP_SUCCESS,
		      "expected: GB_OP_SUCCESS actual: %u",
		      ((struct gb_operation_hdr *)rsp_)->result);

	zassert_not_equal(0, memcmp(expected_data, actual_data, sizeof(expected_data)),
		"expected response to generate %u bytes ofdata", sizeof(expected_data));
}
