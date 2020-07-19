/*
 * Copyright (c) 2109 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DEBUG

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __ZEPHYR__

#include <zephyr.h>
#include <posix/unistd.h>
#include <posix/pthread.h>

#include <greybus/debug.h>
#include <greybus/greybus.h>
#include <greybus/platform.h>
#include <greybus-utils/manifest.h>

#include <drivers/gpio.h>

#ifdef CONFIG_GREYBUS_STATIC_MANIFEST
#include <greybus/static-manifest.h>
#endif

#define STACK_SIZE 512

static pthread_attr_t control_thread_attr;
static K_THREAD_STACK_DEFINE(control_thread_stack, STACK_SIZE);

static pthread_attr_t gpio_thread_attr;
static K_THREAD_STACK_DEFINE(gpio_thread_stack, STACK_SIZE);

unsigned int sleep(unsigned int seconds)
{
	k_sleep(K_MSEC(1000 * seconds));
	return 0;
}

int usleep(useconds_t usec) {
	k_usleep(usec);
	return 0;
}

#ifdef DEBUG
void perror(const char *s)
{
	printf("%s: %s\n", s, strerror(errno));
}
#endif

#else

#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#endif

#ifdef DEBUG
#define D(fmt, args...) printf("%s(): %d: " fmt "\n", __func__, __LINE__, ##args)
#else
#define D(fmt, args...)
#endif

enum { GB_TYPE_MANIFEST_SET = 0x42,
       GB_TYPE_ANY = (uint8_t)-1,
};

static int getMessage(int fd, struct gb_operation_hdr **msg,
		      const uint8_t expected_msg_type);

static int sendMessage(int fd, struct gb_operation_hdr *msg);

static int control_server_fd = -1;
static int control_client_fd = -1;
static pthread_t control_thread;
pthread_attr_t *control_thread_attr_p;

static int gpio_server_fd;
static int gpio_client_fd;
static pthread_t gpio_thread;
pthread_attr_t *gpio_thread_attr_p;

static void *thread_fun(void *arg)
{
	int r;
	uint8_t cport;
	int i = 0;
	struct gb_operation_hdr *msg = NULL;
	int *fd = (int *)arg;
	char *clientstr;

	if (false) {
	} else if (control_client_fd == *fd) {
		// control traffic
		clientstr = "control";
		cport = 0;
	} else if (gpio_client_fd == *fd) {
		// gpio traffic
		clientstr = "gpio";
		cport = 1;
	} else {
		D("invalid fd %d", *fd);
		return NULL;
	}

	for (;;) {
		r = getMessage(*fd, &msg, GB_TYPE_ANY);
		if (0 == r) {
			D("%s: getMessage() returned EOF", clientstr);
			break;
		}
		if (r < 0) {
			D("%s: failed to receive message (%d)", clientstr, r);
			break;
		}

		//D("%s: Received message", clientstr);

		r = greybus_rx_handler(cport, msg, sys_le16_to_cpu(msg->size));
		if ( 0 == r ) {
			//D("%s: Handled message!", clientstr);
		} else {
			D("%s: failed to handle message %u: size: %u, id: %u, type: %u", clientstr, i, sys_le16_to_cpu(msg->size), sys_le16_to_cpu(msg->id), msg->type);
		}
	}

	D("%s: closing fd %d", clientstr, *fd);
	close(*fd);
	*fd = -1;
	return NULL;
}

static int netsetup(void)
{
	int r;
	const int yes = true;
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_addr = in6addr_any,
	};

#ifdef __ZEPHYR__
	D("initializing control_thread stack");
	r = pthread_attr_init(&control_thread_attr);
	if (0 != r) {
		D("pthred_attr_init() failed: %d", r);
		return r;
	}
	r = pthread_attr_setstack(&control_thread_attr, &control_thread_stack,
				  STACK_SIZE);
	if (0 != r) {
		D("pthred_attr_setstack() failed: %d", r);
		return r;
	}

	D("initializing gpio_thread stack");
	r = pthread_attr_init(&gpio_thread_attr);
	if (0 != r) {
		D("pthred_attr_init() failed: %d", r);
		return r;
	}
	r = pthread_attr_setstack(&gpio_thread_attr, &gpio_thread_stack,
				  STACK_SIZE);
	if (0 != r) {
		D("pthred_attr_setstack() failed: %d", r);
		return r;
	}

	control_thread_attr_p = &control_thread_attr;
	gpio_thread_attr_p = &gpio_thread_attr;
#endif

	D("creating control server socket");
	control_server_fd = socket(AF_INET6, SOCK_STREAM, 0);
	if (-1 == control_server_fd) {
		perror("socket");
		return -errno;
	}
	D("creating gpio server socket");
	gpio_server_fd = socket(AF_INET6, SOCK_STREAM, 0);
	if (-1 == gpio_server_fd) {
		perror("socket");
		return -errno;
	}

	D("setting socket options for control server");
	r = setsockopt(control_server_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
		       sizeof(yes));
	if (-1 == r) {
		perror("setsockopt");
		return -errno;
	}

	D("setting socket options for gpio server");
	r = setsockopt(gpio_server_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
		       sizeof(yes));
	if (-1 == r) {
		perror("setsockopt");
		return -errno;
	}

	D("binding control server socket");
	addr.sin6_port = htons(4242);
	r = bind(control_server_fd, (struct sockaddr *)&addr, sizeof(addr));
	if (-1 == r) {
		perror("bind");
		return -errno;
	}
	D("binding gpio server socket");
	addr.sin6_port = htons(4243);
	r = bind(gpio_server_fd, (struct sockaddr *)&addr, sizeof(addr));
	if (-1 == r) {
		perror("bind");
		return -errno;
	}

	D("listening on control server socket");
	r = listen(control_server_fd, 1);
	if (-1 == r) {
		perror("listen");
		return -errno;
	}
	D("listening on gpio server socket");
	listen(gpio_server_fd, 1);
	if (-1 == r) {
		perror("listen");
		return -errno;
	}

	return 0;
}

int accept_loop(void)
{
	int r;
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_addr = in6addr_any,
	};
	struct pollfd pollfds[2];
	socklen_t addrlen;
	static char addrstr[INET6_ADDRSTRLEN];
	char *addrstrp;

	for (;;) {
		D("preparing pollfds");
		memset(pollfds, 0, sizeof(pollfds));
		pollfds[0].fd = control_server_fd;
		pollfds[0].events = POLLIN;
		pollfds[1].fd = gpio_server_fd;
		pollfds[1].events = POLLIN;

		D("calling poll");
		r = poll(pollfds, ARRAY_SIZE(pollfds), -1);
		if (-1 == r) {
			perror("poll");
			return -errno;
		}
		D("returned from poll");
		if (pollfds[0].revents & POLLIN) {
			D("control socket has a traffic");
			addrlen = sizeof(addr);
			control_client_fd =
				accept(control_server_fd,
				       (struct sockaddr *)&addr, &addrlen);
			if (-1 == control_client_fd) {
				perror("accept");
				return -errno;
			}

			memset(addrstr, '\0', sizeof(addrstr));
			addrstrp = inet_ntop(AF_INET6, &addr.sin6_addr, addrstr, sizeof(addrstr));
			if (NULL == addrstrp) {
				perror("inet_ntop");
				return -errno;
			}
			D("accepted connection from [%s]:%d as fd %d", addrstr, ntohs(addr.sin6_port), control_client_fd);

			D("spawning control thread..");
			r = pthread_create(&control_thread,
					   control_thread_attr_p, thread_fun,
					   &control_client_fd);
			if (0 != r) {
				perror("pthread_create");
				return r;
			}
		}
		if (pollfds[1].revents & POLLIN) {
			D("gpio service has traffic");
			addrlen = sizeof(addr);
			gpio_client_fd =
				accept(gpio_server_fd, (struct sockaddr *)&addr,
				       &addrlen);
			if (-1 == gpio_client_fd) {
				perror("accept");
				return -errno;
			}

			memset(addrstr, '\0', sizeof(addrstr));
			addrstrp = inet_ntop(AF_INET6, &addr.sin6_addr, addrstr, sizeof(addrstr));
			if (NULL == addrstrp) {
				perror("inet_ntop");
				return -errno;
			}
			D("accepted connection from [%s]:%d as fd %d", addrstr, ntohs(addr.sin6_port), gpio_client_fd);

			D("spawning gpio thread..");
			pthread_create(&gpio_thread, gpio_thread_attr_p,
				       thread_fun, &gpio_client_fd);
			if (0 != r) {
				perror("pthread_create");
				return -errno;
			}
		}
	}

	D("out of loop");
	return 0;
}

static int getMessage(int fd, struct gb_operation_hdr **msg,
		      const uint8_t expected_msg_type)
{
	int r;
	void *tmp;
	size_t msg_size;
	size_t payload_size;
	size_t remaining;
	size_t offset;
	size_t recvd;

	if (NULL == msg) {
		D("One or more arguments were NULL or invalid");
		r = -EINVAL;
		goto out;
	}

	tmp = realloc(*msg, sizeof(**msg));
	if (NULL == tmp) {
		D("Failed to allocate memory");
		r = -ENOMEM;
		goto out;
	}
	*msg = tmp;

read_header:
	for (remaining = sizeof(**msg), offset = 0, recvd = 0; remaining;
	     remaining -= recvd, offset += recvd, recvd = 0) {
		//D("attempting to read %u bytes", remaining);
		r = recv(fd, &((uint8_t *)*msg)[offset], remaining, 0);
		//D("recv(%d, &%p[%u], %u, 0) => %d", fd, *msg, offset, remaining, r, errno);
		if (r < 0) {
			D("recv failed. errno: %d (%s)", errno, strerror(errno));
			usleep(100);
			continue;
		}
		if (0 == r) {
			D("recv returned 0 - EOF??");
			goto freemsg;
		}
		recvd = r;
	}

	msg_size = sys_le16_to_cpu((*msg)->size);
	//D("msg_size is %u", (unsigned)msg_size);

	if (msg_size < sizeof(struct gb_operation_hdr)) {
		D("invalid message size %u", (unsigned)msg_size);
		goto read_header;
	}

	payload_size = msg_size - sizeof(**msg);
	//D("payload_size is %u", (unsigned)payload_size);

	if (payload_size > GB_MAX_PAYLOAD_SIZE) {
		D("invalid payload size %u", (unsigned)payload_size);
		goto read_header;
	}

	if (payload_size > 0) {
		tmp = realloc(*msg, msg_size);
		if (NULL == tmp) {
			D("Failed to allocate memory");
			r = -ENOMEM;
			goto freemsg;
		}
		*msg = tmp;

		for (remaining = payload_size, offset = sizeof(**msg),
		    recvd = 0;
		     remaining;
		     remaining -= recvd, offset += recvd, recvd = 0) {
			//D("attempting to read %u bytes", remaining);
			r = recv(fd, &((uint8_t *)*msg)[offset], remaining, 0);
			//D("recv(%d, &%p[%u], %u) => %d", fd, *msg, offset, remaining, r);
			if (r < 0) {
				D("tty_read failed (%d)", r);
				usleep(100);
				continue;
			}
			if (0 == r) {
				D("tty_read returned 0 - EOF??");
				goto freemsg;
			}
			recvd = r;
		}
	}

	//	uint8_t *d = (uint8_t *)*msg;
	//	printk("GB: I: %s() : ", __func__);
	//	for( size_t i = 0; i < msg_size; i++ ) {
	//		printk("%02x ", d[i]);
	//	}
	//	printk("");

	if (!(GB_TYPE_ANY == expected_msg_type ||
	      expected_msg_type == (*msg)->type)) {
		D("Expected message type %d but received messsage type %d",
		  expected_msg_type, (*msg)->type);
		r = -EPROTO;
		goto freemsg;
	}

	r = msg_size;
	goto out;

freemsg:
	free(*msg);
	*msg = NULL;

out:
	return r;
}

static int sendMessage(int fd, struct gb_operation_hdr *msg)
{
	int r;
	size_t offset;
	size_t remaining;
	size_t written;
	char *clientstr;

	if (false) {
	} else if (control_client_fd == fd) {
		// control traffic
		clientstr = "control";
	} else if (gpio_client_fd == fd) {
		// gpio traffic
		clientstr = "gpio";
	} else {
		D("unmapped fd %d", fd);
		return -EINVAL;
	}

	if (NULL == msg) {
		D("%s: One or more arguments were NULL or invalid", clientstr);
		r = -EINVAL;
		goto out;
	}

	for (remaining = sys_le16_to_cpu(msg->size), offset = 0, written = 0;
	     remaining; remaining -= written, offset += written, written = 0) {
		r = send(fd, &((uint8_t *)msg)[offset], remaining, 0);
		//D("%s: send(%d, &%p[%u], %u, 0) => %d", clientstr, fd, msg, offset, remaining, r);
		if (r < 0) {
			D("%s: send failed. errno: %d (%s)", clientstr, errno, strerror(errno));
			return r;
		}
		if (0 == r) {
			D("%s: send returned 0 - EOF??", clientstr);
			return -1;
		}
		written = r;
	}

	//	uint8_t *d = (uint8_t *)msg;
	//	printk("GB: I: %s(): ", __func__);
	//	for( size_t i = 0; i < sys_le16_to_cpu(msg->size); i++ ) {
	//		printk("%02x ", d[i]);
	//	}
	//	printk("");

	r = 0;

out:
	return r;
}

unsigned int unipro_cport_count(void)
{
	// limits us to 1 control port and 1 device
	return 2;
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
	int fd = -1;

	struct gb_operation_hdr *msg;
	msg = (struct gb_operation_hdr *)buf;
	char *clientstr;

	if (false) {
	} else if (0 == cport) {
		// control traffic
		clientstr = "control";
		fd = control_client_fd;
	} else if (1 == cport) {
		// gpio traffic
		clientstr = "gpio";
		fd = gpio_client_fd;
	} else {
		D("unmapped cport %d", fd);
		return -EINVAL;
	}

	//D("%s: cport: %u, buf: %p, len: %u", clientstr, cport, buf, len);

	r = sendMessage(fd, msg);

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

static void register_gb_platform_drivers()
{
	/* Platform drivers are no longer required.
	 * All that is required is a bijection
	 * of cports and devices */

	const unsigned int cport = 1;
	const char *port = DT_GPIO_LABEL(DT_ALIAS(led0), gpios);
	const unsigned pin = DT_GPIO_PIN(DT_ALIAS(led0), gpios);
	D("led0 = device %s pin %u", port, pin);

	struct device *dev = device_get_binding(port);
	__ASSERT(dev != NULL, "failed to get Device Tree binding for led0");
	D("led0 device %s is bound to %p", port, dev);

	int r = gb_add_cport_device_mapping(cport, dev);
	(void)r;
	__ASSERT(r == 0, "failed to add (cport: %u dev: %p", cport, dev);

	D("mapped cport %u to device %s", cport, port);
}

static int gbsetup(void)
{
	int r;
	void *manifest;
	size_t manifest_size;

	D("Registering platform drivers..");
	register_gb_platform_drivers();

	D("Getting static manifest blob..");
	manifest = get_manifest_blob();
	manifest_size = (size_t)manifest_mnfb_len;

	D("Parsing manifest..");
	r = manifest_parse(manifest, manifest_size);
	if (true != r) {
		D("failed to parse manifest");
		return -EINVAL;
	}
	D("Parsed manifest");

	D("Updating manifest blob..");
	set_manifest_blob(manifest);

	D("Initializing Greybus..");
	r = gb_init((struct gb_transport_backend *)&gb_xport);
	if (0 != r) {
		D("gb_init() failed (%d)", r);
		return r;
	}

	D("Enabling Cports..");
	enable_cports();

	D("Greybus is active.");

	return 0;
}

int main(int argc, char *argv[])
{
	return gbsetup() || netsetup() || accept_loop();
}
