#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>

#if defined(CONFIG_BOARD_NATIVE_POSIX_64BIT) \
    || defined(CONFIG_BOARD_NATIVE_POSIX_32BIT) \
    || defined(CONFIG_BOARD_NRF52_BSIM)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sys/byteorder.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

extern int usleep(useconds_t usec);

#else

#include <posix/unistd.h>
#include <posix/pthread.h>
#include <greybus/debug.h>
#include <greybus/greybus.h>
#include <greybus/platform.h>
#include <greybus-utils/manifest.h>

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

#endif

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_transport_tcpip);

#include "transport.h"

/* Based on UniPro, from Linux */
#define CPORT_ID_MAX 4095

enum { GB_TYPE_MANIFEST_SET = 0x42,
       GB_TYPE_ANY = (uint8_t)-1,
};

/*
static size_t num_gb_transport_fds;
static int *gb_transport_fds;
*/

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
		LOG_DBG("invalid fd %d", *fd);
		return NULL;
	}

	for (;;) {
		r = getMessage(*fd, &msg, GB_TYPE_ANY);
		if (0 == r) {
			LOG_DBG("%s: getMessage() returned EOF", clientstr);
			break;
		}
		if (r < 0) {
			LOG_DBG("%s: failed to receive message (%d)", clientstr, r);
			break;
		}

		//LOG_DBG("%s: Received message", clientstr);

		r = greybus_rx_handler(cport, msg, sys_le16_to_cpu(msg->size));
		if ( 0 == r ) {
			//LOG_DBG("%s: Handled message!", clientstr);
		} else {
			LOG_DBG("%s: failed to handle message %u: size: %u, id: %u, type: %u", clientstr, i, sys_le16_to_cpu(msg->size), sys_le16_to_cpu(msg->id), msg->type);
		}
	}

	LOG_DBG("%s: closing fd %d", clientstr, *fd);
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

#if !(defined(CONFIG_BOARD_NATIVE_POSIX_64BIT) \
    || defined(CONFIG_BOARD_NATIVE_POSIX_32BIT) \
    || defined(CONFIG_BOARD_NRF52_BSIM))
    
    LOG_DBG("initializing control_thread stack");
	r = pthread_attr_init(&control_thread_attr);
	if (0 != r) {
		LOG_DBG("pthred_attr_init() failed: %d", r);
		return r;
	}
	r = pthread_attr_setstack(&control_thread_attr, &control_thread_stack,
				  STACK_SIZE);
	if (0 != r) {
		LOG_DBG("pthred_attr_setstack() failed: %d", r);
		return r;
	}

	LOG_DBG("initializing gpio_thread stack");
	r = pthread_attr_init(&gpio_thread_attr);
	if (0 != r) {
		LOG_DBG("pthred_attr_init() failed: %d", r);
		return r;
	}
	r = pthread_attr_setstack(&gpio_thread_attr, &gpio_thread_stack,
				  STACK_SIZE);
	if (0 != r) {
		LOG_DBG("pthred_attr_setstack() failed: %d", r);
		return r;
	}

	control_thread_attr_p = &control_thread_attr;
	gpio_thread_attr_p = &gpio_thread_attr;
#endif

	LOG_DBG("creating control server socket");
	control_server_fd = socket(AF_INET6, SOCK_STREAM, 0);
	if (-1 == control_server_fd) {
		perror("socket");
		return -errno;
	}
	LOG_DBG("creating gpio server socket");
	gpio_server_fd = socket(AF_INET6, SOCK_STREAM, 0);
	if (-1 == gpio_server_fd) {
		perror("socket");
		return -errno;
	}

	LOG_DBG("setting socket options for control server");
	r = setsockopt(control_server_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
		       sizeof(yes));
	if (-1 == r) {
		perror("setsockopt");
		return -errno;
	}

	LOG_DBG("setting socket options for gpio server");
	r = setsockopt(gpio_server_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
		       sizeof(yes));
	if (-1 == r) {
		perror("setsockopt");
		return -errno;
	}

	LOG_DBG("binding control server socket");
	addr.sin6_port = htons(4242);
	r = bind(control_server_fd, (struct sockaddr *)&addr, sizeof(addr));
	if (-1 == r) {
		perror("bind");
		return -errno;
	}
	LOG_DBG("binding gpio server socket");
	addr.sin6_port = htons(4243);
	r = bind(gpio_server_fd, (struct sockaddr *)&addr, sizeof(addr));
	if (-1 == r) {
		perror("bind");
		return -errno;
	}

	LOG_DBG("listening on control server socket");
	r = listen(control_server_fd, 1);
	if (-1 == r) {
		perror("listen");
		return -errno;
	}
	LOG_DBG("listening on gpio server socket");
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
		LOG_DBG("preparing pollfds");
		memset(pollfds, 0, sizeof(pollfds));
		pollfds[0].fd = control_server_fd;
		pollfds[0].events = POLLIN;
		pollfds[1].fd = gpio_server_fd;
		pollfds[1].events = POLLIN;

		LOG_DBG("calling poll");
		r = poll(pollfds, ARRAY_SIZE(pollfds), -1);
		if (-1 == r) {
			perror("poll");
			return -errno;
		}
		LOG_DBG("returned from poll");
		if (pollfds[0].revents & POLLIN) {
			LOG_DBG("control socket has a traffic");
			addrlen = sizeof(addr);
			control_client_fd =
				accept(control_server_fd,
				       (struct sockaddr *)&addr, &addrlen);
			if (-1 == control_client_fd) {
				perror("accept");
				return -errno;
			}

			memset(addrstr, '\0', sizeof(addrstr));
			addrstrp = (char *)inet_ntop(AF_INET6, &addr.sin6_addr, addrstr, sizeof(addrstr));
			if (NULL == addrstrp) {
				perror("inet_ntop");
				return -errno;
			}
			LOG_DBG("accepted connection from [%s]:%d as fd %d", addrstr, ntohs(addr.sin6_port), control_client_fd);

			LOG_DBG("spawning control thread..");
			r = pthread_create(&control_thread,
					   control_thread_attr_p, thread_fun,
					   &control_client_fd);
			if (0 != r) {
				perror("pthread_create");
				return r;
			}
		}
		if (pollfds[1].revents & POLLIN) {
			LOG_DBG("gpio service has traffic");
			addrlen = sizeof(addr);
			gpio_client_fd =
				accept(gpio_server_fd, (struct sockaddr *)&addr,
				       &addrlen);
			if (-1 == gpio_client_fd) {
				perror("accept");
				return -errno;
			}

			memset(addrstr, '\0', sizeof(addrstr));
			addrstrp = (char *)inet_ntop(AF_INET6, &addr.sin6_addr, addrstr, sizeof(addrstr));
			if (NULL == addrstrp) {
				perror("inet_ntop");
				return -errno;
			}
			LOG_DBG("accepted connection from [%s]:%d as fd %d", addrstr, ntohs(addr.sin6_port), gpio_client_fd);

			LOG_DBG("spawning gpio thread..");
			pthread_create(&gpio_thread, gpio_thread_attr_p,
				       thread_fun, &gpio_client_fd);
			if (0 != r) {
				perror("pthread_create");
				return -errno;
			}
		}
	}

	LOG_DBG("out of loop");
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
		//LOG_DBG("attempting to read %u bytes", remaining);
		r = recv(fd, &((uint8_t *)*msg)[offset], remaining, 0);
		//LOG_DBG("recv(%d, &%p[%u], %u, 0) => %d", fd, *msg, offset, remaining, r, errno);
		if (r < 0) {
			LOG_DBG("recv failed. errno: %d (%s)", errno, strerror(errno));
			usleep(100);
			continue;
		}
		if (0 == r) {
			LOG_DBG("recv returned 0 - EOF??");
			goto freemsg;
		}
		recvd = r;
	}

	msg_size = sys_le16_to_cpu((*msg)->size);
	//LOG_DBG("msg_size is %u", (unsigned)msg_size);

	if (msg_size < sizeof(struct gb_operation_hdr)) {
		LOG_DBG("invalid message size %u", (unsigned)msg_size);
		goto read_header;
	}

	payload_size = msg_size - sizeof(**msg);
	//LOG_DBG("payload_size is %u", (unsigned)payload_size);

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
			//LOG_DBG("attempting to read %u bytes", remaining);
			r = recv(fd, &((uint8_t *)*msg)[offset], remaining, 0);
			//LOG_DBG("recv(%d, &%p[%u], %u) => %d", fd, *msg, offset, remaining, r);
			if (r < 0) {
				LOG_DBG("tty_read failed (%d)", r);
				usleep(100);
				continue;
			}
			if (0 == r) {
				LOG_DBG("tty_read returned 0 - EOF??");
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
		LOG_DBG("Expected message type %d but received messsage type %d",
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
		LOG_DBG("unmapped fd %d", fd);
		return -EINVAL;
	}

	if (NULL == msg) {
		LOG_DBG("%s: One or more arguments were NULL or invalid", clientstr);
		r = -EINVAL;
		goto out;
	}

	for (remaining = sys_le16_to_cpu(msg->size), offset = 0, written = 0;
	     remaining; remaining -= written, offset += written, written = 0) {
		r = send(fd, &((uint8_t *)msg)[offset], remaining, 0);
		//LOG_DBG("%s: send(%d, &%p[%u], %u, 0) => %d", clientstr, fd, msg, offset, remaining, r);
		if (r < 0) {
			LOG_DBG("%s: send failed. errno: %d (%s)", clientstr, errno, strerror(errno));
			return r;
		}
		if (0 == r) {
			LOG_DBG("%s: send returned 0 - EOF??", clientstr);
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
		LOG_DBG("unmapped cport %d", fd);
		return -EINVAL;
	}

	//LOG_DBG("%s: cport: %u, buf: %p, len: %u", clientstr, cport, buf, len);

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

struct gb_transport_backend *gb_transport_get_backend(size_t num_cports) {

    struct gb_transport_backend *ret;

    if (num_cports >= CPORT_ID_MAX) {
        LOG_ERR("invalid number of cports %u", (unsigned)num_cports);
        ret = NULL;
        goto out;
    }

    //gb_transport_fds = realloc(gb_transport_fds, num_cports * sizeof(int));
    if (0) {
        netsetup();
    }

    ret = (struct gb_transport_backend *)&gb_xport;

out:
    return ret;
}
