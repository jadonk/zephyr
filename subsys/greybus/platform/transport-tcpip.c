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

#include <greybus/debug.h>
#include <greybus/greybus.h>
#include <greybus/platform.h>
#include <greybus-utils/manifest.h>
#include <posix/unistd.h>
#include <posix/pthread.h>


unsigned int sleep(unsigned int seconds)
{
	k_sleep(K_MSEC(1000 * seconds));
	return 0;
}

int usleep(useconds_t usec) {
	k_usleep(usec);
	return 0;
}

#endif

//#define LOG_LEVEL CONFIG_GB_LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_transport_tcpip);

#include "transport.h"

/* Based on UniPro, from Linux */
#define CPORT_ID_MAX 4095

#define GB_TRANSPORT_TCPIP_BASE_PORT 4242
#define GB_TRANSPORT_TCPIP_BACKLOG 1

struct gb_transport_tcpip_context {
    int server_fd;
    /* currently, only one simultaneous connection per cport is supported */
    int client_fd;
    unsigned int cport;
    pthread_t client_thread;
};

static int getMessage(int fd, struct gb_operation_hdr **msg);
static int sendMessage(int fd, struct gb_operation_hdr *msg);
struct pollfd *pollfds;

static size_t num_gb_transport_tcpip_contexts;
static struct gb_transport_tcpip_context *gb_transport_tcpip_contexts;

static pthread_t accept_thread;

static void *thread_fun(void *arg)
{
	int r;
	int i = 0;
	struct gb_operation_hdr *msg = NULL;
    struct gb_transport_tcpip_context *ctx = NULL;
	int *fd = (int *)arg;
    unsigned int cport = -1;

    if (fd == NULL) {
    	LOG_ERR("invalid argument");
    	goto out;
    }

    for(size_t i = 0; i < num_gb_transport_tcpip_contexts; ++i) {
        ctx = &gb_transport_tcpip_contexts[i];
        if (*fd == ctx->client_fd) {
            cport = ctx->cport;
            break;
        }
    }

    if (ctx == NULL) {
    	LOG_ERR("no context for fd %d!", *fd);
    	goto out;
    }

    if (cport == -1) {
    	LOG_ERR("cport not found for fd %d!", *fd);
    	goto out;
    }


	for (;;) {
		r = getMessage(*fd, &msg);
		if (0 == r) {
			LOG_DBG("recv from fd %d returned EOF", *fd);
			break;
		}
		if (r < 0) {
			LOG_DBG("failed to receive message (%d)", r);
			break;
		}

		r = greybus_rx_handler(cport, msg, sys_le16_to_cpu(msg->size));
		if (r < 0) {
			LOG_DBG("failed to handle message %u: size: %u, id: %u, type: %u",
				i, sys_le16_to_cpu(msg->size), sys_le16_to_cpu(msg->id),
				msg->type);
		}
	}

out:
	if (*fd != -1) {
		LOG_DBG("closing fd %d", *fd);
		close(*fd);
		*fd = -1;
	}

	return NULL;
}

static int netsetup(void)
{
	int r;
    int i;
	const int yes = true;
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_addr = in6addr_any,
        .sin6_port = htons(GB_TRANSPORT_TCPIP_BASE_PORT),
	};
    struct gb_transport_tcpip_context *ctx; 

    for(i = 0; i < num_gb_transport_tcpip_contexts; ++i) {
        ctx = &gb_transport_tcpip_contexts[i];

        ctx->server_fd = socket(AF_INET6, SOCK_STREAM, 0);
        if (ctx->server_fd == -1) {
            r = -errno;
            LOG_ERR("socket: %d", errno);
            goto cleanup;
        }
	    LOG_DBG("created server socket %d for cport %u",
            ctx->server_fd, ctx->cport);

        LOG_DBG("setting socket options for socket %d", ctx->server_fd);
        r = setsockopt(ctx->server_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
                sizeof(yes));
        if (-1 == r) {
            LOG_ERR("setsockopt: %d", errno);
            r = -errno;
            goto cleanup;
        }

        LOG_DBG("binding socket %d (cport %u) to port %u",
            ctx->server_fd, ctx->cport, GB_TRANSPORT_TCPIP_BASE_PORT + i);
    	addr.sin6_port = htons(GB_TRANSPORT_TCPIP_BASE_PORT + i);
        r = bind(ctx->server_fd, (struct sockaddr *)&addr, sizeof(addr));
        if (-1 == r) {
            LOG_ERR("bind: %d", errno);
            r = -errno;
            goto cleanup;
        }

        LOG_DBG("listening on socket %d (cport %u)", ctx->server_fd, ctx->cport);
        r = listen(ctx->server_fd, GB_TRANSPORT_TCPIP_BACKLOG);
        if (-1 == r) {
            LOG_ERR("listen: %d", errno);
            r = -errno;
            goto cleanup;
        }
    }

    r = 0;
    goto out;


cleanup:
    for(--i; i >= 0; --i) {
        ctx = &gb_transport_tcpip_contexts[i];
        if (ctx != NULL && ctx->server_fd != -1) {
			close(ctx->server_fd);
			ctx->server_fd = -1;
        }
    }

out:
    return r;
}

void prepare_pollfds(void) {
    struct gb_transport_tcpip_context *ctx; 
    memset(pollfds, 0, num_gb_transport_tcpip_contexts * sizeof(*pollfds));
    for(size_t i = 0; i < num_gb_transport_tcpip_contexts; ++i) {
        ctx = &gb_transport_tcpip_contexts[i];
		pollfds[i].fd = ctx->server_fd;
		pollfds[i].events = POLLIN;
    }
}

void *accept_loop(void *arg)
{
	int r;
	unsigned int num_pollfds;
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_addr = in6addr_any,
	};
	socklen_t addrlen;
	static char addrstr[INET6_ADDRSTRLEN];
	char *addrstrp;
    struct gb_transport_tcpip_context *ctx;

	for (;;) {

        prepare_pollfds();

		LOG_DBG("calling poll");
		r = poll(pollfds, num_gb_transport_tcpip_contexts, -1);
		if (-1 == r) {
			LOG_ERR("poll: %d", errno);
			return NULL;
		}

		num_pollfds = r;
		LOG_DBG("poll returned %d", num_pollfds);

        for(size_t i = 0; num_pollfds > 0 && i < num_gb_transport_tcpip_contexts; ++i) {
            if (pollfds[i].revents & POLLIN) {

                ctx = &gb_transport_tcpip_contexts[i];

                LOG_DBG("socket %d (cport %u) has traffic",
                    ctx->server_fd, ctx->cport);

                addrlen = sizeof(addr);
                ctx->client_fd =
                    accept(ctx->server_fd,
                        (struct sockaddr *)&addr, &addrlen);
                if (ctx->client_fd == -1) {
                    LOG_ERR("accept: %d", errno);
                    return NULL;
                }

                memset(addrstr, '\0', sizeof(addrstr));
                addrstrp = (char *)inet_ntop(AF_INET6, &addr.sin6_addr,
                    addrstr, sizeof(addrstr));
                if (NULL == addrstrp) {
                    LOG_ERR("inet_ntop: %d", errno);
                    return NULL;
                }
                LOG_DBG("accepted connection from [%s]:%d as fd %d",
                    log_strdup(addrstr), ntohs(addr.sin6_port), ctx->client_fd);

                LOG_DBG("spawning client thread..");
                r = pthread_create(&ctx->client_thread,
                        NULL, thread_fun,
                        &ctx->client_fd);
                if (r != 0) {
                    LOG_ERR("pthread_create: %d", r);
                    return NULL;
                }

                num_pollfds--;
            }
        }
	}

	return NULL;
}

static int getMessage(int fd, struct gb_operation_hdr **msg)
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

		r = recv(fd, &((uint8_t *)*msg)[offset], remaining, 0);
		if (r < 0) {
			LOG_DBG("recv failed. errno: %d", errno);
			goto freemsg;
		}
		if (0 == r) {
			LOG_DBG("recv returned 0 - EOF??");
			goto freemsg;
		}
		recvd = r;
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
			r = recv(fd, &((uint8_t *)*msg)[offset], remaining, 0);
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

	for (remaining = sys_le16_to_cpu(msg->size), offset = 0, written = 0;
	     remaining; remaining -= written, offset += written, written = 0) {
		r = send(fd, &((uint8_t *)msg)[offset], remaining, 0);
		if (r < 0) {
			LOG_ERR("send: %d", errno);
			return r;
		}
		if (0 == r) {
			LOG_ERR("send returned 0 - EOF?");
			return -1;
		}
		written = r;
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
	int fd = -1;
	struct gb_operation_hdr *msg;
	struct gb_transport_tcpip_context *ctx;

	msg = (struct gb_operation_hdr *)buf;
    if (NULL == msg) {
		LOG_ERR("message is NULL");
	    return -EINVAL;
	}

	for(size_t i = 0; i < num_gb_transport_tcpip_contexts; ++i) {
		ctx = &gb_transport_tcpip_contexts[i];
		if (ctx->cport == cport) {
			fd = ctx->client_fd;
			break;
		}
	}

	if (fd == -1) {
		LOG_ERR("unable to find fd corresponding to cport %u", cport);
		return -EINVAL;
	}

    if (sys_le16_to_cpu(msg->size) != len || len < sizeof(*msg)) {
		LOG_ERR("invalid message size %u (len: %u)",
			(unsigned)sys_le16_to_cpu(msg->size), (unsigned)len);
        return -EINVAL;
    }

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

struct gb_transport_backend *gb_transport_backend_init(unsigned int *cports, size_t num_cports) {

    int r;
    struct gb_transport_tcpip_context *ctx;
    struct gb_transport_backend *ret;

	if (num_gb_transport_tcpip_contexts != 0) {
		LOG_ERR("Greybus TCP/IP Transport has already been initialized");
		return NULL;
	}

	LOG_DBG("Greybus TCP/IP Transport initializing..");

    if (num_cports >= CPORT_ID_MAX) {
        LOG_ERR("invalid number of cports %u", (unsigned)num_cports);
        ret = NULL;
        goto out;
    }

    pollfds = realloc(pollfds, num_cports * sizeof(*pollfds));
    if (pollfds == NULL) {
        LOG_ERR("failed to allocate file descriptors");
        ret = NULL;
        goto out;
    }

    ctx = realloc(gb_transport_tcpip_contexts, num_cports * sizeof(*gb_transport_tcpip_contexts));
    if (ctx == NULL) {
        LOG_ERR("failed to allocate file descriptors");
        ret = NULL;
        goto cleanup;
    }

    num_gb_transport_tcpip_contexts = num_cports;
    gb_transport_tcpip_contexts = ctx;

    for(size_t i = 0; i < num_cports; ++i) {
        ctx = &gb_transport_tcpip_contexts[i];
        ctx->server_fd = -1;
        ctx->client_fd = -1;
        ctx->cport = cports[i];
    }

    r = netsetup();
    if (r < 0) {
        goto cleanup;
    }

	r = pthread_create(&accept_thread, NULL, accept_loop, NULL);
	if (r != 0) {
		LOG_ERR("pthread_create: %d", r);
		goto cleanup;
	}

    ret = (struct gb_transport_backend *)&gb_xport;

	LOG_INF("Greybus TCP/IP Transport initialized");
	goto out;

cleanup:
    if (pollfds != NULL) {
        free(pollfds);
        pollfds = NULL;
    }
    num_gb_transport_tcpip_contexts = 0;
    if (gb_transport_tcpip_contexts != NULL) {
        free(gb_transport_tcpip_contexts);
        gb_transport_tcpip_contexts = NULL;
    }

out:
    return ret;
}
