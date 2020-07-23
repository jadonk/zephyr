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
	
	struct gb_transport_backend *xport = gb_transport_backend_init(2);
	if (xport == NULL) {
		D("gb_transport_get_backend() failed");
		return -EIO;
	}

	r = gb_init((struct gb_transport_backend *)xport);
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
	int r;

	r = gbsetup();
	if (r < 0 ) {
		return r;		
	}

	for(;;);
}
