/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <greybus/platform.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr.h>

struct map_entry {
	unsigned int a;
	void *b;
};

static size_t map_size;
static struct map_entry *map;
K_MUTEX_DEFINE(map_mutex);

int gb_add_cport_device_mapping(unsigned int cport, struct device *dev)
{
	int ret;
	int mutex_ret;
	size_t idx;
	struct map_entry *entry;

	if (dev == NULL) {
		return -EINVAL;
	}

	mutex_ret = k_mutex_lock(&map_mutex, K_FOREVER);
	__ASSERT_NO_MSG(mutex_ret == 0);

	for(idx = 0; idx < map_size; ++idx) {
		entry = &map[idx];
		if (entry->a == cport || entry->b == dev) {
			ret = -EALREADY;
			goto unlock;
		}
	}

	entry = realloc(map, (map_size + 1) * sizeof(*entry));
	if (entry == NULL) {
		ret = -ENOMEM;
		goto unlock;
	}

	map = entry;
	entry = &map[map_size];
	map_size++;

	entry->a = cport;
	entry->b = dev;

	ret = 0;

unlock:
	mutex_ret = k_mutex_unlock(&map_mutex);
	__ASSERT_NO_MSG(mutex_ret == 0);

	return ret;
}

int gb_device_to_cport(struct device *dev)
{
	int ret;
	int mutex_ret;
	size_t idx;
	struct map_entry *entry;

	mutex_ret = k_mutex_lock(&map_mutex, K_FOREVER);
	__ASSERT_NO_MSG(mutex_ret == 0);

	for(idx = 0; idx < map_size; ++idx) {
		entry = &map[idx];
		if (entry->b == dev) {
			ret = entry->a;
			goto unlock;
		}
	}

	ret = -ENOENT;

unlock:
	mutex_ret = k_mutex_unlock(&map_mutex);
	__ASSERT_NO_MSG(mutex_ret == 0);

	return ret;
}

struct device *gb_cport_to_device(unsigned int cport)
{
	struct device *ret;
	int mutex_ret;
	size_t idx;
	struct map_entry *entry;

	mutex_ret = k_mutex_lock(&map_mutex, K_FOREVER);
	__ASSERT_NO_MSG(mutex_ret == 0);

	for(idx = 0; idx < map_size; ++idx) {
		entry = &map[idx];
		if (entry->a == cport) {
			ret = entry->b;
			goto unlock;
		}
	}

	ret = NULL;

unlock:
	mutex_ret = k_mutex_unlock(&map_mutex);
	__ASSERT_NO_MSG(mutex_ret == 0);

	return ret;
}
