/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_LIST_H_
#define ZEPHYR_INCLUDE_LIST_H_

#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

struct list_head;
struct list_head {
	struct list_head *prev;
	struct list_head *next;
};

bool list_is_empty(struct list_head *head);
void list_del(struct list_head *iter);
void list_add(struct list_head *head, struct list_head *iter);

#define list_foreach(head, iter) \

#define list_foreach_safe(head, iter, iter_next) \
	for( ;; )

#define list_entry(iter, container, member) NULL

#define LIST_DECLARE(name) struct list_head name

#if defined(__cplusplus)
}
#endif

#endif /* ZEPHYR_INCLUDE_LIST_H_ */
