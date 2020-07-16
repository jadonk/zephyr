/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <zephyr.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test);

void test_main(void) {
	LOG_INF("%s(): ahoy!", __func__);
    for(;;) {
    	k_sleep(K_MSEC(500));
    }
}
