/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>

extern void test_i2c_gy271(void);
extern void test_i2c_burst_gy271(void);

static void board_setup(void)
{
	if (IS_ENABLED(CONFIG_I2C_SIM)) {
		extern void sim_setup(void);
		sim_setup();
	}
}

void test_main(void)
{
	board_setup();
	ztest_test_suite(i2c_test,
			 ztest_unit_test(test_i2c_gy271),
			 ztest_unit_test(test_i2c_burst_gy271));
	ztest_run_test_suite(i2c_test);
}
