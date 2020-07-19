/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

#define LOG_LEVEL 11
#include <logging/log.h>
LOG_MODULE_REGISTER(greybus_test);

extern void test_greybus_gpio_protocol_version(void);
extern void test_greybus_gpio_cport_shutdown(void);
extern void test_greybus_gpio_line_count(void);
extern void test_greybus_gpio_activate(void);
extern void test_greybus_gpio_deactivate(void);
extern void test_greybus_gpio_get_direction(void);
extern void test_greybus_gpio_direction_input(void);
extern void test_greybus_gpio_direction_output(void);
extern void test_greybus_gpio_get_value(void);
extern void test_greybus_gpio_set_value(void);
extern void test_greybus_gpio_set_debounce(void);
extern void test_greybus_gpio_irq_type(void);
extern void test_greybus_gpio_irq_mask(void);
extern void test_greybus_gpio_irq_unmask(void);
extern void test_greybus_gpio_irq_event(void);

void test_main(void) {

    ztest_test_suite(greybus_gpio,
        /*
        ztest_unit_test(test_greybus_gpio_protocol_version),
        ztest_unit_test(test_greybus_gpio_cport_shutdown),
        */
        ztest_unit_test(test_greybus_gpio_line_count)/*,
        ztest_unit_test(test_greybus_gpio_activate),
        ztest_unit_test(test_greybus_gpio_deactivate),
        ztest_unit_test(test_greybus_gpio_get_direction),
        ztest_unit_test(test_greybus_gpio_direction_input),
        ztest_unit_test(test_greybus_gpio_direction_output),
        ztest_unit_test(test_greybus_gpio_get_value),
        ztest_unit_test(test_greybus_gpio_set_value),
        ztest_unit_test(test_greybus_gpio_set_debounce),
        ztest_unit_test(test_greybus_gpio_irq_type),
        ztest_unit_test(test_greybus_gpio_irq_mask),
        ztest_unit_test(test_greybus_gpio_irq_unmask),
        ztest_unit_test(test_greybus_gpio_irq_event)
        */
        );

    ztest_run_test_suite(greybus_gpio);
}
