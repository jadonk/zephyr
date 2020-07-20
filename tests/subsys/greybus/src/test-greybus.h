#ifndef TESTS_SUBSYS_TEST_GREYBUS_H_
#define TESTS_SUBSYS_TEST_GREYBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#if DT_NODE_HAS_STATUS(DT_INST(0, test_greybus_gpio), okay)
/* Execution of the test requires hardware configuration described in
 * devicetree.  See the test,gpio_basic_api binding local to this test
 * for details.
 *
 * If this is not present devices that have gpio-0, gpio-1, or gpio-2
 * aliases are supported for build-only tests.
 */
#define DEV_NAME DT_GPIO_LABEL(DT_INST(0, test_greybus_gpio), out_gpios)
#define PIN_OUT DT_GPIO_PIN(DT_INST(0, test_greybus_gpio, out_gpios)
#define PIN_IN DT_GPIO_PIN(DT_INST(0, test_greybus_gpio), in_gpios)

#elif DT_NODE_HAS_STATUS(DT_ALIAS(gpio_0), okay)
#define DEV_NAME DT_LABEL(DT_ALIAS(gpio_0))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(gpio_1), okay)
#define DEV_NAME DT_LABEL(DT_ALIAS(gpio_1))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(gpio_3), okay)
#define DEV_NAME DT_LABEL(DT_ALIAS(gpio_3))
#else
#error Unsupported board
#endif

#ifndef PIN_OUT
/* For build-only testing use fixed pins. */
#define PIN_OUT 2
#define PIN_IN 3
#endif

#ifdef __cplusplus
}
#endif

#endif /* TESTS_SUBSYS_TEST_GREYBUS_H_ */