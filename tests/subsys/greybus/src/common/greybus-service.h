#ifndef ZEPHYR_GREYBUS_SERVICE_H_
#define ZEPHYR_GREYBUS_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cpluspus */

int z_greybus_register_interface(void *const interface);
int z_greybus_register_bundle(void *const bundle);
int z_greybus_register_control(void *const control);
int z_greybus_register_gpio(void *const gpio);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_GREYBUS_SERVICE_H_ */
