#include <unordered_set>

#include "greybus-service.h"

using namespace std;

static unordered_set<void*> z_greybus_interfaces;
static unordered_set<void*> z_greybus_bundles;
static unordered_set<void*> z_greybus_controls;
static unordered_set<void*> z_greybus_gpio;

int z_greybus_register_interface(void *const interface)
{
	int ret = 0;

	LOG_INF("registering interface %p", interface);

	auto it = z_greybus_interfaces.find(interface);
	if (it != z_greybus_interfaces.end()) {
		ret = -EALREADY;
	}
	z_greybus_interfaces.insert(interface);

	return ret;
}

int z_greybus_register_bundle(void *const bundle)
{
	int ret = 0;

	LOG_INF("registering bundle %p", bundle);

	auto it = z_greybus_bundles.find(bundle);
	if (it != z_greybus_bundles.end()) {
		ret = -EALREADY;
	}
	z_greybus_bundles.insert(bundle);

	return ret;
}

int z_greybus_register_control(void *const control)
{
	int ret = 0;

	LOG_INF("registering control %p", control);

	auto it = z_greybus_controls.find(control);
	if (it != z_greybus_controls.end()) {
		ret = -EALREADY;
	}
	z_greybus_controls.insert(control);

	return ret;
}

int z_greybus_register_gpio(void *const gpio)
{
	int ret = 0;

	LOG_INF("registering gpio %p", gpio);

	auto it = z_greybus_gpios.find(gpio);
	if (it != z_greybus_gpios.end()) {
		ret = -EALREADY;
	}
	z_greybus_gpios.insert(gpio);

	return ret;
}
