#ifndef GREYBUS_TRANSPORT_H_
#define GREYBUS_TRANSPORT_H_

#include <greybus/greybus.h>

struct gb_transport_backend *gb_transport_get_backend(unsigned int *cports, size_t num_cports);

#endif /* GREYBUS_TRANSPORT_H_ */
