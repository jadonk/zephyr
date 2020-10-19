/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_DNS_SD_H_
#define ZEPHYR_INCLUDE_NET_DNS_SD_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** RFC 1034 Section 3.1 */
#define DNS_SD_INSTANCE_MIN_SIZE 1
/** RFC 1034 Section 3.1, RFC 6763 Section 7.2 */
#define DNS_SD_INSTANCE_MAX_SIZE 63
/** RFC 6763 Section 7.2 - inclusive of underscore */
#define DNS_SD_SERVICE_MIN_SIZE 2
/** RFC 6763 Section 7.2 - inclusive of underscore */
#define DNS_SD_SERVICE_MAX_SIZE 16
/** RFC 6763 Section 4.1.2 */
#define DNS_SD_SERVICE_PREFIX '_'
/** RFC 6763 Section 4.1.2 - either _tcp or _udp (case insensitive) */
#define DNS_SD_PROTO_SIZE 4
/** ICANN Rules for TLD naming */
#define DNS_SD_DOMAIN_MIN_SIZE 2
/** RFC 1034 Section 3.1, RFC 6763 Section 7.2 */
#define DNS_SD_DOMAIN_MAX_SIZE 63

/**
 * Register a service for DNS Service Discovery
 *
 * Example (no TXT):
 * @code{c}
 * #include <net/dns_sd.h>
 * DNS_SD_REGISTER_UDP_SERVICE(foo, "My Foo", "_foo", DNS_SD_EMPTY_TXT, 4242);
 * @endcode{cpp}
 *
 * TXT records begin with a single length byte (hex-encoded)
 * and contain key=value pairs.
 *
 * For additional rules on TXT encoding, see RFC 6763, Section 6.4.
 *
 * Example (with TXT):
 * @code{c}
 * #include <net/dns_sd.h>
 * static const bar_txt[] = {
 *   "\x06" "path=/"
 *   "\x0f" "this=is the way"
 *   "\x0e" "foo or=foo not"
 *   "\x17" "this=has\0embedded\0nulls"
 *   "\x04" "true"
 * };
 * DNS_SD_REGISTER_TCP_SERVICE(bar, "My Bar", "_bar", bar_txt, 0x4242);
 * @endcode{cpp}
 */
#define DNS_SD_REGISTER_SERVICE(id, instance, service, proto, text, port) \
	const Z_STRUCT_SECTION_ITERABLE(dns_sd_rec, id) = {		  \
		instance,						  \
		service,						  \
		proto,							  \
		"local",						  \
		(const char *)text,					  \
		sizeof(text) - 1,					  \
		port							  \
	}

#define DNS_SD_REGISTER_TCP_SERVICE(id, instance, service, text, port) \
	DNS_SD_REGISTER_SERVICE(id, instance, service, "_tcp", text,   \
				port)

#define DNS_SD_REGISTER_UDP_SERVICE(id, instance, service, text, port) \
	DNS_SD_REGISTER_SERVICE(id, instance, service, "_udp", text,   \
				port)

/**
 * Properly determine the size of DNS-SD text data
 *
 * Please do not use strlen() on @ref dns_sd_rec.text .
 */
#define DNS_SD_TXT_SIZE(ref) (ref->text_size)

/** Empty DNS-SD TXT specifier */
#define DNS_SD_EMPTY_TXT dns_sd_empty_txt

struct dns_sd_rec {
	const char *instance;
	const char *service;
	const char *proto;
	const char *domain;
	const char *text;
	size_t text_size;
	uint16_t port;
};

extern const char dns_sd_empty_txt[1];

#ifdef __cplusplus
};
#endif

#endif /* ZEPHYR_INCLUDE_NET_DNS_SD_H_ */
