/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DNS_SD_H_
#define DNS_SD_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <net/dns_sd.h>
#include <net/net_ip.h>

#include "dns_pack.h"

/* TODO: Move these into Kconfig */
#define DNS_SD_PTR_TTL 4500
#define DNS_SD_TXT_TTL 4500
#define DNS_SD_SRV_TTL 120
#define DNS_SD_A_TTL 120
#define DNS_SD_AAAA_TTL 120

#define DNS_SD_PTR_MASK (NS_CMPRSFLGS << 8)

#ifdef __cplusplus
extern "C" {
#endif

#define DNS_SD_FOREACH(it) \
	Z_STRUCT_SECTION_FOREACH(dns_sd_rec, it)

struct dns_header {
	/** Transaction ID */
	uint16_t id;
	/**
	 * | Name | Bit Position | Width | Description |
	 * |------|--------------|-------|-------------|
	 * | RCODE | 0 | 4 | Response / Error code |
	 * | CD | 4 | 1 | |
	 * | AD | 5 | 1 | Authenticated Data. 0 := Unacceptable, 1 := Acceptable |
	 * | Z | 6 | 1 | Reserved (WZ/RAZ) |
	 * | RA | 7 | 1 | Recursion Available. 0 := Unavailable, 1 := Available |
	 * | RD | 8 | 1 | Recursion Desired. 0 := No Recursion, 1 := Recursion |
	 * | TC | 9 | 1 | 0 := Not Truncated, 1 := Truncated |
	 * | AA | 10 | 1 | Answer Authenticated / Answer Authoritative. 0 := Not Authenticated, 1 := Authenticated|
	 * | Opcode | 11 | 4 | See @ref dns_opcode |
	 * | QR | 15 | 1 | 0 := Query, 1 := Response |
	 */
	uint16_t flags;
	/** Query count */
	uint16_t qdcount;
	/** Answer count */
	uint16_t ancount;
	/** Authority count */
	uint16_t nscount;
	/** Additional information count */
	uint16_t arcount;
	/** Flexible array member for records */
	uint8_t data[];
} __packed;

struct dns_query {
	uint16_t type;
	uint16_t class_;
} __packed;

struct dns_rr {
	uint16_t type;
	uint16_t class_;
	uint32_t ttl;
	uint16_t rdlength;
	uint8_t rdata[];
} __packed;

struct dns_srv_rdata {
	uint16_t priority;
	uint16_t weight;
	uint16_t port;
} __packed;

struct dns_a_rdata {
	uint32_t address;
} __packed;

struct dns_aaaa_rdata {
	uint8_t address[16];
} __packed;

int dns_sd_handle_ptr_query(const struct dns_sd_rec *inst, bool ipv6,
			    const uint8_t *addr,
			    const struct dns_query *query,
			    uint8_t *buf, uint16_t buf_size);

#ifdef __cplusplus
};
#endif

#endif /* DNS_SD_H_ */
