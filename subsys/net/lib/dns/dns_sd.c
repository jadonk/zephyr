/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include <net/dns_sd.h>
#include <sys/util.h>
#include <zephyr.h>

#include "dns_pack.h"
#include "dns_sd.h"

#define LOG_LEVEL CONFIG_DNS_SD_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(net_dns_sd);

const char dns_sd_empty_txt[1];

#ifndef CONFIG_ZTEST

static size_t dns_sd_service_proto_size(const struct dns_sd_rec *inst);
static bool label_is_valid(const char *label, size_t label_size);
static int dns_sd_rec_check(const struct dns_sd_rec *inst);
static int add_a_record(const struct dns_sd_rec *inst, uint32_t ttl,
			uint16_t host_offset, uint32_t addr, uint8_t *buf,
			uint16_t buf_offset, uint16_t buf_size);
static int add_ptr_record(const struct dns_sd_rec *inst, uint32_t ttl,
			  uint8_t *buf, uint16_t buf_offset, uint16_t buf_size,
			  uint16_t *service_offset, uint16_t *instance_offset,
			  uint16_t *domain_offset);
static int add_txt_record(const struct dns_sd_rec *inst, uint32_t ttl,
			  uint16_t instance_offset, uint8_t *buf,
			  uint16_t buf_offset, uint16_t buf_size);
static int add_aaaa_record(const struct dns_sd_rec *inst, uint32_t ttl,
			   uint16_t host_offset, const uint8_t addr[16],
			   uint8_t *buf, uint16_t buf_offset,
			   uint16_t buf_size);
static int add_srv_record(const struct dns_sd_rec *inst, uint32_t ttl,
			  uint16_t instance_offset, uint16_t domain_offset,
			  uint8_t *buf, uint16_t buf_offset, uint16_t buf_size,
			  uint16_t *host_offset);
static bool dns_sd_rec_is_valid(const struct dns_sd_rec *inst);

#endif

/**
 * Calculate the size of a DNS-SD service
 *
 * This macro calculates the size of the DNS-SD service for a DNS
 * Resource Record (RR).
 *
 * For example, if there is a service called 'My Foo'._http._tcp.local.,
 * then the returned size is 18. That is broken down as shown below.
 *
 * - 1 byte for the size of "_http"
 * - 5 bytes for the value of "_http"
 * - 1 byte for the size of "_tcp"
 * - 4 bytes for the value of "_tcp"
 * - 1 byte for the size of "local"
 * - 5 bytes for the value of "local"
 * - 1 byte for the trailing NUL terminator '\0'
 *
 * @param ref the DNS-SD record
 * @return the size of the DNS-SD service for a DNS Resource Record
 */
size_t dns_sd_service_proto_size(const struct dns_sd_rec *ref)
{
	return 0
	       + DNS_LABEL_LEN_SIZE + strlen(ref->service)
	       + DNS_LABEL_LEN_SIZE + strlen(ref->proto)
	       + DNS_LABEL_LEN_SIZE + strlen(ref->domain)
	       + DNS_LABEL_LEN_SIZE
	;
}

/**
 * Check Label Validity according to RFC 1035, Section 3.5
 *
 * <label> ::= <letter> [ [ <ldh-str> ] <let-dig> ]
 * <ldh-str> ::= <let-dig-hyp> | <let-dig-hyp> <ldh-str>
 * <let-dig-hyp> ::= <let-dig> | -
 * <let-dig> ::= <letter> | <digit>
 * <letter> ::= [a-zA-Z]
 * <digit> ::= [0-9]
 */
bool label_is_valid(const char *label, size_t label_size)
{
	size_t i;

	if (label == NULL) {
		return false;
	}

	if (label_size == 0) {
		/* automatically calculate the length of the string */
		label_size = strlen(label);
	}

	if (label_size < DNS_LABEL_MIN_SIZE ||
	    label_size > DNS_LABEL_MAX_SIZE) {
		return false;
	}

	for (i = 0; i < label_size; ++i) {
		if (isalpha((int)label[i])) {
			continue;
		}

		if (i > 0) {
			if (isdigit((int)label[i])) {
				continue;
			}

			if ('-' == label[i]) {
				continue;
			}
		}

		return false;
	}

	return true;
}


static bool instance_is_valid(const char *instance)
{
	size_t i;
	size_t instance_size;

	if (instance == NULL) {
		LOG_DBG("label is NULL");
		return false;
	}

	instance_size = strlen(instance);
	if (instance_size < DNS_SD_INSTANCE_MIN_SIZE) {
		LOG_DBG("label '%s' is too small (%zu, min: %u)",
			instance, instance_size,
			DNS_SD_INSTANCE_MIN_SIZE);
		return false;
	}

	if (instance_size > DNS_SD_INSTANCE_MAX_SIZE) {
		LOG_DBG("label '%s' is too big (%zu, max: %u)",
			instance, instance_size,
			DNS_SD_INSTANCE_MAX_SIZE);
		return false;
	}

	for (i = 0; i < instance_size; ++i) {
		/* RFC 6763 Section 4.1.1 */
		if (instance[i] <= 0x1f ||
		    instance[i] == 0x7f) {
			LOG_DBG(
				"instance '%s' contains illegal byte 0x%02x",
				instance, instance[i]);
			return false;
		}
	}

	return instance_size;
}

static bool service_is_valid(const char *service)
{
	size_t service_size;

	if (service == NULL) {
		LOG_DBG("label is NULL");
		return false;
	}

	service_size = strlen(service);
	if (service_size < DNS_SD_SERVICE_MIN_SIZE) {
		LOG_DBG("label '%s' is too small (%zu, min: %u)",
			service, service_size, DNS_SD_SERVICE_MIN_SIZE);
		return false;
	}

	if (service_size > DNS_SD_SERVICE_MAX_SIZE) {
		LOG_DBG("label '%s' is too big (%zu, max: %u)",
			service, service_size, DNS_SD_SERVICE_MAX_SIZE);
		return false;
	}

	if (service[0] != DNS_SD_SERVICE_PREFIX) {
		LOG_DBG("service '%s' invalid (no leading underscore)",
			service);
		return false;
	}

	if (!label_is_valid(&service[1], service_size - 1)) {
		LOG_DBG("service '%s' contains invalid characters",
			service);
		return false;
	}

	return service_size;
}

static bool proto_is_valid(const char *proto)
{
	size_t proto_size;

	if (proto == NULL) {
		LOG_DBG("label is NULL");
		return false;
	}

	proto_size = strlen(proto);
	if (proto_size != DNS_SD_PROTO_SIZE) {
		LOG_DBG("label '%s' wrong size (%zu, exp: %u)",
			proto, proto_size, DNS_SD_PROTO_SIZE);
		return false;
	}

	if (!(strncasecmp("_tcp", proto, DNS_SD_PROTO_SIZE) == 0 ||
	      strncasecmp("_udp", proto, DNS_SD_PROTO_SIZE) == 0)) {
		/* RFC 1034 Section 3.1 */
		LOG_DBG("proto '%s' is invalid (not _tcp or _udp)",
			proto);
		return false;
	}

	return proto_size;
}

static bool domain_is_valid(const char *domain)
{
	size_t domain_size;

	if (domain == NULL) {
		LOG_DBG("label is NULL");
		return false;
	}

	domain_size = strlen(domain);
	if (domain_size < DNS_SD_DOMAIN_MIN_SIZE) {
		LOG_DBG("label '%s' is too small (%zu, min: %u)",
			domain, domain_size, DNS_SD_DOMAIN_MIN_SIZE);
		return false;
	}

	if (domain_size > DNS_SD_DOMAIN_MAX_SIZE) {
		LOG_DBG("label '%s' is too big (%zu, max: %u)",
			domain, domain_size, DNS_SD_DOMAIN_MAX_SIZE);
		return false;
	}

	if (!label_is_valid(domain, domain_size)) {
		LOG_DBG("domain '%s' contains invalid characters",
			domain);
		return false;
	}

	return domain_size;
}

/**
 * Check DNS SD Record for validity
 *
 * Our records are in the form <Instance>.<Service>.<Proto>.<Domain>
 *
 * Currently, <Subdomain>.<Domain> services are not supported.
 */
bool dns_sd_rec_is_valid(const struct dns_sd_rec *inst)
{
	return true
	       && inst != NULL
	       && instance_is_valid(inst->instance)
	       && service_is_valid(inst->service)
	       && proto_is_valid(inst->proto)
	       && domain_is_valid(inst->domain)
	       && inst->text != NULL
	       && inst->port > 0
	;
}

int add_a_record(const struct dns_sd_rec *inst, uint32_t ttl,
		 uint16_t host_offset, uint32_t addr, uint8_t *buf,
		 uint16_t buf_offset, uint16_t buf_size)
{
	uint16_t total_size;
	struct dns_rr *rr;
	struct dns_a_rdata *rdata;
	uint16_t inst_offs;
	uint16_t offset = buf_offset;

	if ((DNS_SD_PTR_MASK & host_offset) != 0) {
		LOG_ERR("offset %u too big for message compression",
			host_offset);
		return -E2BIG;
	}

	/* First, calculate that there is enough space in the buffer */
	total_size =
		/* pointer to .<Instance>.local. */
		2 + sizeof(*rr) + sizeof(*rdata);

	if (offset > buf_size || total_size >= buf_size - offset) {
		LOG_ERR("Buffer too small. required: %u available: %d",
			total_size, (int)buf_size - (int)offset);
		return -ENOSPC;
	}

	/* insert a pointer to the instance + service name */
	inst_offs = host_offset;
	inst_offs |= DNS_SD_PTR_MASK;
	inst_offs = htons(inst_offs);
	memcpy(&buf[offset], &inst_offs, sizeof(inst_offs));
	offset += sizeof(inst_offs);

	rr = (struct dns_rr *)&buf[offset];
	rr->type = htons(DNS_RR_TYPE_A);
	rr->class_ = htons(DNS_CLASS_IN | DNS_CLASS_FLUSH);
	rr->ttl = htonl(ttl);
	rr->rdlength = htons(sizeof(*rdata));
	offset += sizeof(*rr);

	rdata = (struct dns_a_rdata *)&buf[offset];
	rdata->address = htonl(addr);
	offset += sizeof(*rdata);

	__ASSERT_NO_MSG(total_size == offset - buf_offset);

	return offset - buf_offset;
}

int add_ptr_record(const struct dns_sd_rec *inst, uint32_t ttl,
		   uint8_t *buf, uint16_t buf_offset, uint16_t buf_size,
		   uint16_t *service_offset, uint16_t *instance_offset,
		   uint16_t *domain_offset)
{
	uint8_t i;
	int name_size;
	struct dns_rr *rr;
	uint16_t svc_offs;
	uint16_t inst_offs;
	uint16_t dom_offs;
	size_t label_size;
	uint16_t service_proto_size;
	uint16_t offset = buf_offset;
	const char *labels[] = {
		inst->instance,
		inst->service,
		inst->proto,
		inst->domain,
	};

	/* First, ensure that labels and full name are within spec */
	if (!dns_sd_rec_is_valid(inst)) {
		return -EINVAL;
	}

	service_proto_size = dns_sd_service_proto_size(inst);

	/*
	 * Next, calculate that there is enough space in the buffer.
	 *
	 * We require that this is the first time names will appear in the
	 * DNS message. Message Compression is used in subsequent
	 * calculations.
	 *
	 * That is the reason there is an output variable for
	 * service_offset and instance_offset.
	 *
	 * For more information on DNS Message Compression, see
	 * RFC 1035, Section 4.1.4.
	 */
	name_size =
		/* uncompressed. e.g. "._foo._tcp.local." */
		service_proto_size +
		sizeof(*rr)
		/* compressed e.g. .My Foo" followed by (DNS_SD_PTR_MASK | 0x0abc) */
		+ 1 + strlen(inst->instance) + 2;

	if (offset > buf_size || name_size >= buf_size - offset) {
		LOG_ERR("Buffer too small. required: %u available: %d",
			name_size, (int)buf_size - (int)offset);
		return -ENOSPC;
	}

	svc_offs = offset;
	if ((svc_offs & DNS_SD_PTR_MASK) != 0) {
		LOG_ERR("offset %u too big for message compression",
			svc_offs);
		return -E2BIG;
	}

	inst_offs = offset + service_proto_size + sizeof(*rr);
	if ((inst_offs & DNS_SD_PTR_MASK) != 0) {
		LOG_ERR("offset %u too big for message compression",
			inst_offs);
		return -E2BIG;
	}

	dom_offs = offset + service_proto_size - 1 -
		   strlen(inst->domain) - 1;

	/* Finally, write output with confidence that doing so is safe */

	*service_offset = svc_offs;
	*instance_offset = inst_offs;
	*domain_offset = dom_offs;

	/* copy the service name. e.g. "._foo._tcp.local." */
	for (i = 1; i < ARRAY_SIZE(labels); ++i) {
		label_size = strlen(labels[i]);
		buf[offset++] = strlen(labels[i]);
		memcpy(&buf[offset], labels[i], label_size);
		offset += label_size;
		if (i == ARRAY_SIZE(labels) - 1) {
			/* terminator */
			buf[offset++] = '\0';
		}
	}

	__ASSERT_NO_MSG(svc_offs + service_proto_size == offset);

	rr = (struct dns_rr *)&buf[offset];
	rr->type = htons(DNS_RR_TYPE_PTR);
	rr->class_ = htons(DNS_CLASS_IN);
	rr->ttl = htonl(ttl);
	rr->rdlength = htons(
		DNS_LABEL_LEN_SIZE +
		strlen(inst->instance)
		+ DNS_POINTER_SIZE);
	offset += sizeof(*rr);

	__ASSERT_NO_MSG(inst_offs == offset);

	/* copy the instance size, value, and add a pointer */
	label_size = strlen(inst->instance);
	buf[offset++] = label_size;
	memcpy(&buf[offset], inst->instance, label_size);
	offset += label_size;

	svc_offs |= DNS_SD_PTR_MASK;
	svc_offs = htons(svc_offs);
	memcpy(&buf[offset], &svc_offs, sizeof(svc_offs));
	offset += sizeof(svc_offs);

	__ASSERT_NO_MSG(name_size == offset - buf_offset);

	return offset - buf_offset;
}

int add_txt_record(const struct dns_sd_rec *inst, uint32_t ttl,
		   uint16_t instance_offset, uint8_t *buf,
		   uint16_t buf_offset, uint16_t buf_size)
{
	uint16_t total_size;
	struct dns_rr *rr;
	uint16_t inst_offs;
	uint16_t offset = buf_offset;

	if ((DNS_SD_PTR_MASK & instance_offset) != 0) {
		LOG_ERR("offset %u too big for message compression",
			instance_offset);
		return -E2BIG;
	}

	/* First, calculate that there is enough space in the buffer */
	total_size =
		/* pointer to .<Instance>.<Service>.<Protocol>.local. */
		DNS_POINTER_SIZE + sizeof(*rr) + DNS_SD_TXT_SIZE(inst);

	if (offset > buf_size || total_size >= buf_size - offset) {
		LOG_ERR("Buffer too small. required: %u available: %d",
			total_size, (int)buf_size - (int)offset);
		return -ENOSPC;
	}

	/* insert a pointer to the instance + service name */
	inst_offs = instance_offset;
	inst_offs |= DNS_SD_PTR_MASK;
	inst_offs = htons(inst_offs);
	memcpy(&buf[offset], &inst_offs, sizeof(inst_offs));
	offset += sizeof(inst_offs);

	rr = (struct dns_rr *)&buf[offset];
	rr->type = htons(DNS_RR_TYPE_TXT);
	rr->class_ = htons(DNS_CLASS_IN | DNS_CLASS_FLUSH);
	rr->ttl = htonl(ttl);
	rr->rdlength = htons(DNS_SD_TXT_SIZE(inst));
	offset += sizeof(*rr);

	memcpy(&buf[offset], inst->text, DNS_SD_TXT_SIZE(inst));
	offset += DNS_SD_TXT_SIZE(inst);

	__ASSERT_NO_MSG(total_size == offset - buf_offset);

	return offset - buf_offset;
}

int add_aaaa_record(const struct dns_sd_rec *inst, uint32_t ttl,
		    uint16_t host_offset, const uint8_t addr[16],
		    uint8_t *buf, uint16_t buf_offset, uint16_t buf_size)
{
	uint16_t total_size;
	struct dns_rr *rr;
	struct dns_aaaa_rdata *rdata;
	uint16_t inst_offs;
	uint16_t offset = buf_offset;

	if ((DNS_SD_PTR_MASK & host_offset) != 0) {
		LOG_ERR("offset %u too big for message compression",
			host_offset);
		return -E2BIG;
	}

	/* First, calculate that there is enough space in the buffer */
	total_size =
		/* pointer to .<Instance>.local. */
		DNS_POINTER_SIZE + sizeof(*rr) + sizeof(*rdata);

	if (offset > buf_size || total_size >= buf_size - offset) {
		LOG_ERR("Buffer too small. required: %u available: %d",
			total_size, (int)buf_size - (int)offset);
		return -ENOSPC;
	}

	/* insert a pointer to the instance + service name */
	inst_offs = host_offset;
	inst_offs |= DNS_SD_PTR_MASK;
	inst_offs = htons(inst_offs);
	memcpy(&buf[offset], &inst_offs, sizeof(inst_offs));
	offset += sizeof(inst_offs);

	rr = (struct dns_rr *)&buf[offset];
	rr->type = htons(DNS_RR_TYPE_AAAA);
	rr->class_ = htons(DNS_CLASS_IN | DNS_CLASS_FLUSH);
	rr->ttl = htonl(ttl);
	rr->rdlength = htons(sizeof(*rdata));
	offset += sizeof(*rr);

	rdata = (struct dns_aaaa_rdata *)&buf[offset];
	memcpy(rdata->address, addr, sizeof(*rdata));
	offset += sizeof(*rdata);

	__ASSERT_NO_MSG(total_size == offset - buf_offset);

	return offset - buf_offset;
}

int add_srv_record(const struct dns_sd_rec *inst, uint32_t ttl,
		   uint16_t instance_offset, uint16_t domain_offset,
		   uint8_t *buf, uint16_t buf_offset, uint16_t buf_size,
		   uint16_t *host_offset)
{
	uint16_t total_size;
	struct dns_rr *rr;
	struct dns_srv_rdata *rdata;
	size_t label_size;
	uint16_t inst_offs;
	uint16_t offset = buf_offset;

	if ((DNS_SD_PTR_MASK & instance_offset) != 0) {
		LOG_ERR("offset %u too big for message compression",
			instance_offset);
		return -E2BIG;
	}

	if ((DNS_SD_PTR_MASK & domain_offset) != 0) {
		LOG_ERR("offset %u too big for message compression",
			domain_offset);
		return -E2BIG;
	}

	/* First, calculate that there is enough space in the buffer */
	total_size =
		/* pointer to .<Instance>.<Service>.<Protocol>.local. */
		DNS_POINTER_SIZE + sizeof(*rr)
		+ sizeof(*rdata)
		/* .<Instance> */
		+ DNS_LABEL_LEN_SIZE
		+ strlen(inst->instance)
		/* pointer to .local. */
		+ DNS_POINTER_SIZE;

	if (offset > buf_size || total_size >= buf_size - offset) {
		LOG_ERR("Buffer too small. required: %u available: %d",
			total_size, (int)buf_size - (int)offset);
		return -ENOSPC;
	}

	/* insert a pointer to the instance + service name */
	inst_offs = instance_offset;
	inst_offs |= DNS_SD_PTR_MASK;
	inst_offs = htons(inst_offs);
	memcpy(&buf[offset], &inst_offs, sizeof(inst_offs));
	offset += sizeof(inst_offs);

	rr = (struct dns_rr *)&buf[offset];
	rr->type = htons(DNS_RR_TYPE_SRV);
	rr->class_ = htons(DNS_CLASS_IN | DNS_CLASS_FLUSH);
	rr->ttl = htonl(ttl);
	/* .<Instance>.local. */
	rr->rdlength = htons(sizeof(*rdata) + DNS_LABEL_LEN_SIZE
			     + strlen(inst->instance) +
			     DNS_POINTER_SIZE);
	offset += sizeof(*rr);

	rdata = (struct dns_srv_rdata *)&buf[offset];
	rdata->priority = 0;
	rdata->weight = 0;
	rdata->port = htons(inst->port);
	offset += sizeof(*rdata);

	*host_offset = offset;

	label_size = strlen(inst->instance);
	buf[offset++] = label_size;
	memcpy(&buf[offset], inst->instance, label_size);
	offset += label_size;

	domain_offset |= DNS_SD_PTR_MASK;
	domain_offset = htons(domain_offset);
	memcpy(&buf[offset], &domain_offset, sizeof(domain_offset));
	offset += sizeof(domain_offset);

	__ASSERT_NO_MSG(total_size == offset - buf_offset);

	return offset - buf_offset;
}

int dns_sd_handle_ptr_query(const struct dns_sd_rec *inst, bool ipv6,
			    const uint8_t *addr,
			    const struct dns_query *query,
			    uint8_t *buf, uint16_t buf_size)
{
	/*
	 * RFC 6763 Section 12.1
	 *
	 * When including a DNS-SD Service Instance Enumeration or Selective
	 * Instance Enumeration (subtype) PTR record in a response packet, the
	 * server/responder SHOULD include the following additional records:
	 *
	 * o  The SRV record(s) named in the PTR rdata.
	 * o  The TXT record(s) named in the PTR rdata.
	 * o  All address records (type "A" and "AAAA") named in the SRV rdata.
	 *	contain the SRV record(s), the TXT record(s), and the address
	 *      records (A or AAAA)
	 */

	uint16_t instance_offset = -1;
	uint16_t service_offset = -1;
	uint16_t domain_offset = -1;
	uint16_t host_offset = -1;
	uint16_t offset = sizeof(struct dns_header);
	struct dns_header *rsp = (struct dns_header *)buf;
	int r;

	memset(rsp, 0, sizeof(*rsp));

	/* first add the answer record */
	r = add_ptr_record(inst, DNS_SD_PTR_TTL, buf, offset,
			   buf_size - offset,
			   &service_offset, &instance_offset,
			   &domain_offset);
	if (r < 0) {
		return r; /* LCOV_EXCL_LINE */
	}

	rsp->ancount++;
	offset += r;

	/* then add the additional records */
	r = add_txt_record(inst, DNS_SD_TXT_TTL, instance_offset, buf,
			   offset,
			   buf_size - offset);
	if (r < 0) {
		return r; /* LCOV_EXCL_LINE */
	}

	rsp->arcount++;
	offset += r;

	r = add_srv_record(inst, DNS_SD_SRV_TTL, instance_offset,
			   domain_offset,
			   buf, offset, buf_size - offset, &host_offset);
	if (r < 0) {
		return r; /* LCOV_EXCL_LINE */
	}

	rsp->arcount++;
	offset += r;

	if (ipv6) {
		r = add_aaaa_record(inst, DNS_SD_AAAA_TTL, host_offset,
				    addr,
				    buf, offset,
				    buf_size - offset);     /* LCOV_EXCL_LINE */
	} else {
		r = add_a_record(inst, DNS_SD_A_TTL, host_offset,
				 *((uint32_t *)addr), buf, offset,
				 buf_size - offset);
	}

	if (r < 0) {
		return r; /* LCOV_EXCL_LINE */
	}

	rsp->arcount++;
	offset += r;

	rsp->flags = htons(BIT(15) | BIT(10));
	rsp->ancount = htons(rsp->ancount);
	rsp->arcount = htons(rsp->arcount);

	return offset;
}

/* TODO: dns_sd_handle_srv_query() */
/* TODO: dns_sd_handle_txt_query() */

bool dns_sd_rec_match(const struct dns_sd_rec *a, struct dns_sd_rec *b)
{
	#define WILDCARD NULL

	size_t i;
	const char *label_a;
	const char *label_b;

	if (!dns_sd_rec_is_valid(a)) {
		return false;
	}

	if (b == NULL) {
		return false;
	}

	const char *pairs[] = {
		a->instance, b->instance,
		a->service, b->service,
		a->proto, b->proto,
		a->domain, b->domain,
	};

	const bool (*checkers[])(const char *) = {
		&instance_is_valid,
		&service_is_valid,
		&proto_is_valid,
		&domain_is_valid,
	};

	const char *names[] = {
		"instance",
		"service",
		"protocol",
		"domain",
	};

	BUILD_ASSERT(ARRAY_SIZE(pairs) == 2 * ARRAY_SIZE(checkers));
	BUILD_ASSERT(ARRAY_SIZE(names) == ARRAY_SIZE(checkers));

	for (i = 0; i < ARRAY_SIZE(checkers); ++i) {
		label_a = pairs[2 * i];
		label_b = pairs[2 * i + 1];
		if (label_b != WILDCARD) {
			if (!checkers[i](label_b)) {
				LOG_WRN("invalid %s label: '%s'", names[i], label_b);
				return false;
			}

			if (strncasecmp(label_a, label_b,
					DNS_LABEL_MAX_SIZE) != 0) {
				return false;
			}
		}
	}

	if (b->port != 0 && a->port != b->port) {
		return false;
	}

	return true;
}
