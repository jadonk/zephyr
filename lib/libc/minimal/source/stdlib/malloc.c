/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr.h>
#include <init.h>
#include <errno.h>
#include <sys/math_extras.h>
#include <sys/mempool.h>
#include <string.h>
#include <app_memory/app_memdomain.h>

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_STATS
#include <malloc.h>
static struct mallinfo malloc_mallinfo = {
	.arena    = CONFIG_MINIMAL_LIBC_MALLOC_ARENA_SIZE,
	.fordblks = CONFIG_MINIMAL_LIBC_MALLOC_ARENA_SIZE,
};
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_STATS */

#define LOG_LEVEL CONFIG_KERNEL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_DECLARE(os);

#if (CONFIG_MINIMAL_LIBC_MALLOC_ARENA_SIZE > 0)
#ifdef CONFIG_USERSPACE
K_APPMEM_PARTITION_DEFINE(z_malloc_partition);
#define POOL_SECTION K_APP_DMEM_SECTION(z_malloc_partition)
#else
#define POOL_SECTION .data
#endif /* CONFIG_USERSPACE */

SYS_MEM_POOL_DEFINE(z_malloc_mem_pool, NULL, 16,
		    CONFIG_MINIMAL_LIBC_MALLOC_ARENA_SIZE, 1, 4, POOL_SECTION);

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
void *z_malloc_debug(const char *file, const char *func, const int line, size_t size)
{
	printk("malloc: %s:%s():%d: size: %u ", file, func, line, (unsigned)size);
#else /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
void *malloc(size_t size)
{
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
	void *ret;

	ret = sys_mem_pool_alloc(&z_malloc_mem_pool, size);
	if (ret == NULL) {
#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
		printk("ENOMEM\n");
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
		errno = ENOMEM;
	}

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_STATS
	struct sys_mem_pool_block *blk;
	size_t struct_blk_size = WB_UP(sizeof(struct sys_mem_pool_block));
	size_t block_size;
	/* Stored right before the pointer passed to the user */
	blk = (struct sys_mem_pool_block *)((char *)ret - struct_blk_size);

	/* Determine size of previously allocated block by its level.
	 * Most likely a bit larger than the original allocation
	 */
	block_size = blk->pool->base.max_sz;
	for (int i = 1; i <= blk->level; i++) {
		block_size = WB_DN(block_size / 4);
	}

	malloc_mallinfo.uordblks += block_size;
	malloc_mallinfo.fordblks -= block_size;
#endif

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
	printk("ret: %p arena: %08x free: %08x used: %08x\n", ret, malloc_mallinfo.arena, malloc_mallinfo.fordblks, malloc_mallinfo.uordblks);
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */

	return ret;
}

static int malloc_prepare(struct device *unused)
{
	ARG_UNUSED(unused);

	sys_mem_pool_init(&z_malloc_mem_pool);

	return 0;
}

SYS_INIT(malloc_prepare, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#else /* No malloc arena */
void *malloc(size_t size)
{
	ARG_UNUSED(size);

	LOG_DBG("CONFIG_MINIMAL_LIBC_MALLOC_ARENA_SIZE is 0");
	errno = ENOMEM;

	return NULL;
}
#endif

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
void z_free_debug(const char *file, const char *func, const int line, void *ptr)
{
	printk("free: %s:%s():%d %p ", file, func, line, ptr);
#else /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
void free(void *ptr)
{
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_STATS
	struct sys_mem_pool_block *blk;
	size_t struct_blk_size = WB_UP(sizeof(struct sys_mem_pool_block));
	size_t block_size;
	/* Stored right before the pointer passed to the user */
	blk = (struct sys_mem_pool_block *)((char *)ptr - struct_blk_size);

	/* Determine size of previously allocated block by its level.
	 * Most likely a bit larger than the original allocation
	 */
	block_size = blk->pool->base.max_sz;
	for (int i = 1; i <= blk->level; i++) {
		block_size = WB_DN(block_size / 4);
	}

	/* The sys_mem_pool API provides no way to validate if a pointer is valid
	 * so we just assume that it is */
	malloc_mallinfo.uordblks -= block_size;
	malloc_mallinfo.fordblks += block_size;
#endif

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
	printk("total: %08x free: %08x used: %08x\n", malloc_mallinfo.arena, malloc_mallinfo.fordblks, malloc_mallinfo.uordblks);
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */

	sys_mem_pool_free(ptr);
}

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
void *z_calloc_debug(const char *file, const char *func, const int line, size_t nmemb, size_t size)
{
	printk("calloc: %s:%s():%d nmemb: %u size: %u\n", file, func, line, (unsigned)nmemb, (unsigned)size);
#else /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
void *calloc(size_t nmemb, size_t size)
{
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
	void *ret;

	if (size_mul_overflow(nmemb, size, &size)) {
		errno = ENOMEM;
		return NULL;
	}

	ret = malloc(size);

	if (ret != NULL) {
		(void)memset(ret, 0, size);
	}

	return ret;
}

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG
void *z_realloc_debug(const char *file, const char *func, const int line, void *ptr, size_t requested_size)
{
	printk("realloc: %s:%s():%d ptr: %p requested_size: %u\n", file, func, line, ptr, (unsigned)requested_size);
#else /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
void *realloc(void *ptr, size_t requested_size)
{
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */
	struct sys_mem_pool_block *blk;
	size_t struct_blk_size = WB_UP(sizeof(struct sys_mem_pool_block));
	size_t block_size, total_requested_size;
	void *new_ptr;

	if (ptr == NULL) {
		return malloc(requested_size);
	}

	if (requested_size == 0) {
		free(ptr);
		return NULL;
	}

	/* Stored right before the pointer passed to the user */
	blk = (struct sys_mem_pool_block *)((char *)ptr - struct_blk_size);

	/* Determine size of previously allocated block by its level.
	 * Most likely a bit larger than the original allocation
	 */
	block_size = blk->pool->base.max_sz;
	for (int i = 1; i <= blk->level; i++) {
		block_size = WB_DN(block_size / 4);
	}

	/* We really need this much memory */
	total_requested_size = requested_size + struct_blk_size;

	if (block_size >= total_requested_size) {
		/* Existing block large enough, nothing to do */
		return ptr;
	}

	new_ptr = malloc(requested_size);
	if (new_ptr == NULL) {
		return NULL;
	}

	memcpy(new_ptr, ptr, block_size - struct_blk_size);
	free(ptr);

	return new_ptr;
}

void *reallocarray(void *ptr, size_t nmemb, size_t size)
{
	if (size_mul_overflow(nmemb, size, &size)) {
		errno = ENOMEM;
		return NULL;
	}
	return realloc(ptr, size);
}

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_STATS
struct mallinfo mallinfo(void) {
	struct mallinfo ret = malloc_mallinfo;
	return ret;
}
#endif /* CONFIG_MINIMAL_LIBC_MALLOC_STATS */
