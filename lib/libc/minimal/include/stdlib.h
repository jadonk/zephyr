/* stdlib.h */

/*
 * Copyright (c) 2011-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_STDLIB_H_
#define ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_STDLIB_H_

#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

unsigned long int strtoul(const char *str, char **endptr, int base);
long int strtol(const char *str, char **endptr, int base);
int atoi(const char *s);

#ifdef CONFIG_MINIMAL_LIBC_MALLOC_DEBUG

void *z_malloc_debug(const char *file, const char *func, const int line, size_t size);
void z_free_debug(const char *file, const char *func, const int line, void *ptr);
void *z_calloc_debug(const char *file, const char *func, const int line, size_t nmemb, size_t size);
void *z_realloc_debug(const char *file, const char *func, const int line, void *ptr, size_t size);

#define malloc(size) z_malloc_debug(__FILE__, __func__, __LINE__, size)
#define free(ptr) z_free_debug(__FILE__, __func__, __LINE__, ptr)
#define calloc(nmemb, size) z_calloc_debug(__FILE__, __func__, __LINE__, nmemb, size)
#define realloc(ptr, size) z_realloc_debug(__FILE__, __func__, __LINE__, ptr, size)

#else /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */

void *malloc(size_t size);
void free(void *ptr);
void *calloc(size_t nmemb, size_t size);
void *realloc(void *ptr, size_t size);

#endif /* CONFIG_MINIMAL_LIBC_MALLOC_DEBUG */

void *reallocarray(void *ptr, size_t nmemb, size_t size);

void *bsearch(const void *key, const void *array,
	      size_t count, size_t size,
	      int (*cmp)(const void *key, const void *element));

void qsort(void *base, size_t nmemb, size_t size,
           int(*compar)(const void *, const void *));

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
void _exit(int status);
static inline void exit(int status)
{
	_exit(status);
}

int rand(void);

#define abs(x) ((x) < 0 ? -(x) : (x))

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_STDLIB_H_ */
