#include "no_os_alloc.h"

void *no_os_malloc(size_t size)
{
    return malloc(size);
}

void *no_os_calloc(size_t nitems, size_t size)
{
    return calloc(nitems, size);
}

void no_os_free(void *ptr)
{
    free(ptr);
}
