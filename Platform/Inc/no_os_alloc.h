#ifndef RFID_NO_OS_ALLOC_H
#define RFID_NO_OS_ALLOC_H
#include <stddef.h>
void *no_os_malloc(size_t size);
void no_os_free(void *ptr);
#endif
