#ifndef MALLOC_CHECK_H_
#define MALLOC_CHECK_H_

#include <cstdio>

#define ALLOC_CHECK(ptr)                                                      \
{                                                                             \
  if (ptr == NULL) {                                                          \
    printf("Could not allocate memory: %s:%d\n", __FILE__, __LINE__);         \
    exit(1);
  }                                                                           \
}

#endif
