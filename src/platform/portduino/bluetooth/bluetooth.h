#pragma once

#include <stdint.h>

typedef struct {
    uint8_t b[6];
} bdaddr_t;

#ifndef AF_BLUETOOTH
#define AF_BLUETOOTH 31
#endif
