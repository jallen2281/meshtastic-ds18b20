#pragma once

#include <stdint.h>
#include <sys/ioctl.h>
#include "bluetooth.h"

struct hci_dev_info {
    int dev_id;
    bdaddr_t bdaddr;
};

#ifndef HCIGETDEVINFO
#define HCIGETDEVINFO 0
#endif
