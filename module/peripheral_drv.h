
#ifndef _PERIPHERAL_DRIVER_H_
#define _PERIPHERAL_DRIVER_H_
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "uevent.h"

void gsensor_int_config(void);
bool is_gsensor_int(void);

#endif
