#ifndef _LED_DRV_H_
#define _LED_DRV_H_

#include "platform.h"
void led_on_uevt_handler(uevt_t* evt);
void led_start(const uint8_t* array, uint8_t loop);

#endif
