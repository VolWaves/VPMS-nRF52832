
#include "led_drv.h"


__WEAK void led_on(void)
{
	LOG_RAW("[WEAK]led_on not defined!!!");
}

__WEAK void led_off(void)
{
	LOG_RAW("[WEAK]led_off not defined!!!");
}

uint8_t led_loop = 0;
uint8_t led_ptr = 0;
uint8_t led_timer = 0;
const uint8_t* led_array;
void led_start(const uint8_t* array, uint8_t loop)
{
	led_loop = loop;
	led_ptr = 0;
	led_array = array;
}


void led_on_uevt_handler(uevt_t* evt)
{
	switch(evt->evt_id) {
		case UEVT_RTC_8HZ:
			if(led_loop > 0) {
				if(led_array[led_ptr] == 0) {
					led_off();
					if(--led_loop > 0) {
						led_ptr = 0;
					}
				} else {
					if(led_timer == 0) {
						if(led_ptr & 0x1) {
							led_off();
						} else {
							led_on();
						}
						led_timer += 1;
						if(led_timer >= led_array[led_ptr]) {
							led_timer = 0;
							led_ptr += 1;
						}
					} else if(led_timer >= led_array[led_ptr]) {
						led_timer = 0;
						led_ptr += 1;
					} else {
						led_timer += 1;
					}
				}
			}
			break;
	}
}
