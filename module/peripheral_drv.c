
#include "peripheral_drv.h"
#include "platform.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"

void gsensor_int_config(void)
{
	nrf_gpio_cfg_input(GSENSOR_INT_PIN, NRF_GPIO_PIN_NOPULL);
}

bool is_gsensor_int(void)
{
	if(nrf_gpio_pin_read(GSENSOR_INT_PIN) > 0) {
		return false;
	} else {
		return true;
	}
}


// #############################################################
// END
// #############################################################
