
#include "mc3632.h"
#include "nrf_delay.h"

#include "platform.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "peripheral_drv.h"

#define mc3632_w1(x,y) mc3632_w1a(x,y)
#define mc3632_read_log(reg)

void mc3632_w1a(unsigned char addr, unsigned char data)
{
	addr &= 0x7F;
	spi0_set(MC3632_WRITE(addr), data);
}

unsigned char mc3632_r1(unsigned char addr)
{
	unsigned char ret;
	ret = spi0_get(MC3632_READ(addr));
	return ret;
}

void mc3632_read_all(void)
{
	uint8_t rd[7];
	uint8_t i;
	spi0_unconfig();
	spi_config_gsensor();
	mc3632_w1(0x09, 0x00);
	for (i = 0; i < 25; ++i) {
		spi0_read(MC3632_READ(0x02), 6, rd);
		gsensor_data_handler((int16_t*)(rd + 1));
	}
	//mc3632_w1(0x16, 0x59|0x80);
}

void mc3632_int_handler(void)
{
	// nrf_drv_gpiote_in_event_disable(pin);
	mc3632_read_all();
}

// void mc3632_int_ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
// {
// NRF_LOG_RAW_INFO("mc3632_int-->\r\n");
// platform_simple_evt_put(mc3632_int_handler);
// }

void mc3632_int_scan(void)
{
	platform_simple_evt_put(mc3632_int_handler);
}

bool mc3632_test(void)
{
	uint8_t data;
	spi0_unconfig();
	spi_config_gsensor();
	mc3632_w1(0x1B, 0xE3);
	data = mc3632_r1(0x1B);
	if(data == 0xE3) {
		return true;
	} else {
		return false;
	}
}

void mc3632_on_uevt_handler(uevt_t* evt)
{
	switch (evt->evt_id) {
		case UEVT_RTC_8HZ:
			if(is_gsensor_int()) {
				// LOG_RAW("gsensor int\n");
				mc3632_int_scan();
			}
			break;
	}
}

void mc3632_init(bool isSleep)
{
	uint8_t buf[6];
	uint8_t data;
	spi0_unconfig();
	spi_config_gsensor();

	// mc3632_w1(0x24, 0x80);	//reset
	// nrf_delay_ms(100);

	data = mc3632_r1(0x18);
	NRF_LOG_RAW_INFO("[0x18]=0x%02X\r\n", data);

	mc3632_w1(0x10, 0x01);	//standby mode
	mc3632_read_log(0x10);
	nrf_delay_ms(2);

	data = mc3632_r1(0x0D);
	data &= 0x3F;
	data |= 0x80;
	mc3632_w1(0x0D, data);
	mc3632_read_log(0x0D);

	mc3632_w1(0x0F, 0x42);	//Software must write a value of 0x42

	data = mc3632_r1(0x0E);
	data |= 0x20;
	mc3632_w1(0x0E, data);	// enable stream mode
	mc3632_read_log(0x0E);

	// mc3632_w1(0x20, 0x00);
	// mc3632_read_log(0x20);
	// mc3632_w1(0x21, 0x80);
	// mc3632_read_log(0x21);
	mc3632_w1(0x28, 0x00);
	mc3632_read_log(0x28);
	mc3632_w1(0x1A, 0x00);
	mc3632_read_log(0x1A);

	mc3632_w1(0x1B, 0xE3);
	data = mc3632_r1(0x1B);
	NRF_LOG_RAW_INFO("[0x1B]=0x%02X\r\n", data);

	// mc3632_w1(0x14, 0x01);
	// mc3632_w1(0x13, 0x02);	//Sniff Threshold X
	// mc3632_w1(0x14, 0x02);
	// mc3632_w1(0x13, 0x02);	//Sniff Threshold Y
	// mc3632_w1(0x14, 0x03);
	// mc3632_w1(0x13, 0x02);	//Sniff Threshold Z

	mc3632_w1(0x11, 0x06);	//cwake mode 25hz
	mc3632_read_log(0x11);
	// mc3632_w1(0x11, 0x07);	//cwake mode 50hz
	mc3632_w1(0x12, 0x00);	//sniff mode 6Hz
	mc3632_read_log(0x12);
	mc3632_w1(0x15, 0x12);	//Range and Resolution      4G 8bit
	mc3632_read_log(0x15);
	mc3632_w1(0x16, 0x59);	//FIFO Control Register   enable fifo TH=25
	mc3632_read_log(0x16);
	mc3632_w1(0x17, 0x41);	//FIFO TH int	push-pull int output
	// mc3632_w1(0x17, 0x20);	//FIFO FULL int
	mc3632_read_log(0x17);
	mc3632_w1(0x1C, 0xB3);	//Ultra-Low Power Mode
	mc3632_read_log(0x1C);

	mc3632_w1(0x20, 0x01);	//_M_DRV_MC36XX_SetWakeAGAIN
	data = mc3632_r1(0x21);
	data &= 0x3F;
	data |= (2 << 6);
	mc3632_w1(0x21, data);

	mc3632_w1(0x20, 0x00);	//_M_DRV_MC36XX_SetSniffAGAIN
	mc3632_w1(0x21, 0x00);
	mc3632_w1(0x21, 0x80);

	if(!isSleep) {
		mc3632_w1(0x10, 0x05);	//sniff mode 0x02;    cwake mode 0x05  ;swake mode 0x06
	} else {
		mc3632_w1(0x10, 0x00);	//sniff mode 0x02;    cwake mode 0x05  ;swake mode 0x06
	}
	mc3632_read_log(0x10);

	// mc3632_w1(0x16, 0x59|0x80);
	mc3632_w1(0x09, 0x00);	// int clear

	for (data = 0; data < 32; ++data) {
		spi0_read(MC3632_READ(0x02), 6, buf);
	}
	user_event_handler_regist(mc3632_on_uevt_handler);
}
