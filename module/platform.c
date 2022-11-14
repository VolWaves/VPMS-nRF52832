
#define NRF_LOG_MODULE_NAME platform
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"

#include "app_button.h"
#include "app_timer.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"

#include "app_scheduler.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_saadc.h"

#include "nrf_queue.h"
#include "nrf_drv_rng.h"
#include "nrfx_rng.h"

#include "peripheral_drv.h"

#include "bluetooth.h"

#define spi0_wait(); while (!spi0_xfer_done) {}

#include <stdio.h>
void LOG_HEX_RAW_IMP(const uint8_t* array, uint16_t length)
{
	static char buffer[193];
	const uint8_t* pa = array;
	char* pb;
	uint8_t len;	// 当次处理长度
	while(length > 0) {
		pb = buffer;
		if(length > 64) {
			len = 64;
		} else {
			len = length;
		}
		for (uint8_t i = 0; i < len; i++) {
			sprintf(pb, "%02X ", *pa);
			pb += 3;
			pa += 1;
		}
		length -= len;
		*pb = 0;
		LOG_RAW("%s\n", buffer);
	}
}

void io_config(void);

#define USE_FPU
#ifdef USE_FPU
#define FPU_EXCEPTION_MASK 0x0000009F
void FPU_IRQHandler(void)
{
	uint32_t* fpscr = (uint32_t*)(FPU->FPCAR + 0x40);
	(void)__get_FPSCR();
	*fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}
#endif

void delay_ms(uint16_t ms)
{
	nrf_delay_ms(ms);
}

#define MIN(a,b) ((a) < (b) ? (a) : (b))
uint8_t grands_array[32];
void random_seed_init(void)
{
	APP_ERROR_CHECK(nrf_drv_rng_init(NULL));
}

void random_refresh(void)
{
	uint32_t err_code;
	uint8_t  available;

	nrf_drv_rng_bytes_available(&available);
	uint8_t length = MIN(32, available);

	err_code = nrf_drv_rng_rand(grands_array, length);
	APP_ERROR_CHECK(err_code);
}

__WEAK void rtc_8hz_isr(uint8_t tick)
{
	LOG_RAW("(WEAK HANDLER)isr - %d\r\n", tick);
}

__WEAK void rtc_1hz_handler(void)
{
	static uint32_t sec = 0;
	LOG_RAW("(WEAK HANDLER)tick - %d\r\n", sec);
	sec += 1;
}

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	static unsigned char tick = 0;
	if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
	} else if (int_type == NRF_DRV_RTC_INT_TICK) {
		rtc_8hz_isr(tick);
		tick += 1;
		if(tick & 0x8) {
			tick &= 0x7;
			platform_simple_evt_put(rtc_1hz_handler);
		}
	}
}
static void lfclk_config(void)
{
	ret_code_t err_code = nrf_drv_clock_init();
	APP_ERROR_CHECK(err_code);

	nrf_drv_clock_lfclk_request(NULL);
}
static void rtc_config(void)
{
	uint32_t err_code;
	nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
	config.prescaler = RTC_FREQ_TO_PRESCALER(8);
	err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_rtc_tick_enable(&rtc, true);
	nrf_drv_rtc_enable(&rtc);
}
static void app_timer_config(void)
{
	app_timer_init();
}

uint32_t platform_get_systick(void)
{
	// uint32_t cur_tick = nrf_drv_rtc_counter_get(&rtc);
	return app_timer_cnt_get();
}

static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
	ret_code_t err_code;
	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
}

#define REG_POWER_BASE_ADDR	0x40000000
#define RESET_REASON_OFFSET	0x400

uint32_t read_chip_reset_reason(void)
{
	return *(uint32_t*)(REG_POWER_BASE_ADDR + RESET_REASON_OFFSET);
}

uint8_t button_chatter = 0;
uint8_t button_press_time = 0;
void btn_on_uevt_handler(uevt_t* evt)
{
	switch(evt->evt_id) {
		case UEVT_RTC_8HZ:
			if(button_chatter > 0) {
				button_press_time += 1;
				if(button_press_time == 9) {
					uevt_bc_e(UEVT_BTN_LONG);
				}
				if(button_press_time > 0x10) {
					button_press_time = 0x10;
					uevt_bc_e(UEVT_BTN_REPEAT);
				}
			}
			break;
	}
}
void button_handler(uint8_t* data, uint16_t length)
{
	switch(data[1]) {
		case APP_BUTTON_PUSH:
			button_chatter = 2;
			button_press_time = 0;
			uevt_bc_e(UEVT_BTN_DOWN);
			break;
		case APP_BUTTON_RELEASE:
			button_chatter = 0;
			button_press_time = 0;
			uevt_bc_e(UEVT_BTN_RELEASE);
			break;
	}
}

void button_handler_ISR(uint8_t pin_no, uint8_t button_action)
{
	static uint8_t data[2];
	data[0] = pin_no;
	data[1] = button_action;
	platform_evt_put(data, 2, button_handler);
}

bool is_button_release(uint32_t btn)
{
	return (nrf_gpio_pin_read(btn) == (uint32_t)BUTTON_POL - 1) ? true : false;
}

void key_config(void)
{
	uint32_t err;
	static const app_button_cfg_t configs[] = {
		{
			.pin_no = BUTTON1_PIN,
			.active_state = APP_BUTTON_ACTIVE_HIGH,
			.pull_cfg = BUTTON_PULL,
			.button_handler = button_handler_ISR
		},
	};
	err = app_button_init(configs, sizeof(configs) / sizeof(app_button_cfg_t), APP_TIMER_TICKS(20));
	APP_ERROR_CHECK(err);
	err = app_button_enable();
	APP_ERROR_CHECK(err);
	user_event_handler_regist(btn_on_uevt_handler);
}

void leds_config(void)
{
	nrf_gpio_cfg_output(LED_PIN);
	nrf_gpio_pin_set(LED_PIN);
}
void led_on(void)
{
	nrf_gpio_pin_clear(LED_PIN);
}

void led_off(void)
{
	nrf_gpio_pin_set(LED_PIN);
}

void motor_pin_config(void)
{
	nrf_gpio_cfg_output(MOTOR_PIN);
	nrf_gpio_pin_clear(MOTOR_PIN);
}
void motor_on(void)
{
	nrf_gpio_pin_set(MOTOR_PIN);
}
void motor_off(void)
{
	nrf_gpio_pin_clear(MOTOR_PIN);
}

void io_config(void)
{
	// key_config();
	// leds_config();
	// motor_pin_config();
}

nrf_saadc_value_t adc_buffer[8];
static void saadc_event_handler(nrf_drv_saadc_evt_t const* p_evt)
{
	static int16_t p_data[2];
	if (p_evt->type == NRF_DRV_SAADC_EVT_DONE) {
		int16_t voltY = 0;
		int16_t voltX = 0;
		voltY = p_evt->data.done.p_buffer[0];
		voltX = p_evt->data.done.p_buffer[1];
		p_data[0] = voltX;
		p_data[1] = voltY;
		uevt_bc(UEVT_ADC_NEWDATA, p_data);
		LOG_RAW("x=%04d,y=%04d\r\n", voltX, voltY);
	}
}

void adc_config(void)
{
	uint32_t err_code;

	nrf_drv_saadc_config_t saadc_config = {
		.resolution         = (nrf_saadc_resolution_t)NRF_SAADC_RESOLUTION_12BIT,
		.oversample         = (nrf_saadc_oversample_t)NRF_SAADC_OVERSAMPLE_DISABLED,
		.interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,
		.low_power_mode     = 0
	};

	err_code = nrf_drv_saadc_init(&saadc_config, saadc_event_handler);
	APP_ERROR_CHECK(err_code);

	nrf_saadc_channel_config_t config0 = {
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain       = NRF_SAADC_GAIN1,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time   = NRF_SAADC_ACQTIME_15US,
		.mode       = NRF_SAADC_MODE_DIFFERENTIAL,
		.burst      = NRF_SAADC_BURST_ENABLED,
		.pin_p      = NRF_SAADC_INPUT_AIN2,
		.pin_n      = NRF_SAADC_INPUT_AIN3
	};
	err_code = nrf_drv_saadc_channel_init(0, &config0);
	APP_ERROR_CHECK(err_code);

	nrf_saadc_channel_config_t config1 = {
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain       = NRF_SAADC_GAIN1_3,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time   = NRF_SAADC_ACQTIME_15US,
		.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
		.burst      = NRF_SAADC_BURST_ENABLED,
		.pin_p      = NRF_SAADC_INPUT_AIN5,
		.pin_n      = NRF_SAADC_INPUT_DISABLED
	};
	err_code = nrf_drv_saadc_channel_init(1, &config1);
	APP_ERROR_CHECK(err_code);

	// 差分电池电压检测145 = 4.2v, 460 = 3.7v
	nrf_saadc_channel_config_t config2 = {
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain       = NRF_SAADC_GAIN1,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time   = NRF_SAADC_ACQTIME_10US,
		.mode       = NRF_SAADC_MODE_DIFFERENTIAL,
		.burst      = NRF_SAADC_BURST_ENABLED,
		.pin_p      = NRF_SAADC_INPUT_AIN4,
		.pin_n      = NRF_SAADC_INPUT_AIN6
	};
	err_code = nrf_drv_saadc_channel_init(2, &config2);
	APP_ERROR_CHECK(err_code);
}

void adc_start(void)
{
	uint32_t err_code;

	if (!nrf_drv_saadc_is_busy()) {
		err_code = nrf_drv_saadc_buffer_convert(adc_buffer, 3);
		APP_ERROR_CHECK(err_code);

		err_code = nrf_drv_saadc_sample();
		APP_ERROR_CHECK(err_code);
	}
}

int16_t adc_get(uint8_t channel)
{
	// uint32_t err_code;
	static nrf_saadc_value_t adc_value;
	if (!nrf_drv_saadc_is_busy()) {
		nrfx_saadc_sample_convert(channel, &adc_value);
	}
	return (int16_t)adc_value;
}

volatile bool spi0_xfer_done = true;
void spi0_event_handler(nrf_drv_spi_evt_t const* p_event,
                        void*                     p_context)
{
	spi0_xfer_done = true;
}

const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
static uint8_t is_spi0_conf = 0;

void spi0_unconfig(void)
{
	if(is_spi0_conf == 0) {
		return;
	}
	nrf_drv_spi_uninit(&m_spi_master_0);
	nrf_gpio_cfg_default(MOSI_PIN);
	// nrf_gpio_cfg_default(MISO_PIN);		// 未检测到漏电
	nrf_gpio_cfg_default(SCLK_PIN);
	is_spi0_conf = 0;
}

void spi_config_gsensor(void)
{
	if(is_spi0_conf == 10) {
		return;
	}
	spi0_unconfig();

	uint32_t err_code;
	nrf_drv_spi_config_t const config = {
		.sck_pin      = SCLK_PIN,
		.mosi_pin     = MOSI_PIN,
		.miso_pin     = MISO_PIN,
		.ss_pin       = GSENSOR_CS_PIN,
		.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
		.orc          = 0xFF,
		.frequency    = NRF_DRV_SPI_FREQ_4M,
		.mode         = NRF_DRV_SPI_MODE_3,
		.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
	};
	err_code = nrf_drv_spi_init(&m_spi_master_0, &config, spi0_event_handler, NULL);
	APP_ERROR_CHECK(err_code);
	is_spi0_conf = 10;
}

void spi0_trans_advance(uint8_t* wb, uint8_t wlength, uint8_t* rb, uint8_t rlength)
{
	spi0_xfer_done = false;
	nrf_drv_spi_transfer(&m_spi_master_0, wb, wlength, rb, rlength);
	spi0_wait();
}

uint8_t* spi0_write(uint8_t* array, uint8_t length)
{
	static uint8_t buf[0x100];
	spi0_xfer_done = false;
	nrf_drv_spi_transfer(&m_spi_master_0, array, length, buf, length);
	spi0_wait();
	return buf;
}

void spi0_set(uint8_t addr, uint8_t val)
{
	uint8_t a[] = {addr, val};
	spi0_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi_master_0, a, sizeof(a), NULL, 0));
	spi0_wait();
}

uint8_t spi0_get(uint8_t addr)
{
	static uint8_t a[2] = {0xff, 0xff};
	static uint8_t val[2] = {0xff, 0xff};
	a[0] = addr;
	a[1] = 0xFF;
	spi0_xfer_done = false;
	nrf_drv_spi_transfer(&m_spi_master_0, a, sizeof(a), val, 2);
	spi0_wait();
	return val[1];
}

void spi0_read(uint8_t addr, uint8_t length, uint8_t* buf)
{
	static uint8_t a;
	a = addr;
	spi0_xfer_done = false;
	nrf_drv_spi_transfer(&m_spi_master_0, &a, length + 1, buf, length + 1);
	spi0_wait();
}

void factory_error(uint8_t a)
{

}

bool is_factory_pass(void)
{
	return false;
}

void factory_check(void)
{
	platform_powerdown(true);
}

uint8_t self_check(void)
{
	uint8_t a = 0;
	return a;
}

void peripheral_init(void)
{
}

extern void user_init(void);

void platform_init(void)
{
#if NRF_LOG_ENABLED==1
	log_init();
#endif
	// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
	// 在没有烧录bootloader时，必须禁用，否则会报错
	//uint32_t err_code = ble_dfu_buttonless_async_svci_init();
	//APP_ERROR_CHECK(err_code);

	LOG_RAW("RESET=0x%04X\r\n", read_chip_reset_reason());
	APP_SCHED_INIT(32, 32);
	user_event_handler_array_init();
	lfclk_config();
	power_management_init();

	bluetooth_init();
	uevt_bc_e(UEVT_BT_CTL_ADV_ON);

	random_seed_init();
	random_refresh();

	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
	nrf_drv_gpiote_init();
	io_config();
	// adc_config();
	// pwm_config();
#ifdef USE_FPU
	NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_HIGH);
	NVIC_EnableIRQ(FPU_IRQn);
#endif
	// self_check();
	rtc_config();
	app_timer_config();

	peripheral_init();
}

void platform_reboot(void)
{
	sd_nvic_SystemReset();
}

bool is_going_to_shutdown = false;
void platform_powerdown(bool flag)
{
	is_going_to_shutdown = flag;
}

void shutdown_routine(void)
{
	spi0_unconfig();
	nrf_gpio_input_disconnect(SCLK_PIN);
	nrf_gpio_input_disconnect(MOSI_PIN);
	nrf_gpio_input_disconnect(MISO_PIN);
	nrf_gpio_input_disconnect(BUTTON1_PIN);
	// nrf_gpio_cfg_sense_set(BUTTON2_PIN, BUTTON_SENSE);
	// nrf_gpio_cfg_sense_set(BUTTON3_PIN, BUTTON_SENSE);
	nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
	while(1);
}

void platform_scheduler(void)
{
	app_sched_execute();
	if (NRF_LOG_PROCESS() == false) {
		if(is_going_to_shutdown) {
			shutdown_routine();
		}
		nrf_pwr_mgmt_run();
	}
}
