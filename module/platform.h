
#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#include <stdint.h>
#include <string.h>

#include "app_scheduler.h"
#define platform_simple_evt_put(handler) APP_ERROR_CHECK(app_sched_event_put(NULL,0,(app_sched_event_handler_t)handler))
#define platform_evt_put(data,size,handler) APP_ERROR_CHECK(app_sched_event_put(data,size,(app_sched_event_handler_t)handler))

#define LOG_INIT(x) NRF_LOG_MODULE_REGISTER();

#if NRF_LOG_ENABLED==1
	#define LOG_HEX_RAW LOG_HEX_RAW_IMP
	#define LOG_RAW NRF_LOG_RAW_INFO
#else
	#define LOG_HEX_RAW(x...)
	#define LOG_RAW(x...)
#endif

void LOG_HEX_RAW_IMP(const uint8_t* array, uint16_t length);

#define MOTOR_PIN (26)
#define LED_PIN (17)
#define BUTTON1_PIN (5)

#define BUTTON_POL (NRF_GPIOTE_POLARITY_LOTOHI)
#define BUTTON_PULL (nrf_gpio_pin_pull_t)(BUTTON_POL*2-1)
#define BUTTON_SENSE (nrf_gpio_pin_sense_t)(BUTTON_POL+1)

#define SCLK_PIN (11)
#define MOSI_PIN (12)
#define MISO_PIN (13)

#define GSENSOR_INT_PIN (6)
#define GSENSOR_CS_PIN (7)

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "uevent.h"

void platform_init(void);
void platform_scheduler(void);

void platform_reboot(void);
void platform_powerdown(bool flag);

void btn_on_uevt_handler(uevt_t* evt);

void spi_config_oled(void);
void spi0_unconfig(void);
void spi0_trans_advance(uint8_t* wb, uint8_t wlength, uint8_t* rb, uint8_t rlength);
uint8_t* spi0_write(uint8_t* array, uint8_t length);
void spi0_set(uint8_t addr, uint8_t val);
uint8_t spi0_get(uint8_t addr);
void spi0_read(uint8_t addr, uint8_t length, uint8_t* buf);

void adc_config(void);
void adc_start(void);
int16_t adc_get(uint8_t channel);

void motor_on(void);
void motor_off(void);

void random_refresh(void);
uint32_t platform_get_systick(void);


#define UEVT_RTC_BASE (0x0000)
#define UEVT_RTC_8HZ (0x0001)
#define UEVT_RTC_1HZ (0x0002)
#define UEVT_RTC_NEWDAY (0x0003)

#define UEVT_BTN_BASE (0x0100)
#define UEVT_BTN_DOWN (0x0101)
#define UEVT_BTN_RELEASE (0x0102)
#define UEVT_BTN_LONG (0x0103)
#define UEVT_BTN_REPEAT (0x0104)

#define UEVT_ADC_BASE (0x0200)
#define UEVT_ADC_INIT (UEVT_ADC_BASE|0x01)
#define UEVT_ADC_NEWDATA (UEVT_ADC_BASE|0x02)
#define UEVT_ADC_NEWDATA_FL (UEVT_ADC_BASE|0x03)

#define UEVT_DTIME_BASE (0x0500)
#define UEVT_DTIME_UPDATE (UEVT_DTIME_BASE|0x01)

#endif
