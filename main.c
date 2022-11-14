
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "uevent.h"

#include "timedate.h"

#include "led_drv.h"
#include "peripheral_drv.h"

#include "app_timer.h"
#include "bluetooth.h"

#include "bt_air.h"

#ifdef CONFIG_NFCT_PINS_AS_GPIOS
	volatile uint32_t UICR_ADDR_0x20C __attribute__((section("uicr_nfc"))) = 0xFFFFFFFE;
#endif

bool is_time_sync = false;
bool is_bt_connect = false;

uint32_t abs_watch_sec = 0;		// 手表经过的绝对秒数

sTIME utcTime = {0, 0, 0};
sDATE utcDate = {0, 0, 0};
sTIME timezone = {0, 0, 0};
uint8_t localWeek = 0;
sTIME localTime = {0};
sDATE localDate = {0};

#define LOG_EVT(EVT) case EVT: LOG_RAW(#EVT "\n"); break;
const uint8_t led_blink[2] = {1, 1};
void log_on_uevt_handler(uevt_t* evt)
{
	static uint32_t sec = 0;
	switch(evt->evt_id) {
			LOG_EVT(UEVT_BT_INIT);
			LOG_EVT(UEVT_BT_ADV_START);
		case UEVT_RTC_1HZ:
			if(!is_time_sync) {
				LOG_RAW("\n[%06d]:", sec);
			} else {
				LOG_RAW("\n[%02d:%02d:%02d]:", localTime.hour, localTime.min, localTime.sec);
			}
			sec += 1;
			break;
		case UEVT_RTC_8HZ:

			break;
	}
}

void shutdown_now(void)
{
	app_timer_stop_all();
	platform_powerdown(true);
}

static float math_ln(float x)
{
	const float ln10 = 2.302585092994;
	float y, ys;
	float ite = 1;
	float output = 0;
	int8_t k = 0;
	while(x > 1) {
		k += 1;
		x /= 10;
	}
	while(x <= 0.1) {
		k -= 1;
		x *= 10;
	}
	y = (x - 1) / (x + 1);
	ys = y * y;
	for (uint8_t i = 0; i < 13; i++) {
		output += ite / (1 + i * 2);
		ite *= ys;
	}
	output *= 2 * y;
	return output + k * ln10;
}

#include "nrf_gpio.h"

void test_handler(uevt_t* evt)
{
	switch(evt->evt_id) {
		case UEVT_BTN_LONG:
			// shutdown_now();
			break;
		case UEVT_BTN_DOWN:
			break;
		case UEVT_ADC_NEWDATA:
			break;
	}
}

void main_handler(uevt_t* evt)
{
	static uint16_t days;
	switch(evt->evt_id) {
		case UEVT_RTC_1HZ:
			random_refresh();
			if(is_time_sync) {
				if(timeIncSec(&localTime) == DAY) {
					dateInc(&localDate, DAY);
					uevt_bc_e(UEVT_RTC_NEWDAY);
				}
				if(localTime.sec == 0) {
					if(days != localDate.day + localDate.month * 32) {
						days = localDate.day + localDate.month * 32;
						localWeek = getWeekday(localDate.year, localDate.month, localDate.day);
					}
					local2utc(&utcTime, &utcDate, &timezone, &localTime, &localDate);
				}
			}
			break;
		case UEVT_RTC_8HZ:
			break;
		case UEVT_BT_DATARECV:
			bt_air_interface(((ble_user_data_t*)(evt->content))->p_data, ((ble_user_data_t*)(evt->content))->p_length);
			break;
		case UEVT_BT_CONN:
			is_bt_connect = true;
			break;
		case UEVT_BT_DISCONN:
			is_bt_connect = false;
			break;
	}
}

// APP_TIMER_DEF(ADC_TIMER);

void adc_25hz_handler(void* p_context)
{
	static int16_t p_data;
	int16_t volt = adc_get(0);
	p_data = volt;
	uevt_bc(UEVT_ADC_NEWDATA, &p_data);
}

void rtc_1hz_handler(void)
{
	uevt_bc_e(UEVT_RTC_1HZ);
}

void rtc_8hz_isr(uint8_t tick)
{
	uevt_bc_e(UEVT_RTC_8HZ);
}

#include "steps.h"
void gsensor_data_handler(int16_t* data)
{
	static uint8_t ctrl = 1;
	if(is_time_sync) {
		int8_t step = calcStep(data, ctrl);
		if(step > 0) {
			step_add(step);
			ctrl = 0;
		} else if(step < 0) {
			ctrl = 1;
		}
	}
}

#include "mc3632.h"
#include "peripheral_drv.h"
void user_init(void)
{
	app_timer_init();
	// app_timer_create(&ADC_TIMER, APP_TIMER_MODE_REPEATED, adc_25hz_handler);
	// app_timer_start(ADC_TIMER, APP_TIMER_TICKS(40), NULL);
	user_event_handler_regist(log_on_uevt_handler);
	user_event_handler_regist(main_handler);
	user_event_handler_regist(test_handler);
	bt_air_init();
	// if(mc3632_test()) {
	// 	LOG_RAW("[PASS] Gsensor selfcheck\n");
	// 	gsensor_int_config();
	// 	mc3632_init(false);
	// } else {
	// 	LOG_RAW("[NG] Gsensor selfcheck\n");
	// }
}

int main(void)
{
	platform_init();
	user_init();

	LOG_RAW("RTT Started.\n");

	for (;;) {
		platform_scheduler();
	}
}


#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_util_platform.h"
#include "nrf_strerror.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	#include "nrf_sdm.h"
#endif
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	__disable_irq();
	NRF_LOG_FINAL_FLUSH();

#ifndef DEBUG
	NRF_LOG_ERROR("Fatal error");
#else
	switch (id) {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
		case NRF_FAULT_ID_SD_ASSERT:
			NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
			break;
		case NRF_FAULT_ID_APP_MEMACC:
			NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
			break;
#endif
		case NRF_FAULT_ID_SDK_ASSERT: {
			assert_info_t* p_info = (assert_info_t*)info;
			NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
			              p_info->p_file_name,
			              p_info->line_num);
			break;
		}
		case NRF_FAULT_ID_SDK_ERROR: {
			error_info_t* p_info = (error_info_t*)info;
			NRF_LOG_ERROR("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x",
			              p_info->err_code,
			              nrf_strerror_get(p_info->err_code),
			              p_info->p_file_name,
			              p_info->line_num,
			              pc);
			NRF_LOG_ERROR("End of error report");
			break;
		}
		default:
			NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
			break;
	}
#endif

	// On assert, the system can only recover with a reset.

#ifndef DEBUG
	NRF_LOG_WARNING("System reset");
	NVIC_SystemReset();
#else
	app_error_save_and_stop(id, pc, info);
#endif // DEBUG
	NRF_BREAKPOINT_COND;
}
