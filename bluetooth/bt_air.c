
#include "bt_air.h"
#include "nrf_log.h"
#include "bluetooth.h"
#include "timedate.h"
#include "build.h"


#define INT2CHAR(x) (x)&0xFF,(x)>>8
uint16_t ble_send_public_stack_depth = 0;
uint8_t ble_send_public_stack[256];
ble_user_data_t ble_send_public_data;
uint8_t gflowcontrol = 0;

#define ble_stack_init() ble_send_public_stack_depth=0
#define ble_stack_push(c) ble_send_public_stack[ble_send_public_stack_depth++]=c
#define ble_stack_push_uint16(u16) \
	ble_send_public_stack[ble_send_public_stack_depth++]=u16&0xFF;\
	ble_send_public_stack[ble_send_public_stack_depth++]=u16>>8
#define ble_stack_push_uint32(u32) \
	ble_send_public_stack[ble_send_public_stack_depth++]=u32&0xFF;\
	ble_send_public_stack[ble_send_public_stack_depth++]=(u32>>8)&0xFF;\
	ble_send_public_stack[ble_send_public_stack_depth++]=(u32>>16)&0xFF;\
	ble_send_public_stack[ble_send_public_stack_depth++]=(u32>>24)&0xFF
#define ble_stack_send() do{\
		ble_send_public_data.p_data=ble_send_public_stack;\
		ble_send_public_data.p_length=&ble_send_public_stack_depth;\
		uevt_bc(UEVT_BT_CTL_DATASEND, &ble_send_public_data);\
	}while(0)

#define ble_send(arr...) do{\
		const uint8_t t[]={arr};\
		const uint8_t len = sizeof(t);\
		ble_stack_init();\
		for (uint8_t i = 0; i < len; i++) { ble_stack_push(t[i]); }\
		ble_stack_send();\
		LOG_RAW("Tx:");\
		LOG_HEX_RAW(t, sizeof(t));\
	}while(0)

#define ack() ble_send(a[0],0)
#define err(x) ble_send(a[0],3,0xFF,INT2CHAR(x));LOG_RAW("ret ERR:%d\n",x)

#define ble_send_str(v1,v2,str) do{\
		const uint8_t t[]=str;\
		ble_stack_init();\
		ble_stack_push(a[0]);\
		ble_stack_push(sizeof(t)+2);\
		ble_stack_push(v1);\
		ble_stack_push(v2);\
		for (uint8_t i = 0; i < sizeof(t); i++) {\
			ble_stack_push(t[i]);\
		}\
		ble_stack_push(0);\
		ble_stack_send();\
		LOG_RAW("Tx:");\
		LOG_HEX_RAW(ble_send_public_stack, ble_send_public_stack_depth);\
	}while(0)

extern sTIME utcTime;
extern sDATE utcDate;
extern uint8_t localWeek;
extern sTIME localTime;
extern sDATE localDate;
extern sTIME timezone;
void timedate_interface(uint8_t* a)
{
	switch (a[3]) {
		case 0x01:
			// GET
			LOG_RAW("Get time\n");
			local2utc(&utcTime, &utcDate, &timezone, &localTime, &localDate);
			ble_send(a[0], 10, 0x01, 0x01,
			         utcDate.year, utcDate.month, utcDate.day,
			         utcTime.hour, utcTime.min, utcTime.sec,
			         timezone.hour, timezone.min);
			break;
		case 0x02:
			// SET
			LOG_RAW("Set time\n");
			utcDate.year = a[4];
			utcDate.month = a[5];
			utcDate.day = a[6];
			utcTime.hour = a[7];
			utcTime.min = a[8];
			utcTime.sec = a[9];
			timezone.hour = a[10];
			timezone.min = a[11];
			utc2local(&utcTime, &utcDate, &timezone, &localTime, &localDate);
			dateFix(&localDate);
			localWeek = getWeekday(localDate.year, localDate.month, localDate.day);
			uevt_bc_e(UEVT_BT_AIR_TIMESYNC);
			ack();
			break;

		default:
			err(405);
			break;
	}
}

#include "ble_gap.h"
extern ble_gap_addr_t bluetooth_addr;
void sysinfo_interface(uint8_t* a)
{
	switch (a[3]) {
		default:
			err(405);
			break;
		case 0x10:
			switch(a[4]) {
				case 0x00:
					LOG_RAW("GET P_NAME\n");
					ble_send_str(0x06, 0x10, str_project);
					break;
				case 0x01:
					LOG_RAW("GET B_NAME\n");
					ble_send_str(0x06, 0x10, str_branch);
					break;
				default:
					err(406);
					break;
			}
			break;
		case 0x04:	// 广播名称
			do {
				if(a[4] == 0x00) {	// 获取
					LOG_RAW("GET NAME\n");
					ble_send_str(0x06, 0x04, DEVICE_NAME);
				}
			} while(0);
			break;
		case 0x05:	// MAC地址
			do {
				if(a[4] == 0x00) {	// 获取
					LOG_RAW("GET MAC ADDR\n");
					ble_send(a[0], 6 + 2, 0x06, 0x05,
					         bluetooth_addr.addr[5],
					         bluetooth_addr.addr[4],
					         bluetooth_addr.addr[3],
					         bluetooth_addr.addr[2],
					         bluetooth_addr.addr[1],
					         bluetooth_addr.addr[0] );
				}
			} while(0);
			break;
		case 0x21:
			LOG_RAW("GET device id\r\n");
			ble_send(a[0], 3, 0x06, 0x21, DEVICE_IDN);
			break;
	}
}

void data_interface(uint8_t* a)
{
	switch (a[3]) {
		default:
			err(405);
			break;
		case 0x01:
			LOG_RAW("GET STEP\n");
			break;
		case 0x02:
			LOG_RAW("GET HRM\n");
			break;
		case 0x03:
			LOG_RAW("GET SpO2\n");
			break;
		case 0x04:
			LOG_RAW("GET BP\n");
			break;
		case 0x05:
			LOG_RAW("GET temp\n");
			break;
	}
}

void bt_air_interface(uint8_t* a, uint16_t* length)
{
	if(a[1] + 2 > *length) {
		err(406);
		return;
	}
	if((utcDate.month == 0) && ((a[2] != 0x01) || (a[3] != 0x02))) {
		err(401);
		return;
	}
	switch (a[2]) {
		case 0x01:
			// 时间日期
			timedate_interface(a);
			break;
		case 0x06:
			// 系统信息
			sysinfo_interface(a);
			break;
		case 0x08:
			// 数据交互
			data_interface(a);
			break;

		default:
			break;
	}
}

void bt_air_handler(uevt_t* evt)
{
	switch(evt->evt_id) {
		case UEVT_RTC_8HZ:
			break;
	}
}

void bt_air_init(void)
{
	user_event_handler_regist(bt_air_handler);
}
