
#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_
#include "uevent.h"

void bt_on_uevt_handler(uevt_t* evt);
void bluetooth_init(void);

// 反馈信号
#define UEVT_BT_BASE (0x1000)
#define UEVT_BT_INIT (UEVT_BT_BASE|0x01)
#define UEVT_BT_DATARECV (UEVT_BT_BASE|0x02)
#define UEVT_BT_ADV_START (UEVT_BT_BASE|0x03)
#define UEVT_BT_ADV_STOP (UEVT_BT_BASE|0x04)
#define UEVT_BT_CONN (UEVT_BT_BASE|0x05)
#define UEVT_BT_DISCONN (UEVT_BT_BASE|0x06)
#define UEVT_BT_RSSI (UEVT_BT_BASE|0x07)

// 控制信号
#define UEVT_BT_CTL_INIT (UEVT_BT_BASE|0x81)
#define UEVT_BT_CTL_ADV_ON (UEVT_BT_BASE|0x82)
#define UEVT_BT_CTL_ADV_OFF (UEVT_BT_BASE|0x83)
#define UEVT_BT_CTL_DISCONN (UEVT_BT_BASE|0x84)
#define UEVT_BT_CTL_DATASEND (UEVT_BT_BASE|0x85)

#define UEVT_BT_CTL_RSSI_GET (UEVT_BT_BASE|0x86)
#define UEVT_BT_CTL_RSSI_START (UEVT_BT_BASE|0x87)
#define UEVT_BT_CTL_RSSI_STOP (UEVT_BT_BASE|0x88)

#define UEVT_BT_CTL_SCAN_ON (UEVT_BT_BASE|0x89)
#define UEVT_BT_CTL_SCAN_OFF (UEVT_BT_BASE|0x8A)
#define UEVT_BT_CTL_CENTRAL_DATASEND (UEVT_BT_BASE|0x8B)


// data struct for UEVT_BT_DATARECV|UEVT_BT_CTL_DATASEND|UEVT_BT_CTL_CENTRAL_DATASEND
typedef struct {
	uint8_t* p_data;
	uint16_t* p_length;
} ble_user_data_t;
// data struct for UEVT_BT_RSSI
typedef struct {
	int8_t rssi;
	uint8_t ch;
} ble_rssi_t;
// data struct for UEVT_BT_CTL_RSSI_START

#endif
