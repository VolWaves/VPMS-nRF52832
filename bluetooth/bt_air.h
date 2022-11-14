
#ifndef BT_AIR_H
#define BT_AIR_H

#include "platform.h"
#include "uevent.h"

void bt_air_interface(uint8_t* p_data, uint16_t* length);
void bt_air_init(void);

#define UEVT_BT_AIR_BASE (0x1100)
#define UEVT_BT_AIR_TIMESYNC (UEVT_BT_AIR_BASE|0x01)
#define UEVT_BT_AIR_RTSTATU_CHANGE (UEVT_BT_AIR_BASE|0x02)

#endif