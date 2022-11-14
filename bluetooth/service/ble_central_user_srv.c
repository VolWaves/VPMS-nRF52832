
#include "sdk_common.h"
#include "ble_central_user_srv.h"
#include "ble_srv_common.h"
#include "ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define BLE_USRV_MAX_RX_CHAR_LEN        BLE_USRV_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_USRV_MAX_TX_CHAR_LEN        BLE_USRV_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

#define USRV_BASE_UUID                  {{0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */

void ble_user_on_db_disc_evt(ble_central_user_t* p_ble_user, ble_db_discovery_evt_t* p_evt)
{
	ble_central_user_evt_t user_evt;
	memset(&user_evt, 0, sizeof(ble_central_user_evt_t));

	ble_gatt_db_char_t* p_chars = p_evt->params.discovered_db.charateristics;

	// Check if the was discovered.
	if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
	        &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_user->uuid_type)) {
		NRF_LOG_INFO("db_disc_evt: type:%02X,uuid:%04X,base:%02X",
		             p_evt->evt_type,
		             p_evt->params.discovered_db.srv_uuid.uuid,
		             p_evt->params.discovered_db.srv_uuid.type);
		for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++) {
			NRF_LOG_INFO("char:%04X",
			             p_chars[i].characteristic.uuid.uuid);
			switch (p_chars[i].characteristic.uuid.uuid) {
				case BLE_UUID_USRV_RX_CHARACTERISTIC:
					user_evt.handles.user_rx_handle = p_chars[i].characteristic.handle_value;
					break;

				case BLE_UUID_USRV_TX_CHARACTERISTIC:
					user_evt.handles.user_tx_handle = p_chars[i].characteristic.handle_value;
					user_evt.handles.user_tx_cccd_handle = p_chars[i].cccd_handle;
					break;

				default:
					break;
			}
		}
		if (p_ble_user->evt_handler != NULL) {
			user_evt.conn_handle = p_evt->conn_handle;
			user_evt.evt_type    = BLE_USER_EVT_DISCOVERY_COMPLETE;
			p_ble_user->evt_handler(p_ble_user, &user_evt);
		}
	}
}

static void on_hvx(ble_central_user_t* p_ble_user, ble_evt_t const* p_ble_evt)
{
	// HVX can only occur from client sending.
	if (   (p_ble_user->handles.user_tx_handle != BLE_GATT_HANDLE_INVALID)
	        && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_user->handles.user_tx_handle)
	        && (p_ble_user->evt_handler != NULL)) {
		ble_central_user_evt_t ble_user_evt;

		ble_user_evt.evt_type = BLE_USER_EVT_TX_EVT;
		ble_user_evt.p_data   = (uint8_t*)p_ble_evt->evt.gattc_evt.params.hvx.data;
		ble_user_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

		p_ble_user->evt_handler(p_ble_user, &ble_user_evt);
		// NRF_LOG_DEBUG("Client sending data.");
	}
}

static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
	uint8_t buf[BLE_CCCD_VALUE_LEN];

	buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
	buf[1] = 0;

	ble_gattc_write_params_t const write_params = {
		.write_op = BLE_GATT_OP_WRITE_REQ,
		.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
		.handle   = cccd_handle,
		.offset   = 0,
		.len      = sizeof(buf),
		.p_value  = buf
	};

	return sd_ble_gattc_write(conn_handle, &write_params);
}


uint32_t ble_user_tx_notif_enable(ble_central_user_t* p_ble_user)
{
	VERIFY_PARAM_NOT_NULL(p_ble_user);

	if ( (p_ble_user->conn_handle == BLE_CONN_HANDLE_INVALID)
	        || (p_ble_user->handles.user_tx_cccd_handle == BLE_GATT_HANDLE_INVALID)
	   ) {
		return NRF_ERROR_INVALID_STATE;
	}
	return cccd_configure(p_ble_user->conn_handle, p_ble_user->handles.user_tx_cccd_handle, true);
}
uint32_t ble_user_handles_assign(ble_central_user_t*                p_ble_user,
                                 uint16_t                    conn_handle,
                                 ble_central_user_handles_t const* p_peer_handles)
{
	VERIFY_PARAM_NOT_NULL(p_ble_user);

	p_ble_user->conn_handle = conn_handle;
	if (p_peer_handles != NULL) {
		p_ble_user->handles.user_tx_cccd_handle = p_peer_handles->user_tx_cccd_handle;
		p_ble_user->handles.user_tx_handle      = p_peer_handles->user_tx_handle;
		p_ble_user->handles.user_rx_handle      = p_peer_handles->user_rx_handle;
	}
	return NRF_SUCCESS;
}





static void on_connect(ble_central_user_t* p_user, ble_evt_t* p_ble_evt)
{
	p_user->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


static void on_disconnect(ble_central_user_t* p_user, ble_evt_t* p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_user->conn_handle = BLE_CONN_HANDLE_INVALID;
}


static void on_write(ble_central_user_t* p_user, ble_evt_t* p_ble_evt)
{
	ble_gatts_evt_write_t* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (
	    (p_evt_write->handle == p_user->tx_handles.cccd_handle)
	    &&
	    (p_evt_write->len == 2)
	) {
		if (ble_srv_is_notification_enabled(p_evt_write->data)) {
			p_user->is_notification_enabled = true;
		} else {
			p_user->is_notification_enabled = false;
		}
	} else if (
	    (p_evt_write->handle == p_user->rx_handles.value_handle)
	    &&
	    (p_user->data_handler != NULL)
	) {
		p_user->data_handler(p_user, p_evt_write->data, p_evt_write->len);
	} else {
		// Do Nothing. This event is not relevant for this service.
	}
}

static uint32_t tx_char_add(ble_central_user_t* p_user, const ble_central_user_init_t* p_user_init)
{
	/**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

	ble_uuid.type = p_user->uuid_type;
	ble_uuid.uuid = BLE_UUID_USRV_TX_CHARACTERISTIC;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	attr_md.vloc    = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen    = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = sizeof(uint8_t);
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = BLE_USRV_MAX_TX_CHAR_LEN;

	return sd_ble_gatts_characteristic_add(p_user->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_user->tx_handles);
	/**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

static uint32_t rx_char_add(ble_central_user_t* p_user, const ble_central_user_init_t* p_user_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write         = 1;
	char_md.char_props.write_wo_resp = 1;
	char_md.p_char_user_desc         = NULL;
	char_md.p_char_pf                = NULL;
	char_md.p_user_desc_md           = NULL;
	char_md.p_cccd_md                = NULL;
	char_md.p_sccd_md                = NULL;

	ble_uuid.type = p_user->uuid_type;
	ble_uuid.uuid = BLE_UUID_USRV_RX_CHARACTERISTIC;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	attr_md.vloc    = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen    = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = BLE_USRV_MAX_RX_CHAR_LEN;

	return sd_ble_gatts_characteristic_add(p_user->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_user->rx_handles);
}

void ble_central_user_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context)
{
	ble_central_user_t* p_ble_user = (ble_central_user_t*)p_context;

	if ((p_ble_user == NULL) || (p_ble_evt == NULL)) {
		return;
	}

	if ( (p_ble_user->conn_handle != BLE_CONN_HANDLE_INVALID)
	        && (p_ble_user->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
	   ) {
		return;
	}

	switch (p_ble_evt->header.evt_id) {
		case BLE_GATTC_EVT_HVX:
			on_hvx(p_ble_user, p_ble_evt);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_user->conn_handle
			        && p_ble_user->evt_handler != NULL) {
				ble_central_user_evt_t user_evt;

				user_evt.evt_type = BLE_USER_EVT_DISCONNECTED;

				p_ble_user->conn_handle = BLE_CONN_HANDLE_INVALID;
				p_ble_user->evt_handler(p_ble_user, &user_evt);
			}
			break;

		default:
			// No implementation needed.
			break;
	}
}

uint32_t ble_central_user_init(ble_central_user_t* p_user, const ble_central_user_init_t* p_user_init)
{
	uint32_t      err_code;
	ble_uuid_t    ble_uuid;
	ble_uuid128_t usrv_base_uuid = USRV_BASE_UUID;

	VERIFY_PARAM_NOT_NULL(p_user);
	VERIFY_PARAM_NOT_NULL(p_user_init);

	// Initialize the service structure.
	p_user->conn_handle             = BLE_CONN_HANDLE_INVALID;
	p_user->data_handler            = p_user_init->data_handler;
	p_user->evt_handler             = p_user_init->evt_handler;
	p_user->is_notification_enabled = false;

	/**@snippet [Adding proprietary Service to S110 SoftDevice] */
	// Add a custom base UUID.
	err_code = sd_ble_uuid_vs_add(&usrv_base_uuid, &p_user->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_user->uuid_type;
	ble_uuid.uuid = BLE_UUID_USRV_SERVICE;

	// Add the service.
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
	                                    &ble_uuid,
	                                    &p_user->service_handle);
	/**@snippet [Adding proprietary Service to S110 SoftDevice] */
	VERIFY_SUCCESS(err_code);

	// Add the RX Characteristic.
	err_code = rx_char_add(p_user, p_user_init);
	VERIFY_SUCCESS(err_code);

	// Add the TX Characteristic.
	err_code = tx_char_add(p_user, p_user_init);
	VERIFY_SUCCESS(err_code);

	return NRF_SUCCESS;
}

uint32_t ble_central_user_send(ble_central_user_t* p_user, uint8_t* p_string, uint16_t* p_length)
{
	ble_gatts_hvx_params_t hvx_params;

	VERIFY_PARAM_NOT_NULL(p_user);

	if (p_user->conn_handle == BLE_CONN_HANDLE_INVALID) {
		return NRF_ERROR_INVALID_STATE;
	}

	if (*p_length > BLE_USRV_MAX_DATA_LEN) {
		return NRF_ERROR_INVALID_PARAM;
	}

	NRF_LOG_RAW_INFO("Tx:");
	for (uint32_t i = 0; i < *p_length; i++) {
		NRF_LOG_RAW_INFO("0x%02X ", p_string[i]);
	}
	NRF_LOG_RAW_INFO("\n");

	ble_gattc_write_params_t const write_params = {
		.write_op = BLE_GATT_OP_WRITE_CMD,
		.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
		.handle   = p_user->handles.user_rx_handle,
		.offset   = 0,
		.len      = *p_length,
		.p_value  = p_string
	};

	return sd_ble_gattc_write(p_user->conn_handle, &write_params);
}

