
#include "sdk_common.h"
#include "ble.h"
#include "ble_user_srv.h"
#include "ble_srv_common.h"

// #include "platform.h"

#define NRF_LOG_MODULE_NAME ble_usrv
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#define LOG_RAW NRF_LOG_RAW_INFO

#define BLE_UUID_USRV_TX_CHARACTERISTIC 0xa801                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_USRV_RX_CHARACTERISTIC 0xa802                      /**< The UUID of the RX Characteristic. */

#define BLE_USRV_MAX_RX_CHAR_LEN        BLE_USRV_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_USRV_MAX_TX_CHAR_LEN        BLE_USRV_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

#define USRV_BASE_UUID                  {{0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */

// ble_user_t m_user;
uint32_t ble_user_send_from_queue(void);

static void on_connect(ble_user_t* p_user, ble_evt_t const* p_ble_evt)
{
	ret_code_t                 err_code;
	ble_user_evt_t              evt;
	ble_gatts_value_t          gatts_val;
	uint8_t                    cccd_value[2];
	ble_user_client_context_t* p_client = NULL;

	err_code = blcm_link_ctx_get(p_user->p_link_ctx_storage,
	                             p_ble_evt->evt.gap_evt.conn_handle,
	                             (void*) &p_client);
	if (err_code != NRF_SUCCESS) {
		NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
		              p_ble_evt->evt.gap_evt.conn_handle);
	}

	/* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
	memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
	gatts_val.p_value = cccd_value;
	gatts_val.len     = sizeof(cccd_value);
	gatts_val.offset  = 0;

	err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
	                                  p_user->rx_handles.cccd_handle,
	                                  &gatts_val);

	if ((err_code == NRF_SUCCESS)     &&
	        (p_user->data_handler != NULL) &&
	        ble_srv_is_notification_enabled(gatts_val.p_value)) {
		if (p_client != NULL) {
			p_client->is_notification_enabled = true;
		}

		memset(&evt, 0, sizeof(ble_user_evt_t));
		evt.type        = BLE_USER_EVT_COMM_STARTED;
		evt.p_user       = p_user;
		evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		evt.p_link_ctx  = p_client;

		p_user->data_handler(&evt);
	}
}


static void on_disconnect(ble_user_t* p_user, ble_evt_t const* p_ble_evt)
{
	// UNUSED_PARAMETER(p_ble_evt);
	// p_user->conn_handle = BLE_CONN_HANDLE_INVALID;
}


static void on_write(ble_user_t* p_user, ble_evt_t const* p_ble_evt)
{
	ret_code_t                    err_code;
	ble_user_evt_t                 evt;
	ble_user_client_context_t*     p_client;
	ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	err_code = blcm_link_ctx_get(p_user->p_link_ctx_storage,
	                             p_ble_evt->evt.gatts_evt.conn_handle,
	                             (void*) &p_client);
	if (err_code != NRF_SUCCESS) {
		NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
		              p_ble_evt->evt.gatts_evt.conn_handle);
	}

	memset(&evt, 0, sizeof(ble_user_evt_t));
	evt.p_user       = p_user;
	evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
	evt.p_link_ctx  = p_client;

	if ((p_evt_write->handle == p_user->tx_handles.cccd_handle) &&
	        (p_evt_write->len == 2)) {
		if (p_client != NULL) {
			if (ble_srv_is_notification_enabled(p_evt_write->data)) {
				p_client->is_notification_enabled = true;
				evt.type                          = BLE_USER_EVT_COMM_STARTED;
			} else {
				p_client->is_notification_enabled = false;
				evt.type                          = BLE_USER_EVT_COMM_STOPPED;
			}

			if (p_user->data_handler != NULL) {
				p_user->data_handler(&evt);
			}

		}
	} else if ((p_evt_write->handle == p_user->rx_handles.value_handle) &&
	           (p_user->data_handler != NULL)) {
		evt.type                  = BLE_USER_EVT_RX_DATA;
		evt.params.rx_data.p_data = p_evt_write->data;
		evt.params.rx_data.length = p_evt_write->len;

		p_user->data_handler(&evt);
	} else {
		// Do Nothing. This event is not relevant for this service.
	}
}

static void on_hvx_tx_complete(ble_user_t* p_user, ble_evt_t const* p_ble_evt)
{
	ret_code_t                 err_code;
	ble_user_evt_t              evt;
	ble_user_client_context_t* p_client;

	err_code = blcm_link_ctx_get(p_user->p_link_ctx_storage,
	                             p_ble_evt->evt.gatts_evt.conn_handle,
	                             (void*) &p_client);
	if (err_code != NRF_SUCCESS) {
		NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
		              p_ble_evt->evt.gatts_evt.conn_handle);
		return;
	}

	if (p_client->is_notification_enabled) {
		memset(&evt, 0, sizeof(ble_user_evt_t));
		evt.type        = BLE_USER_EVT_TX_RDY;
		evt.p_user       = p_user;
		evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
		evt.p_link_ctx  = p_client;

		p_user->data_handler(&evt);
	}
}

uint8_t gatt_hvn_tx_lock_timeout = 0;
bool gatt_hvn_tx_lock = false;
uint8_t gatt_hvn_tx_queue_length = 0;
void ble_user_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context)
{
	if ((p_context == NULL) || (p_ble_evt == NULL)) {
		return;
	}
	ble_user_t* p_user = (ble_user_t*)p_context;

	switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_user, p_ble_evt);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_user, p_ble_evt);
			break;

		case BLE_GATTS_EVT_WRITE:
			on_write(p_user, p_ble_evt);
			break;

		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
			on_hvx_tx_complete(p_user, p_ble_evt);
			// if(gatt_hvn_tx_queue_length > p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count) {
			// 	gatt_hvn_tx_queue_length -= p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
			// } else {
			// 	gatt_hvn_tx_queue_length = 0;
			// 	if(ble_user_send_from_queue() == 0) {
			// 		gatt_hvn_tx_lock = false;
			// 	}
			// }
			// LOG_RAW("BLE_GATTS_EVT_HVN_TX_COMPLETE LENGTH:%d\r\n", gatt_hvn_tx_queue_length);
			break;

		default:
			// No implementation needed.
			break;
	}
}

uint32_t ble_user_init(ble_user_t* p_user, const ble_user_init_t* p_user_init)
{
	uint32_t      err_code;
	ble_uuid_t    ble_uuid;
	ble_uuid128_t usrv_base_uuid = USRV_BASE_UUID;
	ble_add_char_params_t add_char_params;

	VERIFY_PARAM_NOT_NULL(p_user);
	VERIFY_PARAM_NOT_NULL(p_user_init);

	// Initialize the service structure.
	p_user->data_handler            = p_user_init->data_handler;

	// Add a custom base UUID.
	err_code = sd_ble_uuid_vs_add(&usrv_base_uuid, &p_user->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_user->uuid_type;
	ble_uuid.uuid = BLE_UUID_USRV_SERVICE;

	// Add the service.
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
	                                    &ble_uuid,
	                                    &p_user->service_handle);
	VERIFY_SUCCESS(err_code);

	// Add the RX Characteristic.
	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid                     = BLE_UUID_USRV_RX_CHARACTERISTIC;
	add_char_params.uuid_type                = p_user->uuid_type;
	add_char_params.max_len                  = BLE_USRV_MAX_RX_CHAR_LEN;
	add_char_params.init_len                 = sizeof(uint8_t);
	add_char_params.is_var_len               = true;
	// add_char_params.char_props.write         = 1;
	add_char_params.char_props.write_wo_resp = 1;

	add_char_params.read_access  = SEC_OPEN;
	add_char_params.write_access = SEC_OPEN;

	err_code = characteristic_add(p_user->service_handle, &add_char_params, &p_user->rx_handles);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	// Add the TX Characteristic.
	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_USRV_TX_CHARACTERISTIC;
	add_char_params.uuid_type         = p_user->uuid_type;
	add_char_params.max_len           = BLE_USRV_MAX_TX_CHAR_LEN;
	add_char_params.init_len          = sizeof(uint8_t);
	add_char_params.is_var_len        = true;
	add_char_params.char_props.notify = 1;

	add_char_params.read_access       = SEC_OPEN;
	add_char_params.write_access      = SEC_OPEN;
	add_char_params.cccd_write_access = SEC_OPEN;

	return characteristic_add(p_user->service_handle, &add_char_params, &p_user->tx_handles);
}

ble_user_t* last_p_user;

uint8_t sdp_w = 0;	// 数据池写入指针
uint8_t send_data_pool[160];	// 修改为固定分配20bytes每个slot
typedef struct {
	uint8_t* p_string;
	uint16_t length;
} sDATAQUEUE;

uint8_t pDQ_w = 0;
sDATAQUEUE sDataQueue[4] = {{NULL, 0}};

uint32_t ble_user_send_queue(uint8_t* p_string, uint16_t length)
{
	if(pDQ_w >= 4) {
		LOG_RAW("USER_QUEUE_OVERFLOW!!!----------!!!\r\n");
		return -1;
	}

	memcpy(send_data_pool + sdp_w, p_string, length);
	sDataQueue[pDQ_w].p_string = send_data_pool + sdp_w;
	sDataQueue[pDQ_w].length = length;
	sdp_w = pDQ_w * 20;
	pDQ_w += 1;
	LOG_RAW("USER_QUEUE_LEN:%d\r\n", pDQ_w);
	return 0;
}

uint32_t ble_user_send(ble_user_t* p_user, uint8_t* p_string, uint16_t* p_length, uint16_t conn_handle)
{
	ret_code_t                 err_code;
	ble_gatts_hvx_params_t     hvx_params;
	ble_user_client_context_t* p_client;

	VERIFY_PARAM_NOT_NULL(p_user);

	err_code = blcm_link_ctx_get(p_user->p_link_ctx_storage, conn_handle, (void*) &p_client);
	VERIFY_SUCCESS(err_code);

	if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL)) {
		return NRF_ERROR_NOT_FOUND;
	}

	if (!p_client->is_notification_enabled) {
		return NRF_ERROR_INVALID_STATE;
	}

	if (*p_length > BLE_USRV_MAX_DATA_LEN) {
		return NRF_ERROR_INVALID_PARAM;
	}

	memset(&hvx_params, 0, sizeof(hvx_params));

	hvx_params.handle = p_user->tx_handles.value_handle;
	hvx_params.p_data = p_string;
	hvx_params.p_len  = p_length;
	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

	err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);

	return err_code;
}

