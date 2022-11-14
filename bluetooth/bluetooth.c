
#include "bluetooth.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_user_srv.h"
// #include "ble_central_user_srv.h"
// #include "ble_dfu.h"

// #include "app_uart.h"
#include "app_util_platform.h"
#include "platform.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// #include "app_util.h"
#include "ble_gap.h"
#include "nrf_ble_scan.h"
// ********

#include "build.h"

#define APP_BLE_CONN_CFG_TAG            2                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define USER_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_FAST_INTERVAL            MSEC_TO_UNITS(50,UNIT_0_625_MS)         /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL            MSEC_TO_UNITS(150,UNIT_0_625_MS)        /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT             10                                       /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT             0                                        /**< The duration of the slow advertising period (in seconds). */

#define MIN_CONN_MS 25
#define MAX_CONN_MS 50
#define CONN_TIMEOUT_MS 1000

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(MIN_CONN_MS, UNIT_1_25_MS)         	/**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(MAX_CONN_MS, UNIT_1_25_MS)         	/**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                    2                                        			/**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(CONN_TIMEOUT_MS, UNIT_10_MS)         /**< Connection supervisory timeout (430 ms). */

#if (MAX_CONN_MS * (SLAVE_LATENCY+1) > 2000)
	#error "Connection parameter will not be accepted by iOS"
#elif (MAX_CONN_MS < 20)
	#error "Connection parameter will not be accepted by iOS"
#elif (MIN_CONN_MS + 20 > MAX_CONN_MS)
	#error "Connection parameter will not be accepted by iOS"
#elif (SLAVE_LATENCY > 4)
	#error "Connection parameter will not be accepted by iOS"
#elif (CONN_TIMEOUT_MS > 6000)
	#error "Connection parameter will not be accepted by iOS"
#elif (MAX_CONN_MS * (SLAVE_LATENCY + 1) * 3 >= CONN_TIMEOUT_MS)
	#error "Connection parameter will not be accepted by iOS"
#endif

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

NRF_BLE_SCAN_DEF(m_scan);

BLE_USER_DEF(m_user, NRF_SDH_BLE_TOTAL_LINK_COUNT);

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */


static ble_uuid_t const m_user_uuid = {
	.uuid = BLE_UUID_USRV_SERVICE,
	.type = BLE_UUID_TYPE_BLE
};

static uint8_t m_gap_role     = BLE_GAP_ROLE_INVALID;       /**< BLE role for this connection, see @ref BLE_GAP_ROLES */
static uint16_t m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
	{BLE_UUID_USRV_SERVICE, USER_SERVICE_UUID_TYPE}
};

static void scan_start(void);
static void scan_stop(void);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
}


// 48bit addr in LSB = bluetooth_addr.addr[6]
ble_gap_addr_t bluetooth_addr;

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
	uint32_t                err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
	                                      (const uint8_t*) DEVICE_NAME,
	                                      strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_addr_get(&bluetooth_addr);
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_WATCH);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static uint8_t data_buffer_flag = 0;
static uint8_t data_buffer[4][256];
static uint8_t data_buffer_length[4];
static ble_user_data_t p_user_data[4];

static void user_data_handler(ble_user_evt_t* p_evt)
{
	if (p_evt->type == BLE_USER_EVT_RX_DATA) {
		// uint32_t err_code;
		LOG_RAW("Rx:");
		LOG_HEX_RAW(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
		memcpy(data_buffer[data_buffer_flag], p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
		data_buffer_length[data_buffer_flag] = p_evt->params.rx_data.length;
		p_user_data[data_buffer_flag].p_data = data_buffer[data_buffer_flag];
		p_user_data[data_buffer_flag].p_length = (uint16_t*)&data_buffer_length[data_buffer_flag];
		uevt_bc(UEVT_BT_DATARECV, &p_user_data[data_buffer_flag]);
		data_buffer_flag = (data_buffer_flag + 1) & 0x3;
	}
}
/**@snippet [Handling the data received over BLE] */

static void disconnect(uint16_t conn_handle, void* p_context)
{
	UNUSED_PARAMETER(p_context);

	ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (err_code != NRF_SUCCESS) {
		NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
	} else {
		NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
	}
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t* p_evt)
{
	uint32_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
	uint32_t               err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params                  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail             = false;
	cp_init.evt_handler                    = on_conn_params_evt;
	cp_init.error_handler                  = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	uint32_t err_code = sd_power_system_off();
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	// uint32_t err_code;

	switch (ble_adv_evt) {
		case BLE_ADV_EVT_FAST:
			break;
		case BLE_ADV_EVT_IDLE:
			sleep_mode_enter();
			break;
		default:
			break;
	}
}
// static void ble_central_user_evt_handler(ble_central_user_t* p_ble_user, ble_central_user_evt_t const* p_ble_user_evt)
// {
// 	ret_code_t err_code;

// 	switch (p_ble_user_evt->evt_type) {
// 		case BLE_USER_EVT_DISCOVERY_COMPLETE:
// 			NRF_LOG_INFO("Discovery complete.");
// 			err_code = ble_user_handles_assign(p_ble_user, p_ble_user_evt->conn_handle, &p_ble_user_evt->handles);
// 			APP_ERROR_CHECK(err_code);

// 			err_code = ble_user_tx_notif_enable(p_ble_user);
// 			APP_ERROR_CHECK(err_code);
// 			NRF_LOG_INFO("Connected to Corumi device.");
// 			break;

// 		case BLE_USER_EVT_TX_EVT:
// 			// ble_user_data_handler(p_ble_user, p_ble_user_evt->p_data, p_ble_user_evt->data_len);
// 			break;

// 		case BLE_USER_EVT_DISCONNECTED:
// 			NRF_LOG_INFO("Disconnected.");
// 			scan_start();
// 			break;
// 	}
// }

bool is_valid_device(const ble_gap_evt_adv_report_t* p_adv_report)
{
	// uint8_t i,j,p = 0;
	if(p_adv_report->rssi < -88) {
		return false;
	}

	// if(p_adv_report->peer_addr.addr[5] == 0xF1
	//         && p_adv_report->peer_addr.addr[4] == 0x0C
	//         && p_adv_report->peer_addr.addr[3] == 0x26
	//         && p_adv_report->peer_addr.addr[2] == 0xEC
	//         && p_adv_report->peer_addr.addr[1] == 0xF8
	//         && p_adv_report->peer_addr.addr[0] == 0xF7) {
	// 	NRF_LOG_RAW_INFO(" -> Found It!\n");
	// 	return true;
	// }
	// NRF_LOG_RAW_INFO("-->find_device:%02X:%02X:%02X:%02X:%02X:%02X",
	// 	p_adv_report->peer_addr.addr[0],
	// 	p_adv_report->peer_addr.addr[1],
	// 	p_adv_report->peer_addr.addr[2],
	// 	p_adv_report->peer_addr.addr[3],
	// 	p_adv_report->peer_addr.addr[4],
	// 	p_adv_report->peer_addr.addr[5]
	// );
	// NRF_LOG_RAW_INFO(" - rssi:%ddBm\n",
	//     p_adv_report->rssi
	// );
	// while(p<p_adv_report->data.len && p_adv_report->data.p_data[p+1]!=0xFF){
	//     p += p_adv_report->data.p_data[p+0]+1;
	// }
	// if(p>=p_adv_report->data.len){
	//     return false;
	// }else{
	//     if(p_adv_report->data.p_data[p+2]==0x50
	//         && p_adv_report->data.p_data[p+3]==0x50
	//         && p_adv_report->data.p_data[p+4]==0xDE
	//         && p_adv_report->data.p_data[p+5]==0xAD
	//         && p_adv_report->data.p_data[p+6]==0xBE
	//         && p_adv_report->data.p_data[p+7]==0xEF
	//         && p_adv_report->data.p_data[p+8]==0x00
	//         && p_adv_report->data.p_data[p+9]==0x01){
	//         return true;
	//     }
	// }
	return false;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const* p_ble_evt, void* p_context)
{
	uint32_t err_code;
	ble_gap_evt_t const* p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_ADV_REPORT: {
			const ble_gap_evt_adv_report_t* p_adv_report = &p_gap_evt->params.adv_report;
			// For readability.
			nrf_ble_scan_t*               p_scan_ctx  = &m_scan;
			ble_gap_addr_t const*         p_addr        = &p_adv_report->peer_addr;
			ble_gap_scan_params_t const* p_scan_params = &p_scan_ctx->scan_params;
			ble_gap_conn_params_t const* p_conn_params = &p_scan_ctx->conn_params;
			uint8_t                       con_cfg_tag   = p_scan_ctx->conn_cfg_tag;

			if(is_valid_device(p_adv_report) == true) {
				NRF_LOG_RAW_INFO("Scan stop\n");
				nrf_ble_scan_stop();
				// Establish connection.
				NRF_LOG_RAW_INFO("Start connect\n");
				err_code = sd_ble_gap_connect(p_addr,
				                              p_scan_params,
				                              p_conn_params,
				                              con_cfg_tag);
				NRF_LOG_RAW_INFO("Connecting\n");
				if (err_code != NRF_SUCCESS) {
					NRF_LOG_RAW_INFO("Err:[%d]\n", err_code);
					sd_ble_gap_connect_cancel();
					scan_start();
				}
			}
		}
		break; // BLE_GAP_EVT_ADV_REPORT
		case BLE_GAP_EVT_CONNECTED: {
			m_gap_role    = p_gap_evt->params.connected.role;
			if (m_gap_role == BLE_GAP_ROLE_PERIPH) {
				NRF_LOG_RAW_INFO("Connected\n");
				m_conn_handle = p_gap_evt->conn_handle;
				uevt_bc_e(UEVT_BT_CONN);
			}
		}
		break;

		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_RAW_INFO("Disconnected\n");
			// LED indication will be changed when advertising starts.
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			uevt_bc_e(UEVT_BT_DISCONN);
			if (m_gap_role == BLE_GAP_ROLE_CENTRAL) {
				// 如果是主机模式则自动
				scan_start();
			}
			break;

		case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
			NRF_LOG_DEBUG("PHY update request.");
			ble_gap_phys_t const phys = {
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
			};
			err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
			APP_ERROR_CHECK(err_code);
		}
		break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			// Pairing not supported
			err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
			// Accepting parameters requested by peer.
			err_code = sd_ble_gap_conn_param_update(p_ble_evt->evt.gap_evt.conn_handle,
			                                        &p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GAP_EVT_TIMEOUT:
			if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
				NRF_LOG_RAW_INFO("Connection Request timed out.\n");
			}
			break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			// No system attributes have been stored.
			err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTC_EVT_TIMEOUT:
			// Disconnect on GATT Client timeout event.
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
			                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server timeout event.
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
			                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;

		default:
			// No implementation needed.
			break;
	}
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	uint32_t           err_code;
	ble_user_init_t     user_init;

	memset(&user_init, 0, sizeof(user_init));
	user_init.data_handler = user_data_handler;
	err_code = ble_user_init(&m_user, &user_init);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// // Overwrite some of the default configurations for the BLE stack.
	// ble_cfg_t ble_cfg;

	// // MTU & DLE
	// memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	// ble_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
	// ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 10;
	// ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
	// err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
	// APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t* p_gatt, nrf_ble_gatt_evt_t const* p_evt)
{
	if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
		m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
		NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
	}
	NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
	              p_gatt->att_mtu_desired_central,
	              p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
	ret_code_t err_code;

	err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	uint32_t               err_code;
	ble_advertising_init_t init;

	uint8_t manuf_data_raw[] = {
		0xCC, 0x22,
		DEVICE_IDN,
		bluetooth_addr.addr[5],
		bluetooth_addr.addr[4],
		bluetooth_addr.addr[3],
		bluetooth_addr.addr[2],
		bluetooth_addr.addr[1],
		bluetooth_addr.addr[0],
	};
	ble_advdata_manuf_data_t manuf_data = {0x5050, {sizeof(manuf_data_raw), manuf_data_raw}};
	memset(&init, 0, sizeof(init));

	init.srdata.name_type          = BLE_ADVDATA_FULL_NAME;
	init.srdata.include_appearance = true;
	init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	init.advdata.p_manuf_specific_data = &manuf_data;

	// init.advdata.uuids_more_available.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	// init.advdata.uuids_more_available.p_uuids  = m_adv_uuids;

	init.config.ble_adv_fast_enabled  = true;
	init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
	init.config.ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT;
	init.config.ble_adv_slow_enabled  = true;
	init.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
	init.config.ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT;
	init.config.ble_adv_extended_enabled = false;
	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void scan_evt_handler(scan_evt_t const* p_scan_evt)
{
	ret_code_t err_code;

	switch(p_scan_evt->scan_evt_id) {
		case NRF_BLE_SCAN_EVT_CONNECTING_ERROR: {
			err_code = p_scan_evt->params.connecting_err.err_code;
			APP_ERROR_CHECK(err_code);
		}
		break;

		case NRF_BLE_SCAN_EVT_CONNECTED: {
			ble_gap_evt_connected_t const* p_connected =
			    p_scan_evt->params.connected.p_connected;
			// Scan is automatically stopped by the connection.
			NRF_LOG_RAW_INFO("Connecting to target %02x:%02x:%02x:%02x:%02x:%02x\n",
			                 p_connected->peer_addr.addr[0],
			                 p_connected->peer_addr.addr[1],
			                 p_connected->peer_addr.addr[2],
			                 p_connected->peer_addr.addr[3],
			                 p_connected->peer_addr.addr[4],
			                 p_connected->peer_addr.addr[5]
			                );
		}
		break;

		case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
			NRF_LOG_RAW_INFO("Scan timed out.\n");
			scan_start();
		}
		break;

		default:
			break;
	}
}

static void scan_init(void)
{
	ret_code_t          err_code;
	nrf_ble_scan_init_t init_scan;

	memset(&init_scan, 0, sizeof(init_scan));

	// init_scan.connect_if_match = true;
	init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

	err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
	APP_ERROR_CHECK(err_code);

	// err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, scan_name);
	// APP_ERROR_CHECK(err_code);

	// err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
	// APP_ERROR_CHECK(err_code);
}

static void scan_start(void)
{
	ret_code_t ret;
	ret = nrf_ble_scan_start(&m_scan);
	APP_ERROR_CHECK(ret);
}
static void scan_stop(void)
{
	nrf_ble_scan_stop();
}

/**@brief Function for starting advertising.
 */
void ble_adv_start(void)
{
	uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);
	uevt_bc_e(UEVT_BT_ADV_START);
}

void ble_adv_stop(void)
{
	ret_code_t ret = sd_ble_gap_adv_stop(m_advertising.adv_handle);
	if(ret == 0) {
		NRF_LOG_RAW_INFO("Stop advertising.\n");
		uevt_bc_e(UEVT_BT_ADV_STOP);
	}
}

void bluetooth_init(void)
{
	user_event_handler_regist(bt_on_uevt_handler);
	timers_init();
	// db_discovery_init();	// central
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	// scan_init();	// central
	advertising_init();
	conn_params_init();
	uevt_bc_e(UEVT_BT_INIT);
}

void bt_on_uevt_handler(uevt_t* evt)
{
	// uint32_t err_code;
	switch(evt->evt_id) {
		case UEVT_BT_CTL_INIT:
			bluetooth_init();
			break;
		case UEVT_BT_CTL_ADV_ON:
			ble_adv_start();
			break;
		case UEVT_BT_CTL_ADV_OFF:
			ble_adv_stop();
			break;
		case UEVT_BT_CTL_DISCONN:
			if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
				disconnect(m_conn_handle, NULL);
			}
			break;
		case UEVT_BT_CTL_DATASEND:
			ble_user_send(&m_user,
			              ((ble_user_data_t*)(evt->content))->p_data,
			              ((ble_user_data_t*)(evt->content))->p_length,
			              m_conn_handle);
			break;
		case UEVT_BT_CTL_RSSI_START:
			sd_ble_gap_rssi_start(m_conn_handle, 5, 3);
			break;
		case UEVT_BT_CTL_RSSI_STOP:
			sd_ble_gap_rssi_stop(m_conn_handle);
			break;
		case UEVT_BT_CTL_RSSI_GET: {
			static ble_rssi_t rssi;
			if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
				if(sd_ble_gap_rssi_get(m_conn_handle, &(rssi.rssi), &(rssi.ch)) == NRF_SUCCESS) {
					uevt_bc(UEVT_BT_RSSI, &rssi);
				}
			}
		}
		break;

		case UEVT_BT_CTL_SCAN_ON:
			scan_start();
			break;
		case UEVT_BT_CTL_SCAN_OFF:
			scan_stop();
			break;
		case UEVT_RTC_8HZ: {
			static uint8_t flag = 0;
			if(m_conn_handle == BLE_CONN_HANDLE_INVALID) {
				if((flag & 0x7) == 0) {
					// motor_on();
				}
				if((flag & 0x7) == 1) {
					motor_off();
				}
			} else {
				if((flag & 0xF) == 0) {
					// motor_on();
				}
				if((flag & 0xF) == 7) {
					motor_off();
				}
			}
			flag += 1;
		}
		break;
	}
}
