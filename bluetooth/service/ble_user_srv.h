
#ifndef BLE_USER_SRV_H__
#define BLE_USER_SRV_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#define BLE_USER_BLE_OBSERVER_PRIO 2
#define BLE_USER_DEF(_name, _user_max_clients)                      \
	BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
	                         (_user_max_clients),                  \
	                         sizeof(ble_user_client_context_t));   \
	static ble_user_t _name =                                      \
	        {                                                             \
	                                                                      .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
	        };                                                            \
	NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
	                     BLE_USER_BLE_OBSERVER_PRIO,               \
	                     ble_user_on_ble_evt,                      \
	                     &_name)

#define BLE_UUID_USRV_SERVICE 0xa800

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
	#define BLE_USRV_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#else
	#define BLE_USRV_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
	#warning NRF_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

/* Forward declaration of the ble_user_t type. */
typedef struct ble_user_s ble_user_t;
/**@brief   Nordic UART Service event types. */
typedef enum {
	BLE_USER_EVT_RX_DATA,      /**< Data received. */
	BLE_USER_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
	BLE_USER_EVT_COMM_STARTED, /**< Notification has been enabled. */
	BLE_USER_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_user_evt_type_t;
typedef struct {
	uint8_t const* p_data;  /**< A pointer to the buffer with received data. */
	uint16_t        length; /**< Length of received data. */
} ble_user_evt_rx_data_t;
typedef struct {
	bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_user_client_context_t;
typedef struct {
	ble_user_evt_type_t         type;        /**< Event type. */
	ble_user_t*                 p_user;       /**< A pointer to the instance. */
	uint16_t                   conn_handle; /**< Connection handle. */
	ble_user_client_context_t* p_link_ctx;   /**< A pointer to the link context. */
	union {
		ble_user_evt_rx_data_t rx_data; /**< @ref BLE_USER_EVT_RX_DATA event data. */
	} params;
} ble_user_evt_t;
typedef void (* ble_user_data_handler_t) (ble_user_evt_t* p_evt);

typedef struct {
	ble_user_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_user_init_t;

struct ble_user_s {
	uint8_t                         uuid_type;          /**< UUID type for Nordic UART Service Base UUID. */
	uint16_t                        service_handle;     /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
	ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
	ble_gatts_char_handles_t        rx_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
	blcm_link_ctx_storage_t* const p_link_ctx_storage;  /**< Pointer to link context storage with handles of all current connections and its context. */
	ble_user_data_handler_t          data_handler;       /**< Event handler to be called for handling received data. */
};

// extern ble_user_t m_user;
uint32_t ble_user_init(ble_user_t* p_user, ble_user_init_t const* p_user_init);
void ble_user_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);
uint32_t ble_user_send(ble_user_t* p_user, uint8_t* p_data, uint16_t* p_length, uint16_t conn_handle);

extern uint8_t gatt_hvn_tx_lock_timeout;
extern bool gatt_hvn_tx_lock;

#endif
