
#ifndef BLE_CENTRAL_USER_SRV_H__
#define BLE_CENTRAL_USER_SRV_H__


#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_CENTRAL_USER_DEF(_name)                                                                        \
	static ble_central_user_t _name;                                                                           \
	NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
	                     2,                                                   \
	                     ble_central_user_on_ble_evt, &_name)

#define BLE_CENTRAL_USER_ARRAY_DEF(_name, _cnt)                 \
	static ble_central_user_t _name[_cnt];                          \
	NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
	                      2,       \
	                      ble_central_user_on_ble_evt, &_name, _cnt)

#define BLE_UUID_USRV_SERVICE 0xa800
#define BLE_UUID_USRV_TX_CHARACTERISTIC 0xa801                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_USRV_RX_CHARACTERISTIC 0xa802                      /**< The UUID of the RX Characteristic. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
#define BLE_USRV_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
#define BLE_USRV_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
#warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

/**@brief Client event type. */
typedef enum {
	BLE_USER_EVT_DISCOVERY_COMPLETE,   /**< Event indicating that the NUS service and its characteristics was found. */
	BLE_USER_EVT_TX_EVT,           /**< Event indicating that the central has received something from a peer. */
	BLE_USER_EVT_DISCONNECTED          /**< Event indicating that the NUS server has disconnected. */
} ble_central_user_evt_type_t;
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct {
	uint16_t user_tx_handle;      /**< Handle of the NUS TX characteristic as provided by a discovery. */
	uint16_t user_tx_cccd_handle; /**< Handle of the CCCD of the NUS TX characteristic as provided by a discovery. */
	uint16_t user_rx_handle;      /**< Handle of the NUS RX characteristic as provided by a discovery. */
} ble_central_user_handles_t;
/**@brief Structure containing the event data received from the peer. */
typedef struct {
	ble_central_user_evt_type_t evt_type;
	uint16_t             conn_handle;
	uint16_t             max_data_len;
	uint8_t*             p_data;
	uint16_t             data_len;
	ble_central_user_handles_t  handles;     /**< Handles on which the Nordic Uart service characteristics was discovered on the peer device. This will be filled if the evt_type is @ref BLE_user_EVT_DISCOVERY_COMPLETE.*/
} ble_central_user_evt_t;

/* Forward declaration of the ble_central_user_t type. */
typedef struct ble_central_user_s ble_central_user_t;
/**@brief Service event handler type. */
typedef void (*ble_central_user_data_handler_t) (ble_central_user_t* p_user, uint8_t* p_data, uint16_t length);
typedef void (*ble_central_user_evt_handler_t)(ble_central_user_t* p_ble_user, ble_central_user_evt_t const* p_evt);

/**@brief Client initialization structure. */
typedef struct {
	ble_central_user_evt_handler_t evt_handler;
	ble_central_user_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_central_user_init_t;

/**@brief Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_central_user_s {
	uint8_t                  uuid_type;               /**< UUID type for Service Base UUID. */
	uint16_t                 service_handle;          /**< Handle of Service (as provided by the SoftDevice). */
	ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
	ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
	uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
	bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
	ble_central_user_data_handler_t  data_handler;            /**< Event handler to be called for handling received data. */
	ble_central_user_handles_t    	 handles;        /**< Handles on the connected peer device needed to interact with it. */
	ble_central_user_evt_handler_t	 evt_handler;    /**< Application event handler to be called when there is an event related to the NUS. */
};

uint32_t ble_central_user_init(ble_central_user_t* p_user, const ble_central_user_init_t* p_user_init);

void ble_central_user_on_db_disc_evt(ble_central_user_t* p_ble_user, ble_db_discovery_evt_t* p_evt);
uint32_t ble_central_user_tx_notif_enable(ble_central_user_t* p_ble_user);
uint32_t ble_central_user_handles_assign(ble_central_user_t*                p_ble_user,
        uint16_t                    conn_handle,
        ble_central_user_handles_t const* p_peer_handles);


void ble_central_user_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);
uint32_t ble_central_user_send(ble_central_user_t* p_user, uint8_t* p_string, uint16_t* p_length);

#ifdef __cplusplus
}
#endif


#endif
