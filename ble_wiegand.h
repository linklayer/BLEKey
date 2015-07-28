#ifndef BLE_WIEGAND_H__
#define BLE_WIEGAND_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#include "wiegand.h"

// service defines
#define BLE_UUID_WIEGAND_LAST_CARDS	0xAAAA
#define BLE_MAX_TX_LEN	511
#define BLE_MAX_CARDS BLE_MAX_TX_LEN/sizeof(Card)
#define BLE_UUID_WIEGAND_REPLAY		0xBBBB
#define BLE_UUID_WIEGAND_SEND_DATA      0xCCCC
#define BLE_UUID_WIEGAND_SEND_DATA_LEN  20
#define BLE_UUID_WIEGAND_DATA_LENGTH	0xDDDD

/**@brief Heart Rate Service event type. */
typedef enum {
    BLE_WIEGAND_EVT_NOTIFICATION_ENABLED,                   /**< Heart Rate value notification enabled event. */
    BLE_WIEGAND_EVT_NOTIFICATION_DISABLED                   /**< Heart Rate value notification disabled event. */
} ble_wiegand_evt_type_t;

/**@brief Heart Rate Service event. */
typedef struct
{
    ble_wiegand_evt_type_t evt_type;                        /**< Type of event. */
} ble_wiegand_evt_t;

// Forward declaration of the ble_wiegand_t type.
typedef struct ble_wiegand_s ble_wiegand_t;

/**@brief Heart Rate Service event handler type. */
typedef void (*ble_wiegand_evt_handler_t) (ble_wiegand_t * p_wiegand, ble_wiegand_evt_t * p_evt);

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_wiegand_evt_handler_t    evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    bool                         is_sensor_contact_supported;                          /**< Determines if sensor contact detection is to be supported. */
    uint8_t *                    p_body_sensor_location;                               /**< If not NULL, initial value of the Body Sensor Location characteristic. */
    ble_srv_security_mode_t 	 wiegand_last_cards_attr_md;                           /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t      wiegand_replay_attr_md;                               /**< Initial security level for body sensor location attribute */
    ble_srv_security_mode_t      wiegand_send_data_attr_md;                            /**< Initial security level for body sensor location attribute */
    ble_srv_security_mode_t      wiegand_data_length_attr_md;                          /**< Initial security level for body sensor location attribute */
} ble_wiegand_init_t;

/**@brief Heart Rate Service structure. This contains various status information for the service. */
typedef struct ble_wiegand_s
{
    ble_wiegand_evt_handler_t    evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    bool                         is_expended_energy_supported;                         /**< TRUE if Expended Energy measurement is supported. */
    bool                         is_sensor_contact_supported;                          /**< TRUE if sensor contact detection is supported. */
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     last_cards_handles;                             	/**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     replay_handles;                                       /**< Handles related to the Body Sensor Location characteristic. */
    ble_gatts_char_handles_t     send_data_handles;                                    /**< Handles related to the Heart Rate Control Point characteristic. */
    ble_gatts_char_handles_t     data_length_handles;                                  /**< Handles related to the Heart Rate Control Point characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                         is_sensor_contact_detected;                           /**< TRUE if sensor contact has been detected. */
    uint16_t                     rr_interval_count;                                    /**< Number of RR Interval measurements since the last Heart Rate Measurement transmission. */
} ble_wiegand_t;

/**@brief Function for initializing the Heart Rate Service.
 *
 * @param[out]  p_wiegand   Heart Rate Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_wiegand_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_wiegand_init(ble_wiegand_t * p_wiegand, const ble_wiegand_init_t * p_wiegand_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_wiegand  Heart Rate Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_wiegand_on_ble_evt(ble_wiegand_t * p_wiegand, ble_evt_t * p_ble_evt);

/**@brief Function for setting the state of the Sensor Contact Supported bit.
 *
 * @param[in]   p_wiegand                    Heart Rate Service structure.
 * @param[in]   is_sensor_contact_supported  New state of the Sensor Contact Supported bit.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_wiegand_sensor_contact_supported_set(ble_wiegand_t * p_wiegand, bool is_sensor_contact_supported);

/**@brief Function for setting the state of the Sensor Contact Detected bit.
 *
 * @param[in]   p_wiegand                    Heart Rate Service structure.
 * @param[in]   is_sensor_contact_detected   TRUE if sensor contact is detected, FALSE otherwise.
 */
void ble_wiegand_sensor_contact_detected_update(ble_wiegand_t * p_wiegand, bool is_sensor_contact_detected);

/**@brief Function for setting the Body Sensor Location.
 *
 * @details Sets a new value of the Body Sensor Location characteristic. The new value will be sent
 *          to the client the next time the client reads the Body Sensor Location characteristic.
 *
 * @param[in]   p_wiegand             Heart Rate Service structure.
 * @param[in]   body_sensor_location  New Body Sensor Location.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_wiegand_body_sensor_location_set(ble_wiegand_t * p_wiegand, uint8_t body_sensor_location);

uint32_t ble_wiegand_last_cards_set(ble_wiegand_t * p_wiegand, uint8_t *cards, uint16_t len);

#endif // BLE_WIEGAND_H__

/** @} */
