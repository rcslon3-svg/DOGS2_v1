#include "bluetooth_spp.h"

#include <string.h>
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "probe_config.h"

static uint32_t s_client_handle;
static bluetooth_command_cb_t s_command_callback;

/* Bluedroid owns this callback.  Connection events only update the active SPP
 * handle; received bytes are handed to the application's non-blocking queue. */
/* spp_callback
 * Inputs: event and parameter are supplied by the ESP-IDF SPP stack.
 * Returns: none.
 * Does: tracks connection state and forwards received Bluetooth bytes to the
 * command callback registered by main.c.
 */
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *parameter)
{
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_bt_gap_set_device_name(BLUETOOTH_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, BLUETOOTH_DEVICE_NAME);
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            s_client_handle = parameter->srv_open.handle;
            break;
        case ESP_SPP_CLOSE_EVT:
            s_client_handle = 0U;
            break;
        case ESP_SPP_DATA_IND_EVT:
            if (s_command_callback != NULL) {
                for (int i = 0; i < parameter->data_ind.len; ++i)
                    s_command_callback((char)parameter->data_ind.data[i]);
            }
            break;
        default:
            break;
    }
}

/* bluetooth_spp_init
 * Inputs: callback receives each byte from the SPP connection, including CR/LF.
 * Returns: ESP_OK on success, or an ESP-IDF Bluetooth/NVS setup error.
 * Does: initializes Classic Bluetooth SPP and starts the serial service.
 */
esp_err_t bluetooth_spp_init(bluetooth_command_cb_t callback)
{
    s_command_callback = callback;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&config), "bt", "controller init");
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), "bt", "controller enable");
    ESP_RETURN_ON_ERROR(esp_bluedroid_init(), "bt", "bluedroid init");
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), "bt", "bluedroid enable");
    ESP_RETURN_ON_ERROR(esp_spp_register_callback(spp_callback), "bt", "spp callback");
    const esp_spp_cfg_t spp_config = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0U
    };
    return esp_spp_enhanced_init(&spp_config);
}

/* bluetooth_spp_connected
 * Inputs: none.
 * Returns: true when an SPP client is currently connected.
 * Does: exposes the current Bluetooth connection flag to other modules.
 */
bool bluetooth_spp_connected(void)
{
    return s_client_handle != 0U;
}

/* bluetooth_spp_write
 * Inputs: data points to bytes to send; length is the number of bytes.
 * Returns: none.
 * Does: writes data to the connected SPP client when a connection is active.
 */
void bluetooth_spp_write(const char *data, size_t length)
{
    /* Dropping telemetry while disconnected is intentional: measurement must
     * never wait for a phone, and old samples have no value after reconnect. */
    if (s_client_handle == 0U || data == NULL || length == 0U) return;
    esp_spp_write(s_client_handle, (int)length, (uint8_t *)data);
}
