#include "bluetooth_spp.h"

#include <stdio.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "probe_config.h"

#define BLUETOOTH_SPP_TX_BUFFER_SIZE 512U
#define BLUETOOTH_SPP_TX_QUEUE_DEPTH 16U
#define BLUETOOTH_DEFAULT_BOARD_NUMBER 1U

static uint32_t s_client_handle;
static bool s_write_busy;
static uint8_t s_write_buffer[BLUETOOTH_SPP_TX_BUFFER_SIZE];
static uint8_t s_tx_queue[BLUETOOTH_SPP_TX_QUEUE_DEPTH][BLUETOOTH_SPP_TX_BUFFER_SIZE];
static uint16_t s_tx_queue_length[BLUETOOTH_SPP_TX_QUEUE_DEPTH];
static uint8_t s_tx_queue_head;
static uint8_t s_tx_queue_tail;
static uint8_t s_tx_queue_count;
static portMUX_TYPE s_tx_lock = portMUX_INITIALIZER_UNLOCKED;
static bluetooth_command_cb_t s_command_callback;
static char s_device_name[16] = BLUETOOTH_DEVICE_NAME;
static uint16_t s_board_number = BLUETOOTH_DEFAULT_BOARD_NUMBER;
static bool s_initialized;
static bool s_ble_memory_released;
static bool s_init_pending;
static bool s_deinit_pending;
static esp_err_t s_spp_start_result;
static SemaphoreHandle_t s_spp_init_done;
static SemaphoreHandle_t s_spp_uninit_done;

static void tx_queue_clear(void);

static void bluetooth_spp_force_stack_down(void)
{
    s_client_handle = 0U;
    tx_queue_clear();
    s_initialized = false;
    s_init_pending = false;
    s_deinit_pending = false;
    s_command_callback = NULL;

    esp_bluedroid_status_t bluedroid = esp_bluedroid_get_status();
    if (bluedroid == ESP_BLUEDROID_STATUS_ENABLED) {
        (void)esp_bluedroid_disable();
        bluedroid = esp_bluedroid_get_status();
    }
    if (bluedroid != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        (void)esp_bluedroid_deinit();
    }

    esp_bt_controller_status_t controller = esp_bt_controller_get_status();
    if (controller == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        (void)esp_bt_controller_disable();
        controller = esp_bt_controller_get_status();
    }
    if (controller != ESP_BT_CONTROLLER_STATUS_IDLE) {
        (void)esp_bt_controller_deinit();
    }
}

static void load_device_name(void)
{
    uint16_t board_number = BLUETOOTH_DEFAULT_BOARD_NUMBER;
    nvs_handle_t nvs;
    if (nvs_open("device", NVS_READWRITE, &nvs) == ESP_OK) {
        uint16_t stored = 0U;
        if (nvs_get_u16(nvs, "board_no", &stored) == ESP_OK && stored > 0U && stored <= 999U) {
            board_number = stored;
        } else {
            (void)nvs_set_u16(nvs, "board_no", board_number);
            (void)nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    s_board_number = board_number;
    (void)snprintf(s_device_name, sizeof(s_device_name), "DOGS2_A%03u", board_number);
}

static void tx_queue_clear(void)
{
    portENTER_CRITICAL(&s_tx_lock);
    s_write_busy = false;
    s_tx_queue_head = 0U;
    s_tx_queue_tail = 0U;
    s_tx_queue_count = 0U;
    portEXIT_CRITICAL(&s_tx_lock);
}

static bool tx_prepare_next(uint32_t *handle, uint16_t *length)
{
    bool ready = false;

    portENTER_CRITICAL(&s_tx_lock);
    if (s_client_handle != 0U && !s_write_busy && s_tx_queue_count != 0U) {
        *handle = s_client_handle;
        *length = s_tx_queue_length[s_tx_queue_head];
        memcpy(s_write_buffer, s_tx_queue[s_tx_queue_head], *length);
        s_tx_queue_head = (uint8_t)((s_tx_queue_head + 1U) % BLUETOOTH_SPP_TX_QUEUE_DEPTH);
        --s_tx_queue_count;
        s_write_busy = true;
        ready = true;
    }
    portEXIT_CRITICAL(&s_tx_lock);

    return ready;
}

static void tx_kick(void)
{
    uint32_t handle = 0U;
    uint16_t length = 0U;
    if (!tx_prepare_next(&handle, &length)) return;

    if (esp_spp_write(handle, (int)length, s_write_buffer) != ESP_OK) {
        portENTER_CRITICAL(&s_tx_lock);
        s_write_busy = false;
        portEXIT_CRITICAL(&s_tx_lock);
    }
}

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
            if (!s_init_pending) break;
            esp_bt_gap_set_device_name(s_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            s_spp_start_result = esp_spp_start_srv(ESP_SPP_SEC_NONE,
                                                   ESP_SPP_ROLE_SLAVE,
                                                   0,
                                                   s_device_name);
            if (s_spp_start_result != ESP_OK && s_spp_init_done != NULL) {
                xSemaphoreGive(s_spp_init_done);
            }
            break;
        case ESP_SPP_START_EVT:
            if (!s_init_pending) break;
            s_spp_start_result = ESP_OK;
            s_initialized = true;
            s_init_pending = false;
            if (s_spp_init_done != NULL) xSemaphoreGive(s_spp_init_done);
            break;
        case ESP_SPP_UNINIT_EVT:
            s_initialized = false;
            s_deinit_pending = false;
            s_client_handle = 0U;
            tx_queue_clear();
            if (s_spp_uninit_done != NULL) xSemaphoreGive(s_spp_uninit_done);
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            s_client_handle = parameter->srv_open.handle;
            tx_queue_clear();
            break;
        case ESP_SPP_CLOSE_EVT:
            s_client_handle = 0U;
            tx_queue_clear();
            break;
        case ESP_SPP_WRITE_EVT:
            portENTER_CRITICAL(&s_tx_lock);
            s_write_busy = false;
            portEXIT_CRITICAL(&s_tx_lock);
            tx_kick();
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
    esp_err_t result = ESP_OK;
    if (s_initialized) return ESP_OK;
    s_command_callback = callback;
    load_device_name();
    if (s_spp_init_done == NULL) {
        s_spp_init_done = xSemaphoreCreateBinary();
        if (s_spp_init_done == NULL) return ESP_ERR_NO_MEM;
    }
    if (s_spp_uninit_done == NULL) {
        s_spp_uninit_done = xSemaphoreCreateBinary();
        if (s_spp_uninit_done == NULL) return ESP_ERR_NO_MEM;
    }
    while (xSemaphoreTake(s_spp_init_done, 0) == pdTRUE) {}
    while (xSemaphoreTake(s_spp_uninit_done, 0) == pdTRUE) {}
    s_spp_start_result = ESP_ERR_INVALID_STATE;
    if (!s_ble_memory_released) {
        esp_err_t release = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
        if (release != ESP_OK && release != ESP_ERR_INVALID_STATE) return release;
        s_ble_memory_released = true;
    }
    s_init_pending = true;
    s_deinit_pending = false;
    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((result = esp_bt_controller_init(&config)) != ESP_OK) goto fail;
    if ((result = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) goto fail;
    if ((result = esp_bluedroid_init()) != ESP_OK) goto fail;
    if ((result = esp_bluedroid_enable()) != ESP_OK) goto fail;
    if ((result = esp_spp_register_callback(spp_callback)) != ESP_OK) goto fail;
    const esp_spp_cfg_t spp_config = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0U
    };
    if ((result = esp_spp_enhanced_init(&spp_config)) != ESP_OK) goto fail;
    if (xSemaphoreTake(s_spp_init_done, pdMS_TO_TICKS(3000)) != pdTRUE) {
        result = ESP_ERR_TIMEOUT;
        goto fail;
    }
    if (s_spp_start_result != ESP_OK) {
        result = s_spp_start_result;
        goto fail;
    }
    return ESP_OK;

fail:
    bluetooth_spp_force_stack_down();
    return result;
}

esp_err_t bluetooth_spp_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    s_client_handle = 0U;
    tx_queue_clear();
    s_init_pending = false;
    s_deinit_pending = true;
    (void)esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    esp_err_t result = esp_spp_deinit();
    if (result != ESP_OK) {
        s_deinit_pending = false;
        return result;
    }
    if (xSemaphoreTake(s_spp_uninit_done, pdMS_TO_TICKS(3000)) != pdTRUE) {
        s_deinit_pending = false;
        return ESP_ERR_TIMEOUT;
    }
    result = esp_bluedroid_disable();
    if (result != ESP_OK) return result;
    result = esp_bluedroid_deinit();
    if (result != ESP_OK) return result;
    result = esp_bt_controller_disable();
    if (result != ESP_OK) return result;
    result = esp_bt_controller_deinit();
    if (result != ESP_OK) return result;

    s_command_callback = NULL;
    return ESP_OK;
}

bool bluetooth_spp_initialized(void)
{
    return s_initialized;
}

const char *bluetooth_spp_device_name(void)
{
    return s_device_name;
}

uint16_t bluetooth_spp_board_number(void)
{
    return s_board_number;
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
    if (data == NULL || length == 0U) return;
    if (length > sizeof(s_write_buffer)) length = sizeof(s_write_buffer);

    bool send_now = false;
    uint32_t handle = 0U;
    portENTER_CRITICAL(&s_tx_lock);
    if (s_client_handle != 0U) {
        if (!s_write_busy && s_tx_queue_count == 0U) {
            memcpy(s_write_buffer, data, length);
            s_write_busy = true;
            handle = s_client_handle;
            send_now = true;
        } else if (s_tx_queue_count < BLUETOOTH_SPP_TX_QUEUE_DEPTH) {
            memcpy(s_tx_queue[s_tx_queue_tail], data, length);
            s_tx_queue_length[s_tx_queue_tail] = (uint16_t)length;
            s_tx_queue_tail = (uint8_t)((s_tx_queue_tail + 1U) % BLUETOOTH_SPP_TX_QUEUE_DEPTH);
            ++s_tx_queue_count;
        }
    }
    portEXIT_CRITICAL(&s_tx_lock);

    if (send_now && esp_spp_write(handle, (int)length, s_write_buffer) != ESP_OK) {
        portENTER_CRITICAL(&s_tx_lock);
        s_write_busy = false;
        portEXIT_CRITICAL(&s_tx_lock);
        tx_kick();
    }
}
