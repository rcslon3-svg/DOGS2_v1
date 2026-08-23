#include <stdio.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "buzzer.h"
#include "calibration.h"
#include "can_probe.h"
#include "channelA.h"
#include "channelB.h"
#include "control.h"
#include "display.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "i2c_i2s_sniffer.h"
#include "i2c_master_terminal.h"
#include "i2c_sniffer.h"
#include "i2c_worker.h"
#include "ina238_monitor.h"
#include "io26_diag.h"
#include "nvs_flash.h"
#include "analog_probe.h"
#include "probe_config.h"
#include "timing_input.h"
#include "uart_probe.h"

/*
 * Main scheduling model
 * ---------------------
 * app_main() intentionally does not read measurement hardware directly. Fast
 * edge timing is captured asynchronously by the MCPWM ISR inside
 * timing_input.c. UART A RX runs in its own FreeRTOS task inside uart_probe.c.
 * Bluetooth SPP also calls
 * back from its stack-owned task and only enqueues command bytes.
 *
 * The main loop is a cooperative scheduler:
 *
 *   every loop, about each 2 ms:
 *     - timing_input_update(&state, now)
 *       Copies capture/PCNT results into state.timing.
 *     - analog_probe_update(&state, timing_quiet)
 *       Reads the ADC window and updates state.analog.
 *     - uart_probe_update(&state)
 *       Copies UART RX text/errors into state.uart.
 *
 *   every UI_PERIOD_MS:
 *     - display_render()
 *       Draws only from the local snapshot.  The display does not read hardware.
 *
 *   every loop:
 *     - queued command processing.
 *
 * Result storage map
 * ------------------
 * timing_input.c:
 *   - ISR-owned volatile counters: edge count, last high/low/period, min/max.
 *   - task-owned derived state: last activity time, event hold timer.
 *   - output type: app_timing_state_t, copied by timing_input_update().
 *
 * analog_probe.c:
 *   - ADC calibration handle.
 *   - OPEN/prozvonka latch counters and last test amplitude.
 *   - output storage: state.analog.
 *
 * uart_probe.c:
 *   - last completed UART A display line.
 *   - UART A receive error counter.
 *   - output storage: state.uart.
 *
 * ina238_monitor.c:
 *   - I2C device addresses, shunt values and selected shunt ADC range.
 *   - output storage: state.ina238.
 *
 * channelA.c:
 *   - takes U2/I2 setpoints from state.control.
 *   - programs/publishes TPS55289 channel-A output state.
 *
 * channelB.c:
 *   - takes U1/I1 setpoints from state.control.
 *   - programs/publishes LM51772 channel-B output state.
 *
 * display.c:
 *   - previous rendered strings and rounded voltage values, used only to avoid
 *     flicker and redraw unchanged lines.
 */

static const char *TAG = "logic_probe";
static QueueHandle_t s_commands;
static char s_command_line[32];
static size_t s_command_line_used;
static int64_t s_command_line_last_us;
static app_state_t s_state;
static bool s_i2c_i2s_sniffer_active;
static app_mode_t s_i2c_gpio_configured_mode = APP_MODE_POWER_SUPPLY;
static app_mode_t s_bluetooth_configured_mode = APP_MODE_POWER_SUPPLY;

static void bluetooth_command(char command);

#define BOOT_STAGE_MAGIC 0xD0952001UL

typedef enum {
    BOOT_STAGE_UNKNOWN = 0,
    BOOT_STAGE_APP_MAIN,
    BOOT_STAGE_NVS_OK,
    BOOT_STAGE_IO23_CONFIGURED_INACTIVE,
    BOOT_STAGE_IO23_ON,
    BOOT_STAGE_IO23_ON_DONE,
    BOOT_STAGE_BT_INIT,
    BOOT_STAGE_BT_INIT_DONE,
    BOOT_STAGE_DISPLAY_INIT,
    BOOT_STAGE_I2C_INIT,
    BOOT_STAGE_RUN,
} boot_stage_t;

static RTC_NOINIT_ATTR uint32_t s_boot_stage_magic;
static RTC_NOINIT_ATTR uint32_t s_boot_stage;

static const char *boot_stage_name(uint32_t stage)
{
    switch ((boot_stage_t)stage) {
        case BOOT_STAGE_APP_MAIN:                 return "APP_MAIN";
        case BOOT_STAGE_NVS_OK:                   return "NVS_OK";
        case BOOT_STAGE_IO23_CONFIGURED_INACTIVE: return "IO23_CONFIGURED_INACTIVE";
        case BOOT_STAGE_IO23_ON:                  return "IO23_ON";
        case BOOT_STAGE_IO23_ON_DONE:             return "IO23_ON_DONE";
        case BOOT_STAGE_BT_INIT:                  return "BT_INIT";
        case BOOT_STAGE_BT_INIT_DONE:             return "BT_INIT_DONE";
        case BOOT_STAGE_DISPLAY_INIT:             return "DISPLAY_INIT";
        case BOOT_STAGE_I2C_INIT:                 return "I2C_INIT";
        case BOOT_STAGE_RUN:                      return "RUN";
        default:                                  return "UNKNOWN";
    }
}

static void boot_stage_set(boot_stage_t stage)
{
    s_boot_stage_magic = BOOT_STAGE_MAGIC;
    s_boot_stage = (uint32_t)stage;
}

static int64_t measured_current_ua(char channel, const app_ina238_channel_t *measurement)
{
    if (measurement == NULL || !measurement->valid || measurement->shunt_uohm == 0U) return 0;
    int64_t current_ua = measurement->current_ua;
    uint8_t cal_channel = channel == 'A' ? CALIBRATION_CHANNEL_A : CALIBRATION_CHANNEL_B;
    if (calibration_current_available(cal_channel)) {
        return calibration_correct_current_ua(cal_channel, measurement->bus_mv, current_ua);
    }
    if (channel == 'A') {
        current_ua -= ((int64_t)measurement->bus_mv * CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_NUM +
                       CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_DEN / 2LL) /
                      CHANNEL_A_DIVIDER_LEAK_UA_PER_MV_DEN;
    }
    return current_ua;
}

static void power_telemetry_update(const app_state_t *state, int64_t now_us)
{
    static int64_t last_telemetry_us;

    if (state == NULL ||
        state->control.mode != APP_MODE_1WIRE ||
        !bluetooth_spp_connected()) {
        last_telemetry_us = now_us;
        return;
    }
    if (now_us - last_telemetry_us < TELEMETRY_PERIOD_MS * 1000LL) return;
    last_telemetry_us = now_us;

    const app_ina238_channel_t *a = &state->ina238.channel[0];
    const app_ina238_channel_t *b = &state->ina238.channel[1];
    int64_t ia_ua = measured_current_ua('A', a);
    int64_t ib_ua = measured_current_ua('B', b);
    char line[48];

    int length = snprintf(line,
                          sizeof(line),
                          "E%lld,%lld\r\n",
                          (long long)ia_ua,
                          (long long)ib_ua);
    if (length > 0) bluetooth_spp_write(line, (size_t)length);
}

static void bluetooth_mode_update(app_mode_t mode)
{
    static int64_t next_init_retry_us;
    static int64_t next_deinit_retry_us;
    int64_t now_us = esp_timer_get_time();

    if (mode == APP_MODE_POWER_SUPPLY) {
        if (!bluetooth_spp_initialized()) {
            s_bluetooth_configured_mode = APP_MODE_POWER_SUPPLY;
            return;
        }
        if (s_bluetooth_configured_mode == APP_MODE_POWER_SUPPLY && now_us < next_deinit_retry_us) {
            return;
        }
        esp_err_t result = bluetooth_spp_deinit();
        s_bluetooth_configured_mode = APP_MODE_POWER_SUPPLY;
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Bluetooth deinit: %s", esp_err_to_name(result));
            next_deinit_retry_us = now_us + 1000000LL;
        } else {
            ESP_LOGI(TAG, "Bluetooth off in Power Source");
            next_deinit_retry_us = now_us;
        }
        return;
    }

    if (!bluetooth_spp_initialized()) {
        if (now_us < next_init_retry_us) return;
        boot_stage_set(BOOT_STAGE_BT_INIT);
        esp_err_t result = bluetooth_spp_init(bluetooth_command);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Bluetooth init: %s", esp_err_to_name(result));
            next_init_retry_us = now_us + 1000000LL;
            return;
        } else {
            ESP_LOGI(TAG, "Bluetooth device: %s", bluetooth_spp_device_name());
        }
        boot_stage_set(BOOT_STAGE_BT_INIT_DONE);
    }

    s_bluetooth_configured_mode = mode;
}

static const char *mode_hint_text(app_mode_t mode)
{
    switch (mode) {
        case APP_MODE_UART:
            return "[UART HINT] ASCII mode only. BT input -> UART TX. UART RX text preview -> [UART]. Select baud on screen.\r\n";
        case APP_MODE_LIN:
            return "[LIN HINT] Sniffer only. Select baud/mask on screen. RX -> [LIN]P:<PID> <data...> CRC_CL:OK/CRC_ENH:OK/PAR:ERR/CRC:ERR.\r\n";
        case APP_MODE_CAN:
            return "[CAN HINT] Send ASCII HEX: 4 ID digits + 0..8 data bytes. Example 0404112233 -> ID 0x404 data 11 22 33. RX -> [CAN].\r\n";
        case APP_MODE_RS485:
            return "[RS485 HINT] Send byte values as HEX. RX -> [485] HEX. Select baud on screen.\r\n";
        case APP_MODE_I2C:
            return "[I2C SNIFF HINT] Sniffer only. Select mask on screen: -- all, 4- = 0x40..0x4F, 41 = only 0x41. RX -> [I2C].\r\n";
        case APP_MODE_I2C_MASTER:
            return "[I2C MASTER HINT] ASCII commands: S | R addr reg len | W addr reg data... | RR addr len | WW addr data... . addr/reg/data HEX, len decimal.\r\n";
        default:
            return NULL;
    }
}

static void mode_hint_update(app_mode_t mode)
{
    static app_mode_t hinted_mode = APP_MODE_POWER_SUPPLY;
    static bool hint_sent;
    static bool bt_was_connected;

    bool bt_connected = bluetooth_spp_connected();
    bool mode_changed = mode != hinted_mode;
    bool connected_now = bt_connected && !bt_was_connected;

    if (mode == APP_MODE_POWER_SUPPLY) {
        hinted_mode = mode;
        hint_sent = false;
        bt_was_connected = bt_connected;
        return;
    }

    if (mode_changed) {
        hinted_mode = mode;
        hint_sent = false;
    }

    if (bt_connected && !hint_sent && (mode_changed || connected_now)) {
        const char *hint = mode_hint_text(mode);
        if (hint != NULL) {
            bluetooth_spp_write(hint, strlen(hint));
        }
        hint_sent = true;
    }

    if (!bt_connected) hint_sent = false;
    bt_was_connected = bt_connected;
}

static void warning_tone_update(const app_state_t *state, int64_t now_us)
{
    static bool overheat_latched;
    static bool overpower_latched;
    static int64_t next_overheat_us;
    static int64_t next_overpower_us;

    const app_ina238_channel_t *a = &state->ina238.channel[0];
    const app_ina238_channel_t *b = &state->ina238.channel[1];

    bool temperature_valid = a->valid || b->valid;
    int32_t max_temperature_mc = 0;
    if (a->valid && b->valid) {
        max_temperature_mc = a->temperature_mc > b->temperature_mc ?
                             a->temperature_mc : b->temperature_mc;
    } else if (a->valid) {
        max_temperature_mc = a->temperature_mc;
    } else if (b->valid) {
        max_temperature_mc = b->temperature_mc;
    }

    int32_t overheat_on_mc = (int32_t)state->control.overheat_c * 1000;
    int32_t overheat_off_mc = overheat_on_mc - 1000;
    if (temperature_valid && max_temperature_mc >= overheat_on_mc) {
        overheat_latched = true;
    } else if (!temperature_valid || max_temperature_mc <= overheat_off_mc) {
        overheat_latched = false;
        next_overheat_us = now_us;
    }

    int64_t total_power_mw = 0;
    if (a->valid && state->control.channel_a_enabled) {
        int64_t current_ua = measured_current_ua('A', a);
        if (current_ua > 0) total_power_mw += ((int64_t)a->bus_mv * current_ua) / 1000000LL;
    }
    if (b->valid && state->control.channel_b_enabled) {
        int64_t current_ua = measured_current_ua('B', b);
        if (current_ua > 0) total_power_mw += ((int64_t)b->bus_mv * current_ua) / 1000000LL;
    }

    if (state->control.overpower_w == 0U) {
        overpower_latched = false;
        next_overpower_us = now_us;
    } else {
        int64_t overpower_on_mw = (int64_t)state->control.overpower_w * 1000LL;
        int64_t overpower_off_mw = overpower_on_mw - 5000LL;
        if (overpower_off_mw < 0) overpower_off_mw = 0;
        if (total_power_mw >= overpower_on_mw) {
            overpower_latched = true;
        } else if (total_power_mw <= overpower_off_mw) {
            overpower_latched = false;
            next_overpower_us = now_us;
        }
    }

    if (overpower_latched && now_us >= next_overpower_us) {
        buzzer_play_overpower_warning(state->control.volume_percent);
        next_overpower_us = now_us + 3000000LL;
        return;
    }

    if (overheat_latched && now_us >= next_overheat_us) {
        buzzer_play_overheat_warning(state->control.volume_percent);
        next_overheat_us = now_us + 5000000LL;
    }
}

static uint64_t gpio_output_mask(gpio_num_t gpio)
{
    if (gpio == GPIO_NUM_NC) return 0ULL;
    return 1ULL << gpio;
}

static esp_err_t board_power_init(void)
{
    uint64_t pin_mask = gpio_output_mask(BOARD_PERIPHERAL_POWER_GPIO);
    if (pin_mask == 0ULL) return ESP_OK;
    int inactive_level = BOARD_PERIPHERAL_POWER_ACTIVE_LEVEL ? 0 : 1;

    gpio_set_level(BOARD_PERIPHERAL_POWER_GPIO, inactive_level);
    gpio_config_t output = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output), TAG, "peripheral power gpio");
    gpio_set_level(BOARD_PERIPHERAL_POWER_GPIO, inactive_level);
    ESP_LOGI(TAG, "peripheral power gpio %d inactive level %d",
             (int)BOARD_PERIPHERAL_POWER_GPIO,
             inactive_level);
    boot_stage_set(BOOT_STAGE_IO23_CONFIGURED_INACTIVE);
    return ESP_OK;
}

static void board_power_enable(void)
{
    if (BOARD_PERIPHERAL_POWER_GPIO == GPIO_NUM_NC) return;
#if BOARD_PERIPHERAL_POWER_AUTO_ENABLE
    boot_stage_set(BOOT_STAGE_IO23_ON);
    ESP_LOGI(TAG, "peripheral power gpio %d active level %d",
             (int)BOARD_PERIPHERAL_POWER_GPIO,
             BOARD_PERIPHERAL_POWER_ACTIVE_LEVEL);
    gpio_set_level(BOARD_PERIPHERAL_POWER_GPIO, BOARD_PERIPHERAL_POWER_ACTIVE_LEVEL);
    boot_stage_set(BOOT_STAGE_IO23_ON_DONE);
#else
    ESP_LOGW(TAG, "peripheral power auto enable disabled");
#endif
}

/* enqueue_command
 * Inputs:
 *   command - one command byte received from Bluetooth or USB UART.
 * Returns: none.
 * Does: places the command into the main-loop queue so command handling never
 * runs inside a Bluetooth callback or blocking input task.
 */
static void enqueue_command(char command)
{
    if (s_commands != NULL) xQueueSend(s_commands, &command, pdMS_TO_TICKS(10));
}

static void bluetooth_command(char command)
{
    if (can_probe_input_char(command)) return;
    if (i2c_master_terminal_input_char(command)) return;

#if UART_PROBE_ENABLED
    if (s_state.control.mode == APP_MODE_UART ||
        s_state.control.mode == APP_MODE_LIN ||
        s_state.control.mode == APP_MODE_RS485) {
        uart_probe_write_bytes(&command, 1U);
        return;
    }
#endif
    enqueue_command(command);
}

/* uart_command_task
 * Inputs:
 *   argument - unused FreeRTOS task argument.
 * Returns: never returns.
 * Does: reads command characters from USB stdin and forwards them to the same
 * queue that Bluetooth commands use.
 */
static void uart_command_task(void *argument)
{
    (void)argument;
    while (true) {
        int ch = getchar();
        if (ch != EOF) enqueue_command((char)ch);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* process_command_line
 * Inputs:
 *   command - zero-terminated command string.
 * Returns: none.
 * Does: runs whole-line commands. IO26 diagnostic commands are delegated to
 * io26_diag.c.
 */
static void process_command_line(const char *command)
{
    if (command == NULL || command[0] == '\0') return;

    if (command[0] == 'i' || command[0] == 'I') {
        char line[192];
        size_t used = 0U;
        used += snprintf(line + used, sizeof(line) - used, "I2C:");
        for (uint8_t address = 0x03U; address <= 0x77U && used + 6U < sizeof(line); ++address) {
            if (i2c_bus_probe(address) == ESP_OK) {
                used += snprintf(line + used, sizeof(line) - used, " %02X", address);
            }
        }
        snprintf(line + used, sizeof(line) - used, "\r\n");
        printf("%s", line);
        if (bluetooth_spp_connected()) bluetooth_spp_write(line, strlen(line));
        return;
    }

#if IO26_DIAG_ENABLED
    bool handled_by_io26 = io26_diag_handle_command(command);
    if (handled_by_io26 &&
        (command[0] == 'r' || command[0] == 'R' ||
         command[0] == 'v' || command[0] == 'V' ||
         command[0] == 'f' || command[0] == 'F' ||
         command[0] == 'u' || command[0] == 'U')) {
#if ANALOG_PROBE_ENABLED
        analog_probe_reset();
#endif
#if TIMING_INPUT_ENABLED
        timing_input_reset();
#endif
    }
#endif
}

/* command_line_can_auto_run
 * Inputs: none.
 * Returns: true when the current buffered command is complete enough to run.
 * Does: supports Bluetooth terminal apps that send text without CR/LF.
 */
static bool command_line_can_auto_run(void)
{
    if (s_command_line_used < 2U) return false;

    char command = s_command_line[0];
    if (command != 'f' && command != 'F') return false;

    bool saw_dash = false;
    bool saw_duty_digit = false;
    for (size_t i = 1; i < s_command_line_used; ++i) {
        char ch = s_command_line[i];
        if (ch == '-') {
            saw_dash = true;
        } else if (saw_dash && ch >= '0' && ch <= '9') {
            saw_duty_digit = true;
        }
    }
    return saw_dash && saw_duty_digit;
}

/* process_buffered_command_if_idle
 * Inputs:
 *   now_us - current esp_timer_get_time().
 * Returns: none.
 * Does: runs complete Bluetooth-style f commands after input becomes idle.
 */
static void process_buffered_command_if_idle(int64_t now_us)
{
    if (s_command_line_used == 0U || !command_line_can_auto_run()) return;
    if (now_us - s_command_line_last_us < 200000LL) return;

    s_command_line[s_command_line_used] = '\0';
    process_command_line(s_command_line);
    s_command_line_used = 0U;
}

/* process_command_char
 * Inputs:
 *   command - one byte from Bluetooth or USB console.
 * Returns: none.
 * Does: collects line-oriented commands and executes them on CR/LF. Single
 * character h/r commands are also executed immediately when the line buffer is
 * empty.
 */
static void process_command_char(char command)
{
    if (command == '\r' || command == '\n') {
        if (s_command_line_used != 0U) {
            s_command_line[s_command_line_used] = '\0';
            process_command_line(s_command_line);
            s_command_line_used = 0U;
        }
        return;
    }

    if (s_command_line_used == 0U &&
        (command == 'h' || command == 'H' || command == 'r' || command == 'R')) {
        char one[2] = {command, '\0'};
        process_command_line(one);
        return;
    }

    if (s_command_line_used + 1U < sizeof(s_command_line)) {
        s_command_line[s_command_line_used++] = command;
        s_command_line_last_us = esp_timer_get_time();
    } else {
        s_command_line_used = 0U;
        const char *error = "ERR command too long\r\n";
        printf("%s", error);
        if (bluetooth_spp_connected()) bluetooth_spp_write(error, strlen(error));
    }
}

/* app_main
 * Inputs: none; called by ESP-IDF.
 * Returns: never returns.
 * Does: initializes all modules, then runs the cooperative scheduler described
 * at the top of this file.
 */
void app_main(void)
{
    uint32_t previous_stage = s_boot_stage_magic == BOOT_STAGE_MAGIC ?
                              s_boot_stage : BOOT_STAGE_UNKNOWN;
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "boot Logic_v2 app_main");
    ESP_LOGI(TAG,
             "reset reason=%d previous boot stage=%s",
             (int)reset_reason,
             boot_stage_name(previous_stage));
    boot_stage_set(BOOT_STAGE_APP_MAIN);
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs);
    }
    boot_stage_set(BOOT_STAGE_NVS_OK);
    ESP_ERROR_CHECK(board_power_init());
    calibration_load();

    s_commands = xQueueCreate(64, sizeof(char));
    xTaskCreate(uart_command_task, "uart_cmd", 3072, NULL, 4, NULL);

    board_power_enable();

    memset(&s_state, 0, sizeof(s_state));
    s_state.analog.logic_state = PROBE_UNDEFINED;
    ESP_LOGI(TAG, "state initialized");

#if TIMING_INPUT_ENABLED
    ESP_LOGI(TAG, "timing_input_init");
    ESP_ERROR_CHECK(timing_input_init());
#endif
#if ANALOG_PROBE_ENABLED
    ESP_LOGI(TAG, "analog_probe_init");
    ESP_ERROR_CHECK(analog_probe_init());
#endif
    ESP_LOGI(TAG, "display_init begin");
    boot_stage_set(BOOT_STAGE_DISPLAY_INIT);
    ESP_ERROR_CHECK(display_init());
    ESP_LOGI(TAG, "display_init end");
#if IO26_DIAG_ENABLED
    ESP_LOGI(TAG, "io26_diag_init begin");
    ESP_ERROR_CHECK(io26_diag_init());
    ESP_LOGI(TAG, "io26_diag_init end");
#endif
#if UART_PROBE_ENABLED && !I2C_SNIFFER_OWNS_UART_PINS
    ESP_LOGI(TAG, "uart_probe_init");
    ESP_ERROR_CHECK(uart_probe_init());
#endif
#if I2C_SNIFFER_ENABLED
    ESP_LOGI(TAG, "i2c_i2s_sniffer_init");
    esp_err_t i2c_i2s_sniffer_result = i2c_i2s_sniffer_init();
    if (i2c_i2s_sniffer_result == ESP_OK) {
        s_i2c_i2s_sniffer_active = true;
    } else {
        ESP_LOGW(TAG,
                 "i2c_i2s_sniffer_init failed: %s, falling back to gpio sniffer",
                 esp_err_to_name(i2c_i2s_sniffer_result));
        ESP_ERROR_CHECK(i2c_sniffer_init());
    }
#endif
    ESP_LOGI(TAG, "i2c_master_terminal_init");
    ESP_ERROR_CHECK(i2c_master_terminal_init());
    ESP_LOGI(TAG, "can_probe_init");
    ESP_ERROR_CHECK(can_probe_init());
    ESP_LOGI(TAG, "buzzer_init begin");
    ESP_ERROR_CHECK(buzzer_init());
    ESP_LOGI(TAG, "buzzer_init end");
    ESP_LOGI(TAG, "control_init begin");
    ESP_ERROR_CHECK(control_init());
    ESP_LOGI(TAG, "control_init end");
    ESP_LOGI(TAG, "ina238_monitor_init begin");
    boot_stage_set(BOOT_STAGE_I2C_INIT);
    ESP_ERROR_CHECK(ina238_monitor_init());
    ESP_LOGI(TAG, "ina238_monitor_init end");
    ESP_LOGI(TAG, "channelA_init begin");
    ESP_ERROR_CHECK(channelA_init());
    ESP_LOGI(TAG, "channelA_init end");
    ESP_LOGI(TAG, "channelB_init begin");
    ESP_ERROR_CHECK(channelB_init());
    ESP_LOGI(TAG, "channelB_init end");
    ESP_LOGI(TAG, "i2c_worker_start begin");
    ESP_ERROR_CHECK(i2c_worker_start(&s_state));
    ESP_LOGI(TAG, "i2c_worker_start end");

    int64_t last_ui_us = 0;
    display_black();
    ESP_LOGI(TAG, "enter main loop");
    boot_stage_set(BOOT_STAGE_RUN);

    while (true) {
        int64_t now = esp_timer_get_time();
#if TIMING_INPUT_ENABLED
        timing_input_update(&s_state, (uint32_t)now);
#endif
#if ANALOG_PROBE_ENABLED
        analog_probe_update(&s_state, false);
#endif
        control_update(&s_state);
        bluetooth_mode_update(s_state.control.mode);
        mode_hint_update(s_state.control.mode);
#if UART_PROBE_ENABLED && !I2C_SNIFFER_OWNS_UART_PINS
        uart_probe_configure(s_state.control.mode,
                             s_state.control.uart_baud,
                             s_state.control.lin_baud,
                             s_state.control.rs485_baud,
                             s_state.control.lin_mask_value,
                             s_state.control.lin_mask_care);
        uart_probe_update(&s_state);
#endif
#if I2C_SNIFFER_ENABLED
        /*
         * Mode changes have two phases for modules sharing the external I2C
         * pins: first every previous owner is told to leave, then modules are
         * configured for the new mode.  No module needs to know which module
         * will own the pins next.
         */
        if (s_state.control.mode != s_i2c_gpio_configured_mode) {
            i2c_master_terminal_configure(APP_MODE_POWER_SUPPLY);
            if (s_i2c_i2s_sniffer_active) {
                i2c_i2s_sniffer_configure(APP_MODE_POWER_SUPPLY,
                                          s_state.control.i2c_mask_value,
                                          s_state.control.i2c_mask_care);
            } else {
                i2c_sniffer_configure(APP_MODE_POWER_SUPPLY,
                                      s_state.control.i2c_mask_value,
                                      s_state.control.i2c_mask_care);
            }
            s_i2c_gpio_configured_mode = s_state.control.mode;
        }

        i2c_master_terminal_configure(s_state.control.mode);
        if (s_i2c_i2s_sniffer_active) {
            i2c_i2s_sniffer_configure(s_state.control.mode,
                                      s_state.control.i2c_mask_value,
                                      s_state.control.i2c_mask_care);
        } else {
            i2c_sniffer_configure(s_state.control.mode,
                                  s_state.control.i2c_mask_value,
                                  s_state.control.i2c_mask_care);
        }
        if (s_i2c_i2s_sniffer_active) i2c_i2s_sniffer_update(&s_state);
        else i2c_sniffer_update(&s_state);
#else
        i2c_master_terminal_configure(s_state.control.mode);
#endif
        i2c_master_terminal_update(&s_state);
        can_probe_configure(s_state.control.mode,
                            s_state.control.can_bitrate,
                            s_state.control.can_mask_value,
                            s_state.control.can_mask_care);
        can_probe_update(&s_state, now);
        control_persistence_update(now);
        buzzer_set_volume(s_state.control.volume_percent);
        warning_tone_update(&s_state, now);
        buzzer_update();
        power_telemetry_update(&s_state, now);

        if (now - last_ui_us >= UI_PERIOD_MS * 1000LL) {
            last_ui_us = now;
            display_render(&s_state);
        }
        char command;
        while (xQueueReceive(s_commands, &command, 0) == pdTRUE) process_command_char(command);
        process_buffered_command_if_idle(now);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
