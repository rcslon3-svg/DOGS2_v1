#include "io26_diag.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_spp.h"
#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "probe_config.h"

#define IO26_UART_PORT UART_NUM_2
#define IO26_LEDC_TIMER LEDC_TIMER_0
#define IO26_LEDC_CHANNEL LEDC_CHANNEL_0
#define IO26_LEDC_SOURCE_HZ 80000000U

static dac_oneshot_handle_t s_dac;
static esp_timer_handle_t s_slow_timer;
static uint32_t s_slow_high_us;
static uint32_t s_slow_low_us;
static bool s_slow_level;
static bool s_ledc_active;
static bool s_uart_active;
static uint32_t s_uart_baud;

static uint64_t io26_gpio_mask(void)
{
#if LOGIC_V2_BOARD_REV == LOGIC_V2_BOARD_ENGINEERING_SAMPLE
    return 0ULL;
#else
    if (PROBE_TEST_SIGNAL_GPIO == GPIO_NUM_NC) return 0ULL;
    return 1ULL << PROBE_TEST_SIGNAL_GPIO;
#endif
}

/* send_response
 * Inputs:
 *   text - zero-terminated response string.
 * Returns: none.
 * Does: writes command response to USB stdout and Bluetooth SPP when connected.
 */
static void send_response(const char *text)
{
    printf("%s", text);
    if (bluetooth_spp_connected()) bluetooth_spp_write(text, strlen(text));
}

/* io26_stop_peripherals
 * Inputs: none.
 * Returns: none.
 * Does: stops all peripherals that may currently drive IO26: DAC, LEDC,
 * esp_timer waveform generator and UART2.
 */
static void io26_stop_peripherals(void)
{
    if (s_slow_timer != NULL) (void)esp_timer_stop(s_slow_timer);
    s_slow_level = false;

    if (s_ledc_active) {
        ledc_stop(LEDC_LOW_SPEED_MODE, IO26_LEDC_CHANNEL, 0);
        s_ledc_active = false;
    }

    if (s_uart_active) {
        (void)uart_wait_tx_done(IO26_UART_PORT, pdMS_TO_TICKS(50));
        (void)uart_driver_delete(IO26_UART_PORT);
        s_uart_active = false;
    }

    if (s_dac != NULL) {
        (void)dac_oneshot_del_channel(s_dac);
        s_dac = NULL;
    }
}

/* io26_high_z
 * Inputs: none.
 * Returns: none.
 * Does: disconnects IO26 from output peripherals and configures it as a
 * floating input without pull-up or pull-down.
 */
static void io26_high_z(void)
{
    io26_stop_peripherals();
    uint64_t pin_mask = io26_gpio_mask();
    if (pin_mask == 0ULL) return;
    gpio_config_t pin = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    (void)gpio_config(&pin);
}

/* dac_code_from_mv
 * Inputs:
 *   mv - requested DAC output in millivolts.
 * Returns: 8-bit DAC code for an approximate 0..3.3 V output.
 * Does: maps a requested millivolt value to ESP32 DAC code space.
 */
static uint8_t dac_code_from_mv(uint32_t mv)
{
    if (mv >= 3300U) return 255U;
    return (uint8_t)((mv * 255U + 1650U) / 3300U);
}

/* io26_set_dac_mv
 * Inputs:
 *   mv - requested DAC output voltage in millivolts.
 * Returns: none.
 * Does: stops other IO26 modes, enables DAC2/GPIO26 and writes the requested
 * approximate DAC code.
 */
static void io26_set_dac_mv(uint32_t mv)
{
    if (mv > 3300U) {
        send_response("ERR v range is 0..3300 mV\r\n");
        return;
    }

    io26_stop_peripherals();
    esp_err_t err = dac_oneshot_new_channel(&(dac_oneshot_config_t) {
        .chan_id = DAC_CHAN_1
    }, &s_dac);
    if (err != ESP_OK) {
        char response[96];
        snprintf(response, sizeof(response), "ERR DAC26: %s\r\n", esp_err_to_name(err));
        send_response(response);
        return;
    }

    uint8_t code = dac_code_from_mv(mv);
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(s_dac, code));
    char response[96];
    snprintf(response, sizeof(response), "OK IO26 DAC approx %lu mV code=%u\r\n",
             (unsigned long)mv, code);
    send_response(response);
}

/* slow_timer_callback
 * Inputs:
 *   argument - unused esp_timer callback argument.
 * Returns: none.
 * Does: toggles IO26 for sub-Hz test waveforms and schedules the next edge.
 */
static void slow_timer_callback(void *argument)
{
    (void)argument;
    s_slow_level = !s_slow_level;
    gpio_set_level(PROBE_TEST_SIGNAL_GPIO, s_slow_level ? 1 : 0);
    uint32_t next_us = s_slow_level ? s_slow_high_us : s_slow_low_us;
    if (next_us < 20U) next_us = 20U;
    ESP_ERROR_CHECK(esp_timer_start_once(s_slow_timer, next_us));
}

/* configure_io26_gpio_output
 * Inputs: none.
 * Returns: none.
 * Does: configures IO26 as a plain push-pull GPIO output.
 */
static void configure_io26_gpio_output(void)
{
    uint64_t pin_mask = io26_gpio_mask();
    if (pin_mask == 0ULL) return;
    gpio_config_t pin = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&pin));
}

/* io26_start_slow_wave
 * Inputs:
 *   period_us - full waveform period.
 *   duty      - duty cycle in percent.
 * Returns: none.
 * Does: starts a low-frequency waveform on IO26 using esp_timer one-shots.
 */
static void io26_start_slow_wave(uint64_t period_us, uint32_t duty)
{
    io26_stop_peripherals();
    configure_io26_gpio_output();

    uint64_t high_us = period_us * duty / 100U;
    if (high_us == 0U) high_us = 1U;
    if (high_us >= period_us) high_us = period_us - 1U;
    s_slow_high_us = (uint32_t)high_us;
    s_slow_low_us = (uint32_t)(period_us - high_us);

    gpio_set_level(PROBE_TEST_SIGNAL_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    s_slow_level = true;
    gpio_set_level(PROBE_TEST_SIGNAL_GPIO, 1);
    ESP_ERROR_CHECK(esp_timer_start_once(s_slow_timer, s_slow_high_us));
}

/* io26_select_ledc_resolution
 * Inputs:
 *   frequency_hz - requested LEDC frequency in hertz.
 * Returns: highest duty resolution that fits the LEDC source clock.
 * Does: keeps high-frequency generator commands usable, including 1 MHz.
 */
static ledc_timer_bit_t io26_select_ledc_resolution(uint32_t frequency_hz)
{
    uint32_t bits = 10U;
    while (bits > 1U && ((uint64_t)frequency_hz << bits) > IO26_LEDC_SOURCE_HZ) {
        bits--;
    }
    return (ledc_timer_bit_t)bits;
}

/* io26_start_ledc_wave
 * Inputs:
 *   frequency_hz - waveform frequency in hertz.
 *   duty         - duty cycle in percent.
 * Returns: true when LEDC was configured.
 * Does: starts an LEDC PWM waveform on IO26.
 */
static bool io26_start_ledc_wave(uint32_t frequency_hz, uint32_t duty)
{
    io26_stop_peripherals();

    ledc_timer_bit_t resolution = io26_select_ledc_resolution(frequency_hz);
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = IO26_LEDC_TIMER,
        .duty_resolution = resolution,
        .freq_hz = frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        char response[96];
        snprintf(response, sizeof(response), "ERR LEDC timer: %s\r\n", esp_err_to_name(err));
        send_response(response);
        return false;
    }

    uint32_t duty_steps = 1UL << (uint32_t)resolution;
    uint32_t ledc_duty = (duty_steps * duty + 50U) / 100U;
    if (ledc_duty >= duty_steps) ledc_duty = duty_steps - 1U;
    ledc_channel_config_t channel = {
        .gpio_num = PROBE_TEST_SIGNAL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = IO26_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = IO26_LEDC_TIMER,
        .duty = ledc_duty,
        .hpoint = 0
    };
    err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        char response[96];
        snprintf(response, sizeof(response), "ERR LEDC channel: %s\r\n", esp_err_to_name(err));
        send_response(response);
        return false;
    }
    s_ledc_active = true;
    return true;
}

/* parse_frequency_hz
 * Inputs:
 *   text - frequency token from f command.
 *   end  - output pointer to the first non-frequency character.
 * Returns: frequency in hertz; 0 on parse failure.
 * Does: parses integer hertz tokens.
 */
static uint32_t parse_frequency_hz(const char *text, const char **end)
{
    char *local_end = NULL;
    unsigned long hz = strtoul(text, &local_end, 10);
    *end = local_end;
    if (local_end == text || hz == 0UL || hz > 1000000UL) return 0U;
    return (uint32_t)hz;
}

/* io26_set_frequency_command
 * Inputs:
 *   args - text after 'f', formatted as <freq>-<duty>.
 * Returns: none.
 * Does: parses and starts a frequency/duty waveform on IO26.
 */
static void io26_set_frequency_command(const char *args)
{
    const char *dash = NULL;
    uint32_t frequency_hz = parse_frequency_hz(args, &dash);
    if (frequency_hz == 0U || dash == NULL || *dash != '-') {
        send_response("ERR use fFREQ_HZ-DUTY_PERCENT\r\n");
        return;
    }
    if (frequency_hz < 10U || frequency_hz > 1000000U) {
        send_response("ERR frequency range is 10..1000000 Hz\r\n");
        return;
    }

    char *end = NULL;
    unsigned long duty = strtoul(dash + 1, &end, 10);
    if (end == dash + 1 || *end != '\0') {
        send_response("ERR use fFREQ_HZ-DUTY_PERCENT\r\n");
        return;
    }
    if (duty < 1UL || duty > 99UL) {
        send_response("ERR duty range is 1..99 percent\r\n");
        return;
    }

    uint64_t period_us = 1000000ULL / frequency_hz;
    uint64_t high_us = period_us * duty / 100UL;
    if (high_us == 0ULL) high_us = 1ULL;
    if (high_us >= period_us) high_us = period_us - 1ULL;
    uint64_t low_us = period_us - high_us;
    const char *engine = "LEDC";
    if (frequency_hz < 10U) {
        io26_start_slow_wave(period_us, (uint32_t)duty);
    } else {
        if (!io26_start_ledc_wave(frequency_hz, (uint32_t)duty)) return;
    }

    char response[96];
    snprintf(response, sizeof(response), "OK IO26 F=%luHz duty=%lu%% high=%llu us low=%llu us %s\r\n",
             (unsigned long)frequency_hz, duty, high_us, low_us, engine);
    send_response(response);
}

/* uart_tx_task
 * Inputs:
 *   argument - unused FreeRTOS task argument.
 * Returns: never returns.
 * Does: sends an incrementing UART test line while IO26 UART mode is active.
 */
static void uart_tx_task(void *argument)
{
    (void)argument;
    uint32_t counter = 0U;
    while (true) {
        if (s_uart_active) {
            char line[40];
            int length = snprintf(line, sizeof(line), "UARTB %lu\r\n", (unsigned long)counter++);
            if (length > 0) uart_write_bytes(IO26_UART_PORT, line, length);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* io26_set_uart_baud
 * Inputs:
 *   baud - UART baud rate to use on IO26.
 * Returns: none.
 * Does: stops other IO26 modes and attaches UART2 TX to IO26.
 */
static void io26_set_uart_baud(uint32_t baud)
{
    if (baud < 1200U || baud > 1000000U) {
        send_response("ERR UART baud out of range\r\n");
        return;
    }

    io26_stop_peripherals();
    const uart_config_t config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(IO26_UART_PORT, 256, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IO26_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(IO26_UART_PORT, PROBE_TEST_SIGNAL_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_uart_baud = baud;
    s_uart_active = true;

    char response[80];
    snprintf(response, sizeof(response), "OK IO26 UART TX %lu baud\r\n", (unsigned long)baud);
    send_response(response);
}

/* print_help
 * Inputs: none.
 * Returns: none.
 * Does: prints the IO26 diagnostic command list.
 */
static void print_help(void)
{
    send_response(
        "IO26 diagnostic commands:\r\n"
        "h        help\r\n"
        "r        reset IO26 to High-Z\r\n"
        "v0       DAC approx 0 mV\r\n"
        "v800     DAC approx 800 mV\r\n"
        "v1500    DAC approx 1500 mV\r\n"
        "v2700    DAC approx 2700 mV\r\n"
        "v3100    DAC approx 3100 mV\r\n"
        "fFREQ-DUTY 10..1000000 Hz, 1..99% duty\r\n"
        "f100-50    100 Hz 50% duty\r\n"
        "f1000-25   1000 Hz 25% duty\r\n"
        "f100000-75 100000 Hz 75% duty\r\n"
        "u115     UART TX 115200 baud\r\n"
        "u9       UART TX 9600 baud\r\n"
    );
}

/* io26_diag_init
 * Inputs: none.
 * Returns: ESP_OK on success, or an ESP-IDF error from esp_timer setup.
 * Does: creates the slow-waveform timer and UART TX task, then leaves IO26 in
 * High-Z. From this point this module is the only code allowed to drive IO26.
 */
esp_err_t io26_diag_init(void)
{
    ESP_RETURN_ON_ERROR(esp_timer_create(&(esp_timer_create_args_t) {
        .callback = slow_timer_callback,
        .name = "io26slow"
    }, &s_slow_timer), "io26_diag", "slow timer");
    xTaskCreate(uart_tx_task, "io26_uart_tx", 3072, NULL, 4, NULL);
    io26_high_z();
    return ESP_OK;
}

/* io26_diag_handle_command
 * Inputs:
 *   command - zero-terminated diagnostic command line.
 * Returns: true when the command belonged to the IO26 namespace.
 * Does: executes h/r/v/f/u commands. Any mode that drives IO26 is configured
 * here, after stopping the previous IO26 peripheral owner.
 */
bool io26_diag_handle_command(const char *command)
{
    if (command == NULL || command[0] == '\0') return false;

    if (strcmp(command, "h") == 0 || strcmp(command, "H") == 0) {
        print_help();
    } else if (strcmp(command, "r") == 0 || strcmp(command, "R") == 0) {
        io26_high_z();
        send_response("OK IO26 High-Z\r\n");
    } else if (command[0] == 'v' || command[0] == 'V') {
        char *end = NULL;
        unsigned long mv = strtoul(command + 1, &end, 10);
        if (end == command + 1 || *end != '\0') {
            send_response("ERR use v0, v800, v1500, v2700, v3100\r\n");
            return true;
        }
        io26_set_dac_mv((uint32_t)mv);
    } else if (command[0] == 'f' || command[0] == 'F') {
        io26_set_frequency_command(command + 1);
    } else if (command[0] == 'u' || command[0] == 'U') {
        if (strcmp(command + 1, "115") == 0) {
            io26_set_uart_baud(115200U);
        } else if (strcmp(command + 1, "9") == 0) {
            io26_set_uart_baud(9600U);
        } else {
            send_response("ERR use u115 or u9\r\n");
        }
    } else {
        send_response("ERR unknown command; use h\r\n");
        return false;
    }
    return true;
}
