#include "i2c_worker.h"

#include "channelA.h"
#include "channelB.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "ina238_monitor.h"

#define I2C_WORKER_PERIOD_MS 100U
#define I2C_RECOVERY_MIN_INTERVAL_US 1000000LL

static const char *TAG = "i2c_worker";
static app_state_t *s_state;
static int64_t s_last_recovery_us;

static bool i2c_errors_visible(const app_state_t *state)
{
    if (state->tps55289.last_error != ESP_OK) return true;
    if (state->lm51772.last_error != ESP_OK) return true;
    return false;
}

static void recover_i2c_if_needed(app_state_t *state, int64_t now_us)
{
    if (!i2c_errors_visible(state)) return;
    bool lm_error = state->lm51772.last_error != ESP_OK;
    if (!lm_error && now_us - s_last_recovery_us < I2C_RECOVERY_MIN_INTERVAL_US) return;
    s_last_recovery_us = now_us;

    ++state->i2c.recovery_count;
    state->i2c.last_recovery_error = ESP_OK;

    ESP_LOGW(TAG, "I2C recovery #%lu: release devices, release controller, recover lines, recreate bus, chip init",
             (unsigned long)state->i2c.recovery_count);

    ina238_monitor_i2c_release();
    channelA_i2c_release();
    channelB_i2c_release();

    esp_err_t err = i2c_bus_release();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = i2c_bus_recover_lines();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = ina238_monitor_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = channelA_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;

    err = channelB_init();
    if (err != ESP_OK) state->i2c.last_recovery_error = err;
}

static void i2c_worker_task(void *argument)
{
    app_state_t *state = (app_state_t *)argument;

    while (true) {
        int64_t now = esp_timer_get_time();

        ina238_monitor_update(state, now);
        channelA_update(state, now);
        channelB_update(state, now);
        recover_i2c_if_needed(state, now);

        vTaskDelay(pdMS_TO_TICKS(I2C_WORKER_PERIOD_MS));
    }
}

esp_err_t i2c_worker_start(app_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    s_state = state;

    BaseType_t ok = xTaskCreate(i2c_worker_task, "i2c_worker", 4096, s_state, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
