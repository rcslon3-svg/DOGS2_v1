#include "calibration.h"

#include <string.h>

#include "nvs.h"

#define CALIBRATION_NVS_NAMESPACE "calib"

static calibration_data_t s_data;
static bool s_have_data;
static bool s_start_requested;
static bool s_running;
static bool s_done;
static char s_progress_channel;
static uint16_t s_progress_target_mv;
static uint32_t s_progress_measured_mv;
static int64_t s_progress_measured_current_ua;
static uint32_t s_progress_sample_count;

static size_t channel_count(uint8_t channel)
{
    return channel == CALIBRATION_CHANNEL_A ? 4U : 6U;
}

static const calibration_point_t *channel_points(uint8_t channel)
{
    return channel == CALIBRATION_CHANNEL_A ? s_data.a : s_data.b;
}

static int32_t interpolate_i32(uint32_t x,
                               uint32_t x0,
                               int32_t y0,
                               uint32_t x1,
                               int32_t y1)
{
    if (x1 == x0) return y0;
    int64_t dy = (int64_t)y1 - (int64_t)y0;
    int64_t dx = (int64_t)x1 - (int64_t)x0;
    return (int32_t)((int64_t)y0 + dy * ((int64_t)x - (int64_t)x0) / dx);
}

static int64_t interpolate_i64(uint32_t x,
                               uint32_t x0,
                               int64_t y0,
                               uint32_t x1,
                               int64_t y1)
{
    if (x1 == x0) return y0;
    int64_t dy = y1 - y0;
    int64_t dx = (int64_t)x1 - (int64_t)x0;
    return y0 + dy * ((int64_t)x - (int64_t)x0) / dx;
}

static int32_t clamp_voltage_correction(int32_t correction_mv)
{
    if (correction_mv > CALIBRATION_VOLTAGE_LIMIT_MV) return CALIBRATION_VOLTAGE_LIMIT_MV;
    if (correction_mv < -CALIBRATION_VOLTAGE_LIMIT_MV) return -CALIBRATION_VOLTAGE_LIMIT_MV;
    return correction_mv;
}

void calibration_load(void)
{
    nvs_handle_t nvs;
    size_t size = sizeof(s_data);
    memset(&s_data, 0, sizeof(s_data));
    s_have_data = false;

    if (nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    if (nvs_get_blob(nvs, "data", &s_data, &size) == ESP_OK &&
        size == sizeof(s_data)) {
        s_have_data = true;
    }
    nvs_close(nvs);
}

esp_err_t calibration_save(const calibration_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(nvs, "data", data, sizeof(*data));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        s_data = *data;
        s_have_data = true;
    }
    return err;
}

bool calibration_current_available(uint8_t channel)
{
    if (!s_have_data || channel > CALIBRATION_CHANNEL_B) return false;

    const calibration_point_t *points = channel_points(channel);
    size_t count = channel_count(channel);
    for (size_t i = 0U; i < count; ++i) {
        if (points[i].valid) return true;
    }
    return false;
}

int64_t calibration_correct_current_ua(uint8_t channel,
                                       uint32_t measured_mv,
                                       int64_t current_ua)
{
    if (!s_have_data || channel > CALIBRATION_CHANNEL_B) return current_ua;

    const calibration_point_t *points = channel_points(channel);
    size_t count = channel_count(channel);
    const calibration_point_t *previous = NULL;

    for (size_t i = 0U; i < count; ++i) {
        if (!points[i].valid) continue;
        if (measured_mv <= points[i].measured_mv) {
            int64_t offset = points[i].measured_current_ua;
            if (previous != NULL) {
                offset = interpolate_i64(measured_mv,
                                         previous->measured_mv,
                                         previous->measured_current_ua,
                                         points[i].measured_mv,
                                         points[i].measured_current_ua);
            }
            return current_ua - offset;
        }
        previous = &points[i];
    }

    return previous != NULL ? current_ua - previous->measured_current_ua : current_ua;
}

int32_t calibration_voltage_correction_mv(uint8_t channel, uint16_t target_mv)
{
    if (!s_have_data || channel > CALIBRATION_CHANNEL_B) return 0;

    const calibration_point_t *points = channel_points(channel);
    size_t count = channel_count(channel);
    const calibration_point_t *previous = NULL;

    for (size_t i = 0U; i < count; ++i) {
        if (!points[i].valid) continue;
        int32_t error = (int32_t)points[i].target_mv - (int32_t)points[i].measured_mv;
        if (target_mv <= points[i].target_mv) {
            if (previous == NULL) return clamp_voltage_correction(error);
            int32_t previous_error =
                (int32_t)previous->target_mv - (int32_t)previous->measured_mv;
            return clamp_voltage_correction(interpolate_i32(target_mv,
                                                            previous->target_mv,
                                                            previous_error,
                                                            points[i].target_mv,
                                                            error));
        }
        previous = &points[i];
    }

    if (previous == NULL) return 0;
    return clamp_voltage_correction((int32_t)previous->target_mv -
                                    (int32_t)previous->measured_mv);
}

void calibration_request_start(void)
{
    s_start_requested = true;
}

bool calibration_take_start_request(void)
{
    bool requested = s_start_requested;
    s_start_requested = false;
    return requested;
}

bool calibration_running(void)
{
    return s_running;
}

void calibration_set_running(bool running)
{
    s_running = running;
}

void calibration_set_progress(char channel,
                              uint16_t target_mv,
                              uint32_t measured_mv,
                              int64_t measured_current_ua,
                              uint32_t sample_count)
{
    s_progress_channel = channel;
    s_progress_target_mv = target_mv;
    s_progress_measured_mv = measured_mv;
    s_progress_measured_current_ua = measured_current_ua;
    s_progress_sample_count = sample_count;
}

void calibration_get_progress(char *channel,
                              uint16_t *target_mv,
                              uint32_t *measured_mv,
                              int64_t *measured_current_ua,
                              uint32_t *sample_count)
{
    if (channel != NULL) *channel = s_progress_channel;
    if (target_mv != NULL) *target_mv = s_progress_target_mv;
    if (measured_mv != NULL) *measured_mv = s_progress_measured_mv;
    if (measured_current_ua != NULL) *measured_current_ua = s_progress_measured_current_ua;
    if (sample_count != NULL) *sample_count = s_progress_sample_count;
}

void calibration_mark_done(void)
{
    s_done = true;
}

bool calibration_done(void)
{
    return s_done;
}

void calibration_clear_done(void)
{
    s_done = false;
}
