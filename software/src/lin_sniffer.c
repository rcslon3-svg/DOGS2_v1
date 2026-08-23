#include "lin_sniffer.h"

#include <stdio.h>
#include <string.h>

#include "probe_config.h"

#define LIN_FRAME_BYTES 16U
#define LIN_LINE_CHARS 128U
#define LIN_TIMEOUT_MIN_US 2000LL

typedef enum {
    LIN_WAIT_BREAK = 0,
    LIN_WAIT_SYNC,
    LIN_WAIT_PID,
    LIN_COLLECT_DATA,
} lin_state_t;

static lin_state_t s_state = LIN_WAIT_BREAK;
static uint32_t s_baud = 19200U;
static uint16_t s_mask_value;
static uint8_t s_mask_care;
static uint8_t s_pid;
static uint8_t s_data[LIN_FRAME_BYTES];
static size_t s_data_used;
static bool s_truncated;
static bool s_pid_valid;
static int64_t s_last_byte_us;

static int64_t frame_timeout_us(void)
{
    uint32_t baud = s_baud == 0U ? 19200U : s_baud;
    int64_t timeout = (int64_t)((30ULL * 1000000ULL + baud - 1ULL) / baud);
    return timeout < LIN_TIMEOUT_MIN_US ? LIN_TIMEOUT_MIN_US : timeout;
}

static uint8_t lin_id_from_pid(uint8_t pid)
{
    return (uint8_t)(pid & 0x3FU);
}

static bool lin_pid_parity_ok(uint8_t pid)
{
    uint8_t id0 = (uint8_t)((pid >> 0U) & 1U);
    uint8_t id1 = (uint8_t)((pid >> 1U) & 1U);
    uint8_t id2 = (uint8_t)((pid >> 2U) & 1U);
    uint8_t id3 = (uint8_t)((pid >> 3U) & 1U);
    uint8_t id4 = (uint8_t)((pid >> 4U) & 1U);
    uint8_t id5 = (uint8_t)((pid >> 5U) & 1U);
    uint8_t p0 = (uint8_t)(id0 ^ id1 ^ id2 ^ id4);
    uint8_t p1 = (uint8_t)(~(id1 ^ id3 ^ id4 ^ id5) & 1U);
    return (((pid >> 6U) & 1U) == p0) && (((pid >> 7U) & 1U) == p1);
}

static bool address_matches_mask(uint8_t address)
{
    for (uint8_t i = 0U; i < 2U; ++i) {
        if ((s_mask_care & (uint8_t)(1U << i)) == 0U) continue;
        uint8_t shift = (uint8_t)((1U - i) * 4U);
        uint8_t want = (uint8_t)((s_mask_value >> shift) & 0x0FU);
        uint8_t have = (uint8_t)((address >> shift) & 0x0FU);
        if (want != have) return false;
    }
    return true;
}

static uint8_t checksum_sum(const uint8_t *bytes, size_t count)
{
    uint16_t sum = 0U;
    for (size_t i = 0U; i < count; ++i) {
        sum = (uint16_t)(sum + bytes[i]);
        if (sum > 0xFFU) sum = (uint16_t)((sum & 0xFFU) + 1U);
    }
    return (uint8_t)~sum;
}

static bool checksum_classic_ok(const uint8_t *data, size_t data_count, uint8_t received)
{
    return checksum_sum(data, data_count) == received;
}

static bool checksum_enhanced_ok(uint8_t pid,
                                 const uint8_t *data,
                                 size_t data_count,
                                 uint8_t received)
{
    uint8_t bytes[LIN_FRAME_BYTES + 1U];
    bytes[0] = pid;
    if (data_count > LIN_FRAME_BYTES) data_count = LIN_FRAME_BYTES;
    memcpy(&bytes[1], data, data_count);
    return checksum_sum(bytes, data_count + 1U) == received;
}

static void publish_frame(lin_sniffer_line_cb_t callback)
{
    char line[LIN_LINE_CHARS];
    size_t used = 0U;
    uint8_t id = lin_id_from_pid(s_pid);
    bool diagnostic = id == 0x3CU || id == 0x3DU;
    bool classic_ok;
    bool enhanced_ok;

    if (callback == NULL || s_state != LIN_COLLECT_DATA) {
        lin_sniffer_reset();
        return;
    }
    if (!address_matches_mask(id)) {
        lin_sniffer_reset();
        return;
    }

    used = (size_t)snprintf(line,
                            sizeof(line),
                            "P:%02X",
                            s_pid);
    if (s_data_used == 0U) {
        if (used + 5U < sizeof(line)) {
            memcpy(line + used, " HDR", 5U);
        }
        line[sizeof(line) - 1U] = '\0';
        callback(line, false);
        lin_sniffer_reset();
        return;
    }

    uint8_t received_checksum = s_data[s_data_used - 1U];
    size_t payload_count = s_data_used - 1U;
    for (size_t i = 0U; i < payload_count && used + 4U < sizeof(line); ++i) {
        int written = snprintf(line + used,
                               sizeof(line) - used,
                               " %02X",
                               s_data[i]);
        if (written <= 0) break;
        used += (size_t)written;
    }

    if (used + 24U < sizeof(line)) {
        int written;
        if (!s_pid_valid) {
            written = snprintf(line + used, sizeof(line) - used, " PAR:ERR");
        } else {
            classic_ok = checksum_classic_ok(s_data, payload_count, received_checksum);
            enhanced_ok = checksum_enhanced_ok(s_pid, s_data, payload_count, received_checksum);
            if (diagnostic && classic_ok) {
                written = snprintf(line + used, sizeof(line) - used, " CRC_CL:OK");
            } else if (!diagnostic && enhanced_ok) {
                written = snprintf(line + used, sizeof(line) - used, " CRC_ENH:OK");
            } else {
                written = snprintf(line + used, sizeof(line) - used, " CRC:ERR");
            }
        }
        if (written > 0) used += (size_t)written;
    }

    line[sizeof(line) - 1U] = '\0';
    callback(line, s_truncated);
    lin_sniffer_reset();
}

void lin_sniffer_configure(uint32_t baud, uint16_t mask_value, uint8_t mask_care)
{
    s_baud = baud == 0U ? 19200U : baud;
    s_mask_value = mask_value;
    s_mask_care = mask_care;
}

void lin_sniffer_reset(void)
{
    s_state = LIN_WAIT_BREAK;
    s_pid = 0U;
    s_data_used = 0U;
    s_truncated = false;
    s_pid_valid = false;
    s_last_byte_us = 0LL;
}

void lin_sniffer_on_break(void)
{
    s_state = LIN_WAIT_SYNC;
    s_pid = 0U;
    s_data_used = 0U;
    s_truncated = false;
    s_pid_valid = false;
    s_last_byte_us = 0LL;
}

void lin_sniffer_on_byte(uint8_t byte, int64_t now_us, lin_sniffer_line_cb_t callback)
{
    switch (s_state) {
        case LIN_WAIT_BREAK:
            if (byte == 0x00U) {
                lin_sniffer_on_break();
            } else if (byte == 0x55U) {
                s_state = LIN_WAIT_PID;
                s_last_byte_us = now_us;
            }
            break;
        case LIN_WAIT_SYNC:
            if (byte == 0x55U) {
                s_state = LIN_WAIT_PID;
                s_last_byte_us = now_us;
            } else if (byte == 0x00U) {
                s_last_byte_us = 0LL;
            } else {
                lin_sniffer_reset();
            }
            break;
        case LIN_WAIT_PID:
            s_pid = byte;
            s_pid_valid = lin_pid_parity_ok(byte);
            s_data_used = 0U;
            s_state = LIN_COLLECT_DATA;
            s_last_byte_us = now_us;
            break;
        case LIN_COLLECT_DATA:
            if (s_data_used < sizeof(s_data)) {
                s_data[s_data_used++] = byte;
            } else {
                s_truncated = true;
            }
            s_last_byte_us = now_us;
            break;
        default:
            lin_sniffer_reset();
            break;
    }
    lin_sniffer_poll(now_us, callback);
}

void lin_sniffer_poll(int64_t now_us, lin_sniffer_line_cb_t callback)
{
    if (s_state != LIN_COLLECT_DATA || s_last_byte_us == 0LL) return;
    if (now_us - s_last_byte_us < frame_timeout_us()) return;
    publish_frame(callback);
}
