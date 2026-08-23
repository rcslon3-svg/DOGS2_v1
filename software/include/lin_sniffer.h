#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*lin_sniffer_line_cb_t)(const char *line, bool truncated);

void lin_sniffer_configure(uint32_t baud,
                           uint16_t mask_value,
                           uint8_t mask_care);
void lin_sniffer_reset(void);
void lin_sniffer_on_break(void);
void lin_sniffer_on_byte(uint8_t byte, int64_t now_us, lin_sniffer_line_cb_t callback);
void lin_sniffer_poll(int64_t now_us, lin_sniffer_line_cb_t callback);
