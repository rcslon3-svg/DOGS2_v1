#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_state.h"

#define CURRENT_GRAPH_POINTS 260U
#define CURRENT_GRAPH_CHANNEL_A 0x01U
#define CURRENT_GRAPH_CHANNEL_B 0x02U

typedef struct {
    int32_t a_ma[CURRENT_GRAPH_POINTS];
    int32_t b_ma[CURRENT_GRAPH_POINTS];
    bool a_valid[CURRENT_GRAPH_POINTS];
    bool b_valid[CURRENT_GRAPH_POINTS];
    size_t count;
    uint32_t sequence;
} current_graph_snapshot_t;

void current_graph_configure(uint8_t decimation);
void current_graph_reset(void);
void current_graph_add_ina_sample(const app_state_t *state);
uint32_t current_graph_sequence(void);
void current_graph_snapshot(current_graph_snapshot_t *out);
