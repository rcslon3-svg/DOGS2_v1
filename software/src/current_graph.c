#include "current_graph.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int32_t s_a_ma[CURRENT_GRAPH_POINTS];
static int32_t s_b_ma[CURRENT_GRAPH_POINTS];
static bool s_a_valid[CURRENT_GRAPH_POINTS];
static bool s_b_valid[CURRENT_GRAPH_POINTS];
static size_t s_next;
static size_t s_count;
static uint8_t s_decimation = 1U;
static uint8_t s_divider;
static uint32_t s_sequence;

void current_graph_configure(uint8_t decimation)
{
    if (decimation == 0U) decimation = 1U;

    portENTER_CRITICAL(&s_lock);
    s_decimation = decimation;
    if (s_divider >= s_decimation) s_divider = 0U;
    portEXIT_CRITICAL(&s_lock);
}

void current_graph_reset(void)
{
    portENTER_CRITICAL(&s_lock);
    memset(s_a_ma, 0, sizeof(s_a_ma));
    memset(s_b_ma, 0, sizeof(s_b_ma));
    memset(s_a_valid, 0, sizeof(s_a_valid));
    memset(s_b_valid, 0, sizeof(s_b_valid));
    s_next = 0U;
    s_count = 0U;
    s_divider = 0U;
    ++s_sequence;
    portEXIT_CRITICAL(&s_lock);
}

void current_graph_add_ina_sample(const app_state_t *state)
{
    if (state == NULL) return;

    portENTER_CRITICAL(&s_lock);
    ++s_divider;
    if (s_divider < s_decimation) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    s_divider = 0U;

    s_a_ma[s_next] = state->ina238.channel[0].current_ma;
    s_b_ma[s_next] = state->ina238.channel[1].current_ma;
    s_a_valid[s_next] = state->ina238.channel[0].valid;
    s_b_valid[s_next] = state->ina238.channel[1].valid;

    s_next = (s_next + 1U) % CURRENT_GRAPH_POINTS;
    if (s_count < CURRENT_GRAPH_POINTS) ++s_count;
    ++s_sequence;
    portEXIT_CRITICAL(&s_lock);
}

uint32_t current_graph_sequence(void)
{
    uint32_t sequence;
    portENTER_CRITICAL(&s_lock);
    sequence = s_sequence;
    portEXIT_CRITICAL(&s_lock);
    return sequence;
}

void current_graph_snapshot(current_graph_snapshot_t *out)
{
    if (out == NULL) return;

    portENTER_CRITICAL(&s_lock);
    size_t start = s_count < CURRENT_GRAPH_POINTS ? 0U : s_next;
    out->count = s_count;
    out->sequence = s_sequence;
    for (size_t i = 0U; i < s_count; ++i) {
        size_t src = (start + i) % CURRENT_GRAPH_POINTS;
        out->a_ma[i] = s_a_ma[src];
        out->b_ma[i] = s_b_ma[src];
        out->a_valid[i] = s_a_valid[src];
        out->b_valid[i] = s_b_valid[src];
    }
    if (s_count < CURRENT_GRAPH_POINTS) {
        size_t rest = CURRENT_GRAPH_POINTS - s_count;
        memset(&out->a_ma[s_count], 0, rest * sizeof(out->a_ma[0]));
        memset(&out->b_ma[s_count], 0, rest * sizeof(out->b_ma[0]));
        memset(&out->a_valid[s_count], 0, rest * sizeof(out->a_valid[0]));
        memset(&out->b_valid[s_count], 0, rest * sizeof(out->b_valid[0]));
    }
    portEXIT_CRITICAL(&s_lock);
}
