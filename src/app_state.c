#include "app_state.h"

/* app_logic_state_name
 * Inputs:
 *   state - logic classification enum.
 * Returns: static ASCII name for display/telemetry.
 * Does: converts LOW/HIGH/MID/OPEN/OVERVOLTAGE enum values to text.
 */
const char *app_logic_state_name(probe_logic_state_t state)
{
    switch (state) {
        case PROBE_LOW: return "LOW";
        case PROBE_HIGH: return "HIGH";
        case PROBE_OPEN: return "OPEN";
        case PROBE_OVERVOLTAGE: return "OVERVOLT";
        default: return "MID";
    }
}

/* app_event_name
 * Inputs:
 *   event - timing event enum.
 * Returns: static ASCII glyph/name for telemetry.
 * Does: converts timing event enum values to compact edge/pulse text.
 */
const char *app_event_name(probe_event_t event)
{
    switch (event) {
        case PROBE_EVENT_RISE: return "_|-";
        case PROBE_EVENT_FALL: return "-|_";
        case PROBE_EVENT_HIGH_PULSE: return "_|-|_";
        case PROBE_EVENT_LOW_PULSE: return "-|_|-";
        default: return "";
    }
}
