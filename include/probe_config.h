#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* ESP32-WROOM board with external ST7789 1.9" 170x320 SPI TFT. */
#define PROBE_ADC_CHANNEL       ADC_CHANNEL_0 /* GPIO36 / SENSOR_VP */
#define PROBE_ADC_GPIO          GPIO_NUM_36
#define PROBE_COMPARATOR_GPIO   GPIO_NUM_22
#define PROBE_BIAS_GPIO         GPIO_NUM_27
#define PROBE_TEST_SIGNAL_GPIO  GPIO_NUM_26

/* The old probe functions are disabled while the power-channel hardware is
 * being debugged.
 */
#define ANALOG_PROBE_ENABLED    1
#define TIMING_INPUT_ENABLED    0
#define UART_PROBE_ENABLED      1
#define IO26_DIAG_ENABLED       0

/* Panel controls. Internal pull-ups are enabled in encoder_input.c/control.c. */
#define ENCODER_A_GPIO          GPIO_NUM_13
#define ENCODER_B_GPIO          GPIO_NUM_27
#define ENCODER_BUTTON_GPIO     GPIO_NUM_14
#define UI_BUTTON_GPIO          GPIO_NUM_NC
#define MODE_BUTTON_GPIO        GPIO_NUM_32
#define GENERATOR_OUTPUT_GPIO   GPIO_NUM_23

/* UART experiment. */
#define UART_A_RX_GPIO          GPIO_NUM_19
#define UART_A_TX_GPIO          GPIO_NUM_18
#define UART_TEST_BAUD          115200
#define UART_PREVIEW_CHARS      15
#define UART_DISPLAY_CHARS      (UART_PREVIEW_CHARS + 3U)

/* Passive I2C sniffer. External jumpers:
 *   IO26/SCL -> IO21
 *   IO25/SDA -> IO22
 */
#define I2C_SNIFFER_ENABLED     1
#define I2C_SNIFFER_SCL_GPIO    GPIO_NUM_21
#define I2C_SNIFFER_SDA_GPIO    GPIO_NUM_22
/* I2S1 drives BCK on IO19; I2S0 samples that same pad as its external clock. */
#define I2C_SNIFFER_I2S_CLOCK_OUT_GPIO GPIO_NUM_19
#define I2C_SNIFFER_I2S_CLOCK_IN_GPIO  GPIO_NUM_19
#define I2C_SNIFFER_OWNS_UART_PINS     1

/* External I2C board. */
#define INA238_I2C_ENABLED      1
#define INA238_I2C_PORT         I2C_NUM_0
#define INA238_I2C_SDA_GPIO     GPIO_NUM_25
#define INA238_I2C_SCL_GPIO     GPIO_NUM_26
#define INA238_I2C_HZ           100000U
#define INA238_1_ADDRESS        0x41U
#define INA238_2_ADDRESS        0x44U
#define INA238_1_SHUNT_UOHM     100000U
#define INA238_2_SHUNT_UOHM     10000U

/* TPS55289 buck-boost output controlled by channel 2 setpoints.
 *
 * TPS55289 I2C address is selected by MODE pin:
 *   MODE high -> 0x74
 *   MODE low  -> 0x75
 *
 * I2C is still disabled together with the INA238 bus. Set TPS55289_I2C_ENABLED
 * to 1 when SDA/SCL are assigned and the bus is wired.
 *
 * CHANNEL_A_CURRENT_SENSE_UOHM must match the resistor between ISP and ISN.
 * It is left as an explicit board constant because the current limit register
 * programs shunt voltage, not current directly.
 */
#define TPS55289_I2C_ENABLED        INA238_I2C_ENABLED
#define TPS55289_I2C_PORT           INA238_I2C_PORT
#define TPS55289_I2C_SDA_GPIO       INA238_I2C_SDA_GPIO
#define TPS55289_I2C_SCL_GPIO       INA238_I2C_SCL_GPIO
#define TPS55289_I2C_HZ             INA238_I2C_HZ
#define TPS55289_ADDRESS            0x75U
#define CHANNEL_A_CURRENT_SENSE_UOHM 10000U

/* Channel A auxiliary LDO reference DAC.
 *
 * MCP4725A0T at 0x60 drives the TPS73801 REF/FB node through 3.6 kOhm.
 * The same node has 20 kOhm to the LDO output and 2 kOhm to ground.
 * The DAC is used to make the LDO output track U2. TPS55289 itself is
 * programmed to U2 + 1.00 V.
 */
#define CHANNEL_A_LDO_DAC_ENABLED       TPS55289_I2C_ENABLED
#define CHANNEL_A_MCP4725_ADDRESS       0x60U
#define CHANNEL_A_MCP4725_VDD_MV        3300U
#define CHANNEL_A_LDO_FB_REF_MV         1210U
#define CHANNEL_A_LDO_OFFSET_MV         1000U
#define CHANNEL_A_LDO_R_TOP_OHM         20000U
#define CHANNEL_A_LDO_R_BOTTOM_OHM      2000U
#define CHANNEL_A_LDO_R_DAC_OHM         3600U

/* Hidden output-voltage correction.
 *
 * The UI setpoint stays unchanged. The power-channel drivers add a small
 * internal correction to remove static setpoint error and load-dependent
 * output drop. Integral trim is frozen while the channel is in CC mode.
 */
#define OUTPUT_VOLTAGE_TRIM_ENABLED     1
#define OUTPUT_VOLTAGE_TRIM_MAX_MV      300
#define OUTPUT_VOLTAGE_TRIM_DEADBAND_MV 15
#define OUTPUT_VOLTAGE_TRIM_STEP_MV     2
#define OUTPUT_VOLTAGE_TRIM_GAIN_DIV    16
#define CHANNEL_A_LOAD_COMP_MOHM        30
#define CHANNEL_B_LOAD_COMP_MOHM        30

/* LM51772 channel B buck-boost output.
 *
 * ADDR/SLOPE tied to GND selects I2C address 0x6A.
 * U1/I1 setpoints are used for this channel.
 */
#define CHANNEL_B_I2C_ENABLED          INA238_I2C_ENABLED
#define CHANNEL_B_I2C_PORT             INA238_I2C_PORT
#define CHANNEL_B_I2C_SDA_GPIO         INA238_I2C_SDA_GPIO
#define CHANNEL_B_I2C_SCL_GPIO         INA238_I2C_SCL_GPIO
#define CHANNEL_B_I2C_HZ               INA238_I2C_HZ
#define CHANNEL_B_LM51772_ADDRESS      0x6AU
#define CHANNEL_B_INDUCTOR_SHUNT_UOHM  2500U
#define CHANNEL_B_OUTPUT_SHUNT_UOHM    10000U
#define CHANNEL_B_CURRENT_LIMIT_MAX_MA 4000U

#define RGB_LED_ENABLED         0
#define RGB_RED_GPIO            GPIO_NUM_NC
#define RGB_GREEN_GPIO          GPIO_NUM_NC
#define RGB_BLUE_GPIO           GPIO_NUM_NC
#define RGB_ACTIVE_LEVEL        0

#define TFT_MISO_GPIO           GPIO_NUM_NC
#define TFT_MOSI_GPIO           GPIO_NUM_2   /* display SDA */
#define TFT_SCLK_GPIO           GPIO_NUM_15  /* display SCL */
#define TFT_RESET_GPIO          GPIO_NUM_4   /* display RES */
#define TFT_DC_GPIO             GPIO_NUM_16
#define TFT_CS_GPIO             GPIO_NUM_17
#define TFT_BACKLIGHT_GPIO      GPIO_NUM_5   /* display BLK */
#define TFT_BACKLIGHT_ACTIVE_LEVEL 1
#define TFT_WIDTH               320
#define TFT_HEIGHT              170
#define TFT_X_OFFSET            0
#define TFT_Y_OFFSET            35

/*
 * Input-voltage monitor on GPIO36:
 *
 *   VIN -- 102k -- GPIO36/ADC -- 12k -- GND
 *
 * Vin = Vadc * (102k + 12k) / 12k = Vadc * 9.5.
 */
#define ADC_INPUT_SCALE         9.5f
#define PROBE_ADC_ATTEN         ADC_ATTEN_DB_12
#define LOGIC_LOW_MAX_MV        800U
#define LOGIC_HIGH_MIN_MV       2600U
#define TIP_OVERVOLTAGE_MV      15000U

/* GPIO27 drives a weak test level through 300 kOhm.
 * analog_probe.c measures ADC according to current GPIO27 state, stores the
 * sample in the matching LOW/HIGH bucket, then toggles GPIO27.
 */
#define OPEN_TEST_DELTA_MV      70U
#define EXPECTED_OPEN_TEST_MV   304U
#define BIAS_RESISTOR_OHM       300000U
#define ANALOG_SAMPLE_PERIOD_MS 10U
#define ANALOG_WINDOW_SAMPLES   10U
#define ANALOG_ADC_READS        16U
#define ANALOG_VPP_AVG_WINDOWS  5U
#define OPEN_TEST_QUIET_MS      250U

/* Display anti-chatter: keep the last shown value while only the least
 * significant digit is jumping in ADC noise.
 */
#define DISPLAY_VOLTAGE_STEP_MV 20U
#define DISPLAY_VPP_STEP_MV     30U
#define CAPTURE_TIMER_HZ        1000000U
#define PCNT_GATE_MS            250U
#define PULSE_PAIR_MAX_US       300000U
#define PULSE_EVENT_HOLD_MS     800U
#define MIN_PULSE_US            2U
/* Reject very short edge-to-edge intervals as capture noise/ringing.  This is
 * intentionally small enough for the current 10 kHz test signal, whose half
 * period is 50 us.
 */
#define MIN_EDGE_INTERVAL_US    5U
#define SIGNAL_MISSING_MS       1000U
/* Above this rate the signal is treated as a continuous waveform/protocol
 * stream.  The screen should show FREQ/DUTY/UART text, not blink single-edge
 * EVENT UP/DN messages for every bit edge.
 */
#define EVENT_SINGLE_EDGE_MAX_HZ 1.0f
#define UI_PERIOD_MS            50U
#define TELEMETRY_PERIOD_MS     250U
#define BLUETOOTH_DEVICE_NAME   "ESP32-Logic-Probe"
