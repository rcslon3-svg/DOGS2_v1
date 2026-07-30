#include "i2c_bus.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "probe_config.h"

static i2c_master_bus_handle_t s_bus;

static void set_i2c_gpio_drive_min(void)
{
    (void)gpio_set_drive_capability(INA238_I2C_SDA_GPIO, GPIO_DRIVE_CAP_0);
    (void)gpio_set_drive_capability(INA238_I2C_SCL_GPIO, GPIO_DRIVE_CAP_0);
}

/* i2c_bus_get
 * Inputs:
 *   bus - output pointer for the shared I2C master bus handle.
 * Returns: ESP_OK when the bus exists, otherwise an ESP-IDF error.
 * Does: creates the single I2C bus used by INA238, TPS55289/MCP4725 and
 * LM51772. Repeated calls return the same handle.
 */
esp_err_t i2c_bus_get(i2c_master_bus_handle_t *bus)
{
    if (bus == NULL) return ESP_ERR_INVALID_ARG;

    if (s_bus == NULL) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = INA238_I2C_PORT,
            .sda_io_num = INA238_I2C_SDA_GPIO,
            .scl_io_num = INA238_I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus), "i2c_bus", "new bus");
        set_i2c_gpio_drive_min();
    }

    *bus = s_bus;
    return ESP_OK;
}

/* i2c_bus_probe
 * Inputs: 7-bit I2C address.
 * Returns: ESP_OK if a device ACKs the address, otherwise an ESP-IDF error.
 * Does: performs an address-only probe on the shared I2C bus.
 */
esp_err_t i2c_bus_probe(uint8_t address)
{
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_bus_get(&bus);
    if (err != ESP_OK) return err;
    return i2c_master_probe(bus, address, 100);
}

/* i2c_bus_recover_lines
 * Inputs: none.
 * Returns: ESP_OK if bus recovery pulses were generated.
 * Does: performs physical I2C bus recovery, not controller reset:
 *   - disconnects the pins from I2C by configuring them as GPIO open-drain;
 *   - releases both lines high;
 *   - if SDA is stuck low, clocks SCL up to 9 times;
 *   - generates a STOP condition: SDA low while SCL high, then SDA high.
 *
 * This is for a slave holding SDA low after an interrupted byte. It is separate
 * from i2c_master_bus_reset(), which only resets the ESP32 I2C controller/FSM.
 */
esp_err_t i2c_bus_recover_lines(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << INA238_I2C_SDA_GPIO) | (1ULL << INA238_I2C_SCL_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), "i2c_bus", "recover gpio");
    set_i2c_gpio_drive_min();

    gpio_set_level(INA238_I2C_SDA_GPIO, 1);
    gpio_set_level(INA238_I2C_SCL_GPIO, 1);
    esp_rom_delay_us(5);

    for (int i = 0; i < 9 && gpio_get_level(INA238_I2C_SDA_GPIO) == 0; ++i) {
        gpio_set_level(INA238_I2C_SCL_GPIO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(INA238_I2C_SCL_GPIO, 1);
        esp_rom_delay_us(5);
    }

    gpio_set_level(INA238_I2C_SDA_GPIO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(INA238_I2C_SCL_GPIO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(INA238_I2C_SDA_GPIO, 1);
    esp_rom_delay_us(5);

    return ESP_OK;
}

/* i2c_bus_reset_controller
 * Inputs: none.
 * Returns: ESP_OK or ESP-IDF I2C reset error.
 * Does: resets the ESP32 I2C master controller/FSM for the existing bus handle.
 * This is not a physical bus recovery; call i2c_bus_recover_lines() first when
 * SDA/SCL may be held by an external slave.
 */
esp_err_t i2c_bus_reset_controller(void)
{
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_bus_get(&bus), "i2c_bus", "get for reset");
    return i2c_master_bus_reset(bus);
}

/* i2c_bus_release
 * Inputs: none.
 * Returns: ESP_OK or ESP-IDF delete error.
 * Does: deletes the ESP32 I2C master bus handle and forgets it. All device
 * handles must be removed by their owning modules before calling this.
 */
esp_err_t i2c_bus_release(void)
{
    if (s_bus == NULL) return ESP_OK;

    esp_err_t err = i2c_del_master_bus(s_bus);
    if (err == ESP_OK) {
        s_bus = NULL;
    }
    return err;
}
