#include "i2c_aux_bus.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "probe_config.h"

#define I2C_AUX_PORT I2C_NUM_1
#define I2C_AUX_HZ   100000U
#define I2C_AUX_HALF_PERIOD_US 5U
#define I2C_AUX_CLOCK_HIGH_TIMEOUT_US 1000U

static i2c_master_bus_handle_t s_bus;
static SemaphoreHandle_t s_lock;

static void set_i2c_gpio_drive_min(void)
{
    (void)gpio_set_drive_capability(I2C_SNIFFER_SDA_GPIO, GPIO_DRIVE_CAP_0);
    (void)gpio_set_drive_capability(I2C_SNIFFER_SCL_GPIO, GPIO_DRIVE_CAP_0);
}

static esp_err_t create_bus_locked(void)
{
    if (s_bus == NULL) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_AUX_PORT,
            .sda_io_num = I2C_SNIFFER_SDA_GPIO,
            .scl_io_num = I2C_SNIFFER_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus),
                            "i2c_aux_bus",
                            "new bus");
        set_i2c_gpio_drive_min();
    }
    return ESP_OK;
}

static esp_err_t delete_bus_locked(void)
{
    if (s_bus == NULL) return ESP_OK;
    esp_err_t err = i2c_del_master_bus(s_bus);
    if (err == ESP_OK) s_bus = NULL;
    return err;
}

static esp_err_t take_lock(void)
{
    if (s_lock == NULL) return ESP_ERR_INVALID_STATE;
    return xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t i2c_aux_bus_init(void)
{
    if (s_lock != NULL) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    return s_lock != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t i2c_aux_bus_start(void)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock start");
    esp_err_t err = create_bus_locked();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t i2c_aux_bus_stop(void)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock stop");
    esp_err_t err = delete_bus_locked();
    if (err == ESP_OK) {
        gpio_config_t input = {
            .pin_bit_mask = (1ULL << I2C_SNIFFER_SDA_GPIO) |
                            (1ULL << I2C_SNIFFER_SCL_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&input);
    }
    xSemaphoreGive(s_lock);
    return err;
}

static esp_err_t wait_scl_high(void)
{
    for (uint32_t elapsed = 0U; elapsed < I2C_AUX_CLOCK_HIGH_TIMEOUT_US; ++elapsed) {
        if (gpio_get_level(I2C_SNIFFER_SCL_GPIO) != 0) return ESP_OK;
        esp_rom_delay_us(1);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t bitbang_release_bus(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 1),
                        "i2c_aux_bus",
                        "release SDA");
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                        "i2c_aux_bus",
                        "release SCL");
    ESP_RETURN_ON_ERROR(wait_scl_high(), "i2c_aux_bus", "SCL held low");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    return gpio_get_level(I2C_SNIFFER_SDA_GPIO) != 0
               ? ESP_OK
               : ESP_ERR_INVALID_STATE;
}

static esp_err_t bitbang_start(void)
{
    ESP_RETURN_ON_ERROR(bitbang_release_bus(), "i2c_aux_bus", "bus not idle");
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 0),
                        "i2c_aux_bus",
                        "start SDA");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 0),
                        "i2c_aux_bus",
                        "start SCL");
    return ESP_OK;
}

static esp_err_t bitbang_stop(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 0),
                        "i2c_aux_bus",
                        "stop SDA");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                        "i2c_aux_bus",
                        "stop SCL");
    ESP_RETURN_ON_ERROR(wait_scl_high(), "i2c_aux_bus", "stop SCL held low");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 1),
                        "i2c_aux_bus",
                        "stop SDA release");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    return ESP_OK;
}

static esp_err_t bitbang_write_address(uint8_t address, bool *acked)
{
    uint8_t byte = (uint8_t)(address << 1);
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1) {
        ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO,
                                           (byte & mask) != 0U ? 1 : 0),
                            "i2c_aux_bus",
                            "address bit");
        esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
        ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                            "i2c_aux_bus",
                            "address clock high");
        ESP_RETURN_ON_ERROR(wait_scl_high(), "i2c_aux_bus", "address clock stretch");
        esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
        ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 0),
                            "i2c_aux_bus",
                            "address clock low");
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 1),
                        "i2c_aux_bus",
                        "ACK release SDA");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                        "i2c_aux_bus",
                        "ACK clock high");
    ESP_RETURN_ON_ERROR(wait_scl_high(), "i2c_aux_bus", "ACK clock stretch");
    esp_rom_delay_us(I2C_AUX_HALF_PERIOD_US);
    *acked = gpio_get_level(I2C_SNIFFER_SDA_GPIO) == 0;
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 0),
                        "i2c_aux_bus",
                        "ACK clock low");
    return ESP_OK;
}

esp_err_t i2c_aux_bus_scan(uint8_t *addresses,
                           size_t capacity,
                           size_t *address_count)
{
    if (addresses == NULL || address_count == NULL || capacity == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    *address_count = 0U;
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock scan");

    esp_err_t err = delete_bus_locked();
    if (err == ESP_OK) {
        /* Set released latch levels before enabling open-drain outputs. */
        (void)gpio_set_level(I2C_SNIFFER_SDA_GPIO, 1);
        (void)gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1);
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << I2C_SNIFFER_SDA_GPIO) |
                            (1ULL << I2C_SNIFFER_SCL_GPIO),
            .mode = GPIO_MODE_INPUT_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&io);
        set_i2c_gpio_drive_min();
    }

    if (err == ESP_OK) err = bitbang_release_bus();
    for (uint8_t address = 0x03U;
         err == ESP_OK && address <= 0x77U;
         ++address) {
        bool acked = false;
        err = bitbang_start();
        if (err == ESP_OK) err = bitbang_write_address(address, &acked);
        esp_err_t stop_err = bitbang_stop();
        if (err == ESP_OK) err = stop_err;
        if (err == ESP_OK && acked && *address_count < capacity) {
            addresses[(*address_count)++] = address;
        }
    }

    /* Never leave scan GPIO configuration behind, even after an error. */
    esp_err_t restore_err = create_bus_locked();
    if (err == ESP_OK) err = restore_err;
    xSemaphoreGive(s_lock);
    return err;
}

static esp_err_t add_device_locked(uint8_t address, i2c_master_dev_handle_t *dev)
{
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;
    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_AUX_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &config, dev);
}

esp_err_t i2c_aux_bus_transmit_receive(uint8_t address,
                                       const uint8_t *write_data,
                                       size_t write_length,
                                       uint8_t *read_data,
                                       size_t read_length,
                                       int timeout_ms)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock transmit_receive");
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = add_device_locked(address, &dev);
    if (err == ESP_OK) {
        err = i2c_master_transmit_receive(dev,
                                          write_data,
                                          write_length,
                                          read_data,
                                          read_length,
                                          timeout_ms);
        esp_err_t remove_err = i2c_master_bus_rm_device(dev);
        if (err == ESP_OK) err = remove_err;
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t i2c_aux_bus_receive(uint8_t address,
                              uint8_t *data,
                              size_t length,
                              int timeout_ms)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock receive");
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = add_device_locked(address, &dev);
    if (err == ESP_OK) {
        err = i2c_master_receive(dev, data, length, timeout_ms);
        esp_err_t remove_err = i2c_master_bus_rm_device(dev);
        if (err == ESP_OK) err = remove_err;
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t i2c_aux_bus_transmit(uint8_t address,
                               const uint8_t *data,
                               size_t length,
                               int timeout_ms)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock transmit");
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = add_device_locked(address, &dev);
    if (err == ESP_OK) {
        err = i2c_master_transmit(dev, data, length, timeout_ms);
        esp_err_t remove_err = i2c_master_bus_rm_device(dev);
        if (err == ESP_OK) err = remove_err;
    }
    xSemaphoreGive(s_lock);
    return err;
}

static esp_err_t recover_lines_locked(void)
{
    /*
     * The GPIO matrix is owned by the I2C driver while the bus handle exists.
     * Reconfiguring the pads behind the driver's back leaves its software state
     * valid but disconnects the peripheral, which makes the next transaction
     * unpredictable.  Delete the bus first and let i2c_aux_bus_get() recreate
     * and reattach it after recovery.
     */
    ESP_RETURN_ON_ERROR(delete_bus_locked(), "i2c_aux_bus", "release for recovery");

    /* Release both open-drain output latches before enabling their outputs. */
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 1),
                        "i2c_aux_bus",
                        "release SDA");
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                        "i2c_aux_bus",
                        "release SCL");

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << I2C_SNIFFER_SDA_GPIO) | (1ULL << I2C_SNIFFER_SCL_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), "i2c_aux_bus", "recover gpio");
    set_i2c_gpio_drive_min();

    esp_rom_delay_us(5);

    for (int i = 0; i < 9 && gpio_get_level(I2C_SNIFFER_SDA_GPIO) == 0; ++i) {
        ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 0),
                            "i2c_aux_bus",
                            "clock recovery low");
        esp_rom_delay_us(5);
        ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                            "i2c_aux_bus",
                            "clock recovery high");
        esp_rom_delay_us(5);
        if (gpio_get_level(I2C_SNIFFER_SCL_GPIO) == 0) return ESP_ERR_TIMEOUT;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 0),
                        "i2c_aux_bus",
                        "stop SDA low");
    esp_rom_delay_us(5);
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SCL_GPIO, 1),
                        "i2c_aux_bus",
                        "stop SCL high");
    esp_rom_delay_us(5);
    if (gpio_get_level(I2C_SNIFFER_SCL_GPIO) == 0) return ESP_ERR_TIMEOUT;
    ESP_RETURN_ON_ERROR(gpio_set_level(I2C_SNIFFER_SDA_GPIO, 1),
                        "i2c_aux_bus",
                        "stop SDA high");
    esp_rom_delay_us(5);

    if (gpio_get_level(I2C_SNIFFER_SDA_GPIO) == 0) return ESP_ERR_INVALID_STATE;

    return create_bus_locked();
}

esp_err_t i2c_aux_bus_recover_lines(void)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock recovery");
    esp_err_t err = recover_lines_locked();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t i2c_aux_bus_reset_controller(void)
{
    ESP_RETURN_ON_ERROR(take_lock(), "i2c_aux_bus", "lock reset");
    esp_err_t err = s_bus == NULL ? ESP_ERR_INVALID_STATE : i2c_master_bus_reset(s_bus);
    xSemaphoreGive(s_lock);
    return err;
}
