idf_component_register(
    SRCS "main.c" "app_state.c" "analog_probe.c" "timing_input.c" "uart_probe.c" "io26_diag.c" "control.c" "encoder_input.c" "generator_output.c" "i2c_bus.c" "i2c_worker.c" "ina238_monitor.c" "channelA.c" "channelB.c"
         "display.c" "bluetooth_spp.c" "smooth_font.c"
    INCLUDE_DIRS "../include"
    REQUIRES driver esp_adc esp_driver_dac esp_driver_pcnt esp_driver_uart esp_timer bt nvs_flash esp_driver_i2c
)
