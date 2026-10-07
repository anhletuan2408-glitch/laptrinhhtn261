/**
 * @file    hall_port_esp32.h
 * @brief   ESP-IDF GPIO interrupt glue for the Hall wheel sensor driver.
 */
#ifndef HALL_PORT_ESP32_H
#define HALL_PORT_ESP32_H

#ifdef ESP_PLATFORM

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "hall.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure @p pin as a Hall input and attach hall_on_pulse() to its interrupt.
 *
 * @param h          Hall object, already initialised with hall_init(); must outlive the ISR.
 * @param pin        GPIO number of the sensor output.
 * @param active_low true for open-collector sensors (A3144): pull-up + falling-edge interrupt;
 *                   false: rising-edge interrupt.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on bad arguments, or a driver error code.
 */
esp_err_t hall_port_esp32_init(hall_t *h, gpio_num_t pin, bool active_low);

#ifdef __cplusplus
}
#endif

#endif /* ESP_PLATFORM */

#endif /* HALL_PORT_ESP32_H */
