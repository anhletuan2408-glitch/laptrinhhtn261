/**
 * @file    mpu6050_port_esp32.h
 * @brief   ESP32 (ESP-IDF v5.2+, new I2C master driver) bus port for the MPU6050 driver.
 */
#ifndef MPU6050_PORT_ESP32_H
#define MPU6050_PORT_ESP32_H

#ifdef ESP_PLATFORM

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the I2C master bus, attach the MPU6050 and fill a bus descriptor.
 *
 * Internal pull-ups are enabled; the bus is created once (single-instance port,
 * static context). Call before mpu6050_init().
 *
 * @param bus     [out] descriptor to pass to mpu6050_init().
 * @param port    I2C controller number (e.g. I2C_NUM_0).
 * @param sda     SDA GPIO.
 * @param scl     SCL GPIO.
 * @param addr    7-bit device address (MPU6050_ADDR_AD0_LOW / _HIGH).
 * @param clk_hz  SCL frequency, e.g. 400000.
 * @return ESP_OK, ESP_ERR_INVALID_ARG or an ESP-IDF I2C error.
 */
esp_err_t mpu6050_port_esp32_init(mpu6050_bus_t *bus, i2c_port_num_t port,
                                  gpio_num_t sda, gpio_num_t scl,
                                  uint8_t addr, uint32_t clk_hz);

#ifdef __cplusplus
}
#endif

#endif /* ESP_PLATFORM */

#endif /* MPU6050_PORT_ESP32_H */
