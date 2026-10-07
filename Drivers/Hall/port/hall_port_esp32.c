/**
 * @file    hall_port_esp32.c
 * @brief   ESP-IDF GPIO interrupt glue for the Hall wheel sensor driver.
 */
#ifdef ESP_PLATFORM

#include "hall_port_esp32.h"

#include "esp_attr.h"
#include "esp_timer.h"

/**
 * @brief GPIO ISR: timestamp the edge and hand it to the portable driver.
 */
static void IRAM_ATTR hall_gpio_isr(void *arg)
{
    hall_on_pulse((hall_t *)arg, (uint64_t)esp_timer_get_time());
}

/**
 * @brief Configure the GPIO and register the interrupt handler.
 */
esp_err_t hall_port_esp32_init(hall_t *h, gpio_num_t pin, bool active_low)
{
    esp_err_t err;

    if (h == NULL || !GPIO_IS_VALID_GPIO(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << (uint32_t)pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = active_low ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;   /* INVALID_STATE: service already installed, fine */
    }

    return gpio_isr_handler_add(pin, hall_gpio_isr, h);
}

#endif /* ESP_PLATFORM */
