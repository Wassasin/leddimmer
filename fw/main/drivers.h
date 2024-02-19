#pragma once

#include <esp_err.h>

#define DRIVER_PWM_MAX 2047

typedef uint16_t driver_pwm11_t;
typedef driver_pwm11_t drivers_pwm11_t[2];

#define DRIVERS_COUNT (sizeof(drivers_pwm11_t) / sizeof(driver_pwm11_t))

esp_err_t drivers_init(void);

esp_err_t drivers_command(uint8_t driver_i, const driver_pwm11_t duty);
esp_err_t drivers_fetch(drivers_pwm11_t duty_out);
