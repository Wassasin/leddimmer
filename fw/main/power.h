#pragma once

#include <esp_err.h>
#include <stdbool.h>

esp_err_t power_init(void);
esp_err_t power_startup(void);

bool power_high_side_unlocked(uint8_t driver_i);
bool power_low_side_unlocked(uint8_t driver_i);
