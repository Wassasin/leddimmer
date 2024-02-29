#pragma once

#include <esp_err.h>
#include <stdbool.h>

esp_err_t mqtt_init(void);
bool mqtt_is_connected(void);