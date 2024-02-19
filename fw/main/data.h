#pragma once

#include <cJSON.h>
#include <esp_err.h>

const char* data_get_id(void);

esp_err_t data_status_to_json(cJSON* root);
esp_err_t data_power_to_json(cJSON* root);
esp_err_t data_duty_to_json(cJSON* root);
esp_err_t data_sensors_to_json(cJSON* root);
esp_err_t data_performance_to_json(cJSON* root);

esp_err_t data_process_duty_json_str(const char* str, size_t str_len);
