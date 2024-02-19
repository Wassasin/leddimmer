#pragma once

#include <esp_err.h>

typedef struct
{
    const char* task_name;
    uint8_t percentage;
} performance_entry_t;

typedef void (*performance_fetch_cb_t)(performance_entry_t, void*);

esp_err_t performance_init(void);
void performance_fetch(performance_fetch_cb_t cb, void* ctx);
