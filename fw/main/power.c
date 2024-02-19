#include "power.h"

#include <esp_log.h>
#include "led.h"

#define TAG "power"

// static SemaphoreHandle_t s_sem;

static void power_task(void* arg)
{
    while (1) {
        //
    }
}

esp_err_t power_init(void)
{
    // s_sem = xSemaphoreCreateBinary();
    // xTaskCreate(power_task, "power", 1024 * 4, NULL, 10, NULL);

    return ESP_OK;
}