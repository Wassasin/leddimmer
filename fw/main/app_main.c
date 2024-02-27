#include <esp_log.h>
#include <nvs_flash.h>

#include "adc.h"
#include "drivers.h"
#include "events.h"
#include "http_server.h"
#include "led.h"
#include "mqtt.h"
#include "performance.h"
#include "periodic.h"
#include "power.h"
#include "wifi.h"

#define TAG "main"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(events_init());
    ESP_ERROR_CHECK(performance_init());

    ESP_ERROR_CHECK(led_init());
    led_set_color((rgb_t) {
        .r = 0x10,
        .g = 0x00,
        .b = 0x00,
    });

    ESP_ERROR_CHECK(power_init());

    ESP_ERROR_CHECK(adc_init());
    ESP_ERROR_CHECK(drivers_init());

    ESP_ERROR_CHECK(wifi_init());
    ESP_ERROR_CHECK(http_server_init());
    ESP_ERROR_CHECK(mqtt_init());

    ESP_ERROR_CHECK(periodic_init());

    ESP_LOGI(TAG, "Device initialized, running event loop");
}
