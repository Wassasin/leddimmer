#include "periodic.h"

#include <esp_timer.h>

#include "events.h"

static void periodic_timer_callback(void* arg)
{
    ESP_ERROR_CHECK(esp_event_post(EVENTS, EVENT_STATUS_PING,
        NULL, 0, portMAX_DELAY));
}

esp_err_t periodic_init(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "report_timer"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 100000));

    return ESP_OK;
}