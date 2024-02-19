#include "events.h"

ESP_EVENT_DEFINE_BASE(EVENTS);

esp_err_t events_init(void)
{
    return esp_event_loop_create_default();
}
