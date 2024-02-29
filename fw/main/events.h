#pragma once

#include <esp_event.h>

ESP_EVENT_DECLARE_BASE(EVENTS);

enum {
    EVENT_POWER_STATE, // Power state has been updated
    EVENT_ONLINE, // Network was connected (i.e. we got an IP-address)
    EVENT_OFFLINE, // Network was disconnected (interface went down)
    EVENT_MQTT_ONLINE, // MQTT client was connected
    EVENT_MQTT_OFFLINE, // MQTT client was disconnected
    EVENT_STATUS_PING, // Emit our status via MQTT
    EVENT_ADC_SAMPLED, // ADC sample completed
};

esp_err_t events_init(void);
