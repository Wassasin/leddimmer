#include "emotes.h"

#include "events.h"
#include "led.h"
#include "mqtt.h"
#include "power.h"

#include <esp_timer.h>
#include <esp_log.h>

#define TAG "emotes"

#define EMOTE_UPDATE_RATE_US 20000 // 50Hz / 20ms
#define EMOTE_BLINK_RATE_US (250 * 1000)

typedef enum {
    PREMODE_OFF = 0,
    PREMODE_ENABLED,
    PREMODE_ERROR,
} emotes_premode_e;

typedef enum {
    EMOTE_BOOTING, // Red constant
    EMOTE_UNCONNECTED_IDLE, // Blue blinking
    EMOTE_UNCONNECTED_ENABLED, // Green blinking
    EMOTE_CONNECTED_IDLE, // Blue constant
    EMOTE_CONNECTED_ENABLED, // Green constant
    EMOTE_ERROR, // Red blinking
} emotes_mode_e;

typedef struct {
    emotes_mode_e mode;
    uint64_t timestamp_start_us;
} emotes_state_t;

static SemaphoreHandle_t s_mutex;
static emotes_state_t s_state;

const char* emotes_mode_to_str(emotes_mode_e mode)
{
    switch (mode) {
    case EMOTE_BOOTING:
        return "booting";
    case EMOTE_UNCONNECTED_IDLE:
        return "unconnected_idle";
    case EMOTE_UNCONNECTED_ENABLED:
        return "unconnected_enabled";
    case EMOTE_CONNECTED_IDLE:
        return "connected_idle";
    case EMOTE_CONNECTED_ENABLED:
        return "connected_enabled";
    case EMOTE_ERROR:
        return "error";
    }

    return "unknown";
}

static emotes_premode_e emotes_determine_premode_from_ps(power_logic_state_e state)
{
    switch (state) {
    case STATE_OFF:
    case STATE_RAMPING:
    case STATE_DISCHARGING:
        return PREMODE_OFF;
    case STATE_ENABLED:
        return PREMODE_ENABLED;
    case STATE_UVLO:
    case STATE_FAULT_COOLDOWN:
    case STATE_FAULT_BLOCKED:
        return PREMODE_ERROR;
    }

    // Unreachable
    return PREMODE_ERROR;
}

static emotes_premode_e emotes_premode_combine(emotes_premode_e x, emotes_premode_e y)
{
    return x < y ? y : x;
}

static emotes_mode_e emotes_determine_mode(emotes_premode_e premode, bool connected)
{
    if (!connected) {
        switch (premode) {
        case PREMODE_OFF:
            return EMOTE_UNCONNECTED_IDLE;
        case PREMODE_ENABLED:
            return EMOTE_UNCONNECTED_ENABLED;
        case PREMODE_ERROR:
            return EMOTE_ERROR;
        }
    } else {
        switch (premode) {
        case PREMODE_OFF:
            return EMOTE_CONNECTED_IDLE;
        case PREMODE_ENABLED:
            return EMOTE_CONNECTED_ENABLED;
        case PREMODE_ERROR:
            return EMOTE_ERROR;
        }
    }

    // Unreachable
    return EMOTE_ERROR;
}

static bool emotes_is_blinking(emotes_mode_e mode)
{
    switch (mode) {
    case EMOTE_UNCONNECTED_IDLE:
    case EMOTE_UNCONNECTED_ENABLED:
    case EMOTE_ERROR:
        return true;
    default:
        return false;
    }
}

static rgb_t emotes_determine_base_color(emotes_mode_e mode) {
    switch (mode) {
    case EMOTE_BOOTING:
    case EMOTE_ERROR:
        return (rgb_t) { .r = 0x10, .g = 0x00, .b = 0x00 };
    case EMOTE_UNCONNECTED_IDLE:
    case EMOTE_CONNECTED_IDLE:
        return (rgb_t) { .r = 0x00, .g = 0x00, .b = 0x10 };
    case EMOTE_UNCONNECTED_ENABLED:
    case EMOTE_CONNECTED_ENABLED:
        return (rgb_t) { .r = 0x00, .g = 0x10, .b = 0x00 };
    default:
        return (rgb_t) { .r = 0xff, .g = 0xff, .b = 0xff };
    }
}

static void emotes_tick(void)
{
    // Actually send update to LED.
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool should_be_off = false;
    if (emotes_is_blinking(s_state.mode)) {
        // Determine if we are in the 'off' phase of the blinking.
        uint64_t now_us = esp_timer_get_time();
        uint64_t since_start_us = now_us - s_state.timestamp_start_us;
        uint64_t phase_count = since_start_us / EMOTE_BLINK_RATE_US;

        if (phase_count % 2 == 1) {
            should_be_off = true;
        }
    }

    if (should_be_off) {
        led_set_color((rgb_t) { .r = 0x00, .g = 0x00, .b = 0x00 });
    } else {
        led_set_color(emotes_determine_base_color(s_state.mode));
    }
    xSemaphoreGive(s_mutex);
}

static void emotes_timer_callback(void* _arg)
{
    emotes_tick();
}

static void emotes_update(void)
{
    power_logic_states_t power_states;
    power_get_logic_states(&power_states);

    emotes_premode_e premode = PREMODE_OFF;
    for (size_t driver_i = 0; driver_i < DRIVERS_COUNT; driver_i++) {
        premode = emotes_premode_combine(premode, emotes_determine_premode_from_ps(power_states[driver_i]));
    }

    emotes_mode_e mode = emotes_determine_mode(premode, mqtt_is_connected());

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (mode != s_state.mode) {
        ESP_LOGI(TAG, "%s => %s", emotes_mode_to_str(s_state.mode), emotes_mode_to_str(mode));
        s_state.mode = mode;
        s_state.timestamp_start_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
}

static void emotes_status_handler(void* _event_handler_arg,
    esp_event_base_t _event_base,
    int32_t _event_id,
    void* _event_data)
{
    emotes_update();
}

esp_err_t emotes_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_state = (emotes_state_t) {
        .mode = EMOTE_BOOTING,
        .timestamp_start_us = esp_timer_get_time(),
    };

    esp_event_handler_instance_register(EVENTS, EVENT_POWER_STATE,
        emotes_status_handler, NULL, NULL);
    esp_event_handler_instance_register(EVENTS, EVENT_MQTT_ONLINE,
        emotes_status_handler, NULL, NULL);
    esp_event_handler_instance_register(EVENTS, EVENT_MQTT_OFFLINE,
        emotes_status_handler, NULL, NULL);

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &emotes_timer_callback,
        .name = "emotes_timer"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, EMOTE_UPDATE_RATE_US));

    return ESP_OK;
}