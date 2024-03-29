#include "power.h"

#include "adc.h"
#include "drivers.h"
#include "util.h"
#include "events.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define TAG "power"

#define GPIO_DRIVER1_STAT (20)
#define GPIO_DRIVER2_STAT (21)

#define FAULT_TIMEOUT_US 1000000
#define RAMPING_TIMEOUT_US 100000
#define FAULT_LIMIT 10

#define POWER_UVLO_MV 23000;

static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_sem;

typedef struct
{
    gpio_num_t stat_gpio_num;
} power_description_t;

typedef struct
{
    uint64_t timeout_us;
    uint8_t fault_count;
    power_logic_state_e logic_state;
    bool level;
} power_state_t;

static const power_description_t s_descriptions[DRIVERS_COUNT]
    = {
          { .stat_gpio_num = GPIO_DRIVER1_STAT },
          { .stat_gpio_num = GPIO_DRIVER2_STAT },
      };

static power_state_t s_state[DRIVERS_COUNT];

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

const char* power_logic_state_to_str(power_logic_state_e state)
{
    switch (state) {
    case STATE_OFF:
        return "off";
    case STATE_UVLO:
        return "uvlo";
    case STATE_RAMPING:
        return "ramping";
    case STATE_ENABLED:
        return "enabled";
    case STATE_DISCHARGING:
        return "discharging";
    case STATE_FAULT_COOLDOWN:
        return "fault_cooldown";
    case STATE_FAULT_BLOCKED:
        return "fault_blocked";
    }

    return "unknown";
}

static void power_machine_enter_fault(power_state_t* state)
{
    if (state->fault_count < FAULT_LIMIT) {
        state->timeout_us = esp_timer_get_time() + FAULT_TIMEOUT_US;
        state->logic_state = STATE_FAULT_COOLDOWN;
        state->fault_count++;
    } else {
        state->timeout_us = UINT64_MAX;
        state->logic_state = STATE_FAULT_BLOCKED;
    }
}

static bool power_machine_uvlo(void)
{
    adc_samples_t samples;
    adc_fetch(&samples);

    return samples.vbus_mv.rms < POWER_UVLO_MV;
}

static void power_machine_enter_ramping(power_state_t* state)
{
    if (power_machine_uvlo()) {
        state->logic_state = STATE_UVLO;
    } else {
        state->timeout_us
            = esp_timer_get_time() + RAMPING_TIMEOUT_US;
        state->logic_state = STATE_RAMPING;
    }
}

static bool power_timeout_passed(power_state_t* state)
{
    if (state->timeout_us == UINT64_MAX) {
        return false;
    }

    return esp_timer_get_time() > state->timeout_us;
}

static void power_machine_run(size_t system_i, power_state_t* state, const driver_pwm11_t driver)
{
    const power_description_t* description = &s_descriptions[system_i];

    bool stat_level = gpio_get_level(description->stat_gpio_num);

    if (stat_level != state->level) {
        ESP_LOGD(TAG, "%u => %s", system_i, stat_level ? "on" : "off");
    }
    state->level = stat_level;

    bool requested = driver != 0x0000;

    power_logic_state_e prev_logic_state = state->logic_state;
    switch (state->logic_state) {
    case STATE_OFF:
        state->fault_count = 0;

        if (requested) {
            power_machine_enter_ramping(state);
            break;
        }
        break;
    case STATE_UVLO:
        if (!requested) {
            state->logic_state = STATE_OFF;
            break;
        }

        power_machine_enter_ramping(state);
        break;
    case STATE_RAMPING:
        if (!requested) {
            state->logic_state = STATE_DISCHARGING;
            break;
        }

        if (power_timeout_passed(state)) {
            if (!stat_level) {
                // In this phase we expect load less than I_OL, hence STATUS=LOW.
                state->logic_state = STATE_ENABLED;
                break;
            } else {
                // If we see STATUS=HIGH we have high load, even though low side driver is off.
                power_machine_enter_fault(state);
                break;
            }
        }

        break;
    case STATE_ENABLED:
        // Note: we do not check stat/fault here, because I_OL leads to it being unusable in low current use.
        if (!requested) {
            state->logic_state = STATE_DISCHARGING;
            break;
        }

        if (power_machine_uvlo()) {
            state->logic_state = STATE_UVLO;
            break;
        }
        break;
    case STATE_DISCHARGING:
        if (requested) {
            power_machine_enter_ramping(state);
            break;
        }

        if (!stat_level) {
            state->logic_state = STATE_OFF;
            break;
        }
        break;
    case STATE_FAULT_COOLDOWN:
        if (!requested) {
            state->logic_state = STATE_OFF;
            break;
        }

        if (power_timeout_passed(state)) {
            power_machine_enter_ramping(state);
            break;
        }
        break;
    case STATE_FAULT_BLOCKED:
        if (!requested) {
            state->logic_state = STATE_OFF;
            break;
        }
        break;
    }

    if (prev_logic_state != state->logic_state) {
        ESP_LOGI(
            TAG,
            "State[%u] %s => %s",
            system_i,
            power_logic_state_to_str(prev_logic_state),
            power_logic_state_to_str(state->logic_state));

        ESP_ERROR_CHECK(esp_event_post(EVENTS, EVENT_POWER_STATE,
            NULL, 0, portMAX_DELAY));
    }
}

static void power_task(void* arg)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_DRIVER1_STAT) | (1ULL << GPIO_DRIVER2_STAT),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_DRIVER1_STAT, gpio_isr_handler, (void*)(size_t)0);
    gpio_isr_handler_add(GPIO_DRIVER2_STAT, gpio_isr_handler, (void*)(size_t)1);

    while (1) {
        xSemaphoreTake(s_sem, pdMS_TO_TICKS(50)); // Do not test, re-compute state regardless

        drivers_pwm11_t drivers;
        drivers_fetch(&drivers);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        for (size_t i = 0; i < DRIVERS_COUNT; ++i) {
            power_machine_run(i, &s_state[i], drivers[i]);
        }
        xSemaphoreGive(s_mutex);

        drivers_update();
    }
}

esp_err_t power_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_sem = xSemaphoreCreateBinary();

    for (size_t i = 0; i < DRIVERS_COUNT; ++i) {
        s_state[i] = (power_state_t) {
            level : false,
            logic_state : STATE_OFF,
        };
    }

    return ESP_OK;
}

esp_err_t power_startup(void)
{
    xTaskCreate(power_task, "power", 1024 * 4, NULL, 11, NULL);

    return ESP_OK;
}

bool power_high_side_unlocked(uint8_t driver_i)
{
    if (driver_i >= DRIVERS_COUNT) {
        return false;
    }

    bool unlocked = false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    switch (s_state[driver_i].logic_state) {
    case STATE_RAMPING:
    case STATE_ENABLED:
        unlocked = true;
        break;
    default:
    }
    xSemaphoreGive(s_mutex);

    return unlocked;
}

bool power_low_side_unlocked(uint8_t driver_i)
{
    if (driver_i >= DRIVERS_COUNT) {
        return false;
    }

    bool unlocked = false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    switch (s_state[driver_i].logic_state) {
    case STATE_ENABLED:
        unlocked = true;
        break;
    default:
    }
    xSemaphoreGive(s_mutex);

    return unlocked;
}

void power_get_logic_states(power_logic_states_t* logic_states_out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t driver_i = 0; driver_i < DRIVERS_COUNT; ++driver_i) {
        (*logic_states_out)[driver_i] = s_state[driver_i].logic_state;
    }
    xSemaphoreGive(s_mutex);
}