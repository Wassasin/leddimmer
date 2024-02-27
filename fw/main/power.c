#include "power.h"

#include "util.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define TAG "power"

#define GPIO_DRIVER1_STAT (20)
#define GPIO_DRIVER2_STAT (21)

static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_sem;

typedef struct
{
    gpio_num_t stat_gpio_num;
} power_description_t;

typedef enum {
    STATE_OFF,
    STATE_RAMPING,
    STATE_ENABLED,
    STATE_DISCHARGING,
    STATE_FAULT_COOLDOWN,
    STATE_FAULT_BLOCKED,
} power_logic_state_e;

typedef struct
{
    bool level;
    power_logic_state_e logic_state;
} power_state_t;

static power_description_t s_descriptions[2]
    = {
          { stat_gpio_num : GPIO_DRIVER1_STAT },
          { stat_gpio_num : GPIO_DRIVER2_STAT },
      };

#define STAT_NUM (ARRAY_SIZE(s_descriptions))

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static const char* power_logic_state_to_str(power_logic_state_e state)
{
    switch (state) {
    case STATE_OFF:
        return "off";
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

static void
power_machine_run(size_t system_i, power_state_t* state)
{
    power_description_t* description = &s_descriptions[system_i];

    bool level = !gpio_get_level(description->stat_gpio_num);

    if (level != state->level) {
        ESP_LOGD(TAG, "%u => %s", system_i, level ? "on" : "off");
    }
    state->level = level;

    power_logic_state_e prev_logic_state = state->logic_state;
    switch (state->logic_state) {
    case STATE_OFF:
        break;
    case STATE_RAMPING:
        break;
    case STATE_ENABLED:
        break;
    case STATE_DISCHARGING:
        break;
    case STATE_FAULT_COOLDOWN:
        break;
    case STATE_FAULT_BLOCKED:
        break;
    }

    if (prev_logic_state != state->logic_state) {
        ESP_LOGI(
            TAG,
            "State %s => %s",
            power_logic_state_to_str(prev_logic_state),
            power_logic_state_to_str(state->logic_state));
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

    power_state_t state[STAT_NUM];
    for (size_t i = 0; i < STAT_NUM; ++i) {
        state[i] = (power_state_t) {
            level : false,
            logic_state : STATE_OFF,
        };
    }

    while (1) {
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)); // Do not test, re-compute state regardless

        for (size_t i = 0; i < STAT_NUM; ++i) {
            power_machine_run(i, &state[i]);
        }
    }
}

esp_err_t power_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_sem = xSemaphoreCreateBinary();

    xTaskCreate(power_task, "power", 1024 * 4, NULL, 11, NULL);

    return ESP_OK;
}