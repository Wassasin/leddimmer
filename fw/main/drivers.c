#include "drivers.h"

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "led.h"
#include "util.h"

#define TAG "drivers"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_11_BIT
#define LEDC_FREQUENCY (30000)

#define LEDC_DRIVER1_CHANNEL (LEDC_CHANNEL_0)
#define LEDC_DRIVER2_CHANNEL (LEDC_CHANNEL_1)

#define GPIO_DRIVER1_PWM (8)
#define GPIO_DRIVER2_PWM (9)
#define GPIO_DRIVER1_EN (6)
#define GPIO_DRIVER2_EN (7)

static drivers_pwm11_t s_state;
static SemaphoreHandle_t s_mutex;

static esp_err_t channel_config(ledc_channel_t channel, int gpio_num)
{
    ledc_channel_config_t config = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio_num,
        .duty = 0x00, // Set duty to 100% (inverted)
        .hpoint = 0
    };
    return ledc_channel_config(&config);
}

static bool drivers_persist_channel_unsafe(ledc_channel_t channel, driver_pwm11_t duty)
{
    if (duty == 0x00) {
        ESP_ERROR_CHECK(ledc_stop(LEDC_MODE, channel, 0)); // Inverted
    } else {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, duty)); // Inverted
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
    }
    return duty > 0;
}

static void drivers_persist_unsafe(void)
{
    bool drivers[2] = {0};

    drivers[0] = drivers_persist_channel_unsafe(LEDC_DRIVER1_CHANNEL, s_state[0]);
    drivers[1] = drivers_persist_channel_unsafe(LEDC_DRIVER2_CHANNEL, s_state[1]);

    gpio_set_level(GPIO_DRIVER1_EN, drivers[0]);
    gpio_set_level(GPIO_DRIVER2_EN, drivers[1]);

    // // TODO Temporary emotes until controller is written
    // if (any) {
    //     led_set_color((rgb_t) {
    //         .r = 0x00,
    //         .g = 0x10,
    //         .b = 0x00,
    //     });
    // } else {
    //     led_set_color((rgb_t) {
    //         .r = 0x10,
    //         .g = 0x00,
    //         .b = 0x00,
    //     });
    // }
}

esp_err_t drivers_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_DRIVER1_EN) | (1ULL << GPIO_DRIVER2_EN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ESP_ERROR_CHECK(channel_config(LEDC_DRIVER1_CHANNEL, GPIO_DRIVER1_PWM));
    ESP_ERROR_CHECK(channel_config(LEDC_DRIVER2_CHANNEL, GPIO_DRIVER2_PWM));

    drivers_persist_unsafe();

    return ESP_OK;
}

esp_err_t drivers_command(uint8_t driver_i, driver_pwm11_t duty)
{
    if (driver_i > ARRAY_SIZE(s_state)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (duty > DRIVER_PWM_MAX) {
        duty = DRIVER_PWM_MAX;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state[driver_i] = duty;
    drivers_persist_unsafe();
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

esp_err_t drivers_fetch(drivers_pwm11_t duty_out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(duty_out, s_state, sizeof(s_state));
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}
