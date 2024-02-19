#include "led.h"

#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define TAG "led"

#define LEDC_TIMER LEDC_TIMER_1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY (100000)

#define LEDC_R_CHANNEL (LEDC_CHANNEL_5)
#define LEDC_G_CHANNEL (LEDC_CHANNEL_6)
#define LEDC_B_CHANNEL (LEDC_CHANNEL_7)

#define GPIO_R_PWM (48)
#define GPIO_G_PWM (47)
#define GPIO_B_PWM (35)

static SemaphoreHandle_t s_mutex;

// static esp_err_t channel_config(ledc_channel_t channel, int gpio_num)
// {
//     ledc_channel_config_t config = {
//         .speed_mode = LEDC_MODE,
//         .channel = channel,
//         .timer_sel = LEDC_TIMER,
//         .intr_type = LEDC_INTR_DISABLE,
//         .gpio_num = gpio_num,
//         .duty = 0xff, // Set duty to 100% (inverted)
//         .hpoint = 0
//     };
//     return ledc_channel_config(&config);
// }

// static bool channel_persist_channel_unsafe(ledc_channel_t channel, uint8_t duty)
// {
//     if (duty == 0x00) {
//         ESP_ERROR_CHECK(ledc_stop(LEDC_MODE, channel, 1)); // Inverted
//     } else {
//         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, 0xff - duty)); // Inverted
//         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
//     }

//     return duty > 0;
// }

esp_err_t led_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    // ledc_timer_config_t ledc_timer = {
    //     .speed_mode = LEDC_MODE,
    //     .timer_num = LEDC_TIMER,
    //     .duty_resolution = LEDC_DUTY_RES,
    //     .freq_hz = LEDC_FREQUENCY,
    //     .clk_cfg = LEDC_AUTO_CLK
    // };
    // ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // ESP_ERROR_CHECK(channel_config(LEDC_R_CHANNEL, GPIO_R_PWM));
    // ESP_ERROR_CHECK(channel_config(LEDC_G_CHANNEL, GPIO_G_PWM));
    // ESP_ERROR_CHECK(channel_config(LEDC_B_CHANNEL, GPIO_B_PWM));

    return ESP_OK;
}

void led_set_color(rgb_t rgb)
{
    // xSemaphoreTake(s_mutex, portMAX_DELAY);
    // channel_persist_channel_unsafe(LEDC_R_CHANNEL, rgb.r);
    // channel_persist_channel_unsafe(LEDC_G_CHANNEL, rgb.g);
    // channel_persist_channel_unsafe(LEDC_B_CHANNEL, rgb.b);
    // xSemaphoreGive(s_mutex);
}
