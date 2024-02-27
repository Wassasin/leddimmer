#include "led.h"

#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "util/led_strip_encoder.h"

#define TAG "led"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 10
#define LED_NUMBERS 1

static SemaphoreHandle_t s_mutex;
static uint8_t led_strip_pixels[LED_NUMBERS * 3];

static rmt_channel_handle_t m_led_chan = NULL;
static rmt_encoder_handle_t m_led_encoder = NULL;

esp_err_t led_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &m_led_chan));

    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &m_led_encoder));

    ESP_ERROR_CHECK(rmt_enable(m_led_chan));

    return ESP_OK;
}

void led_set_color(rgb_t rgb)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    led_strip_pixels[0] = rgb.g;
    led_strip_pixels[1] = rgb.r;
    led_strip_pixels[2] = rgb.b;

    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    ESP_ERROR_CHECK(rmt_transmit(m_led_chan, m_led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(m_led_chan, portMAX_DELAY));

    vTaskDelay(pdMS_TO_TICKS(10));

    xSemaphoreGive(s_mutex);
}
