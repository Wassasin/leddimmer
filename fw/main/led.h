#pragma once

#include <esp_err.h>

typedef union {
    struct {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };
    uint8_t buf;
} rgb_t;

esp_err_t led_init(void);

void led_set_color(rgb_t rgb);
