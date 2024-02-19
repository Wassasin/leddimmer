#pragma once

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define ERROR_CHECK(a, str, goto_tag, ...)                                        \
    do {                                                                          \
        if (!(a)) {                                                               \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define ERROR_CHECK_SIMPLE(a)                                               \
    do {                                                                    \
        ret = a;                                                            \
        if (ret != ESP_OK) {                                                \
            ESP_LOGE(TAG, "%s(%d): error %i", __FUNCTION__, __LINE__, ret); \
            goto err;                                                       \
        }                                                                   \
    } while (0)
