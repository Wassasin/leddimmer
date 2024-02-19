#include "performance.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <string.h>

#include "util.h"

#define MAX_NUMBER_OF_TASKS 16

#define ARRAY_SIZE_OFFSET 5 // Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

typedef struct
{
    TaskHandle_t task_handle;
    UBaseType_t task_number;
    uint8_t percentage;
} performance_result_entry_t;

typedef struct
{
    performance_result_entry_t entries[MAX_NUMBER_OF_TASKS];
    size_t entries_number;
} performance_result_t;

static SemaphoreHandle_t s_mutex;
static performance_result_t s_result;

static int task_status_handle_cmp(const void* x, const void* y)
{
    const TaskStatus_t* x2 = x;
    const TaskStatus_t* y2 = y;

    if (x2->xHandle == y2->xHandle) {
        return 0;
    }
    return x2->xHandle < y2->xHandle ? -1 : 1;
}

static void performance_task(void* arg)
{
    TaskStatus_t prev_tasks[MAX_NUMBER_OF_TASKS];
    uint32_t prev_run_time;

    UBaseType_t prev_tasks_number = uxTaskGetSystemState(prev_tasks, MAX_NUMBER_OF_TASKS, &prev_run_time);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        TaskStatus_t current_tasks[MAX_NUMBER_OF_TASKS];
        uint32_t current_run_time;

        UBaseType_t current_tasks_number = uxTaskGetSystemState(current_tasks, MAX_NUMBER_OF_TASKS, &current_run_time);

        uint32_t total_elapsed_time = (current_run_time - prev_run_time);

        qsort(current_tasks, current_tasks_number, sizeof(TaskStatus_t), task_status_handle_cmp);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_result.entries_number = 0;
        for (size_t i = 0; i < current_tasks_number; i++) {
            size_t k = MAX_NUMBER_OF_TASKS;
            for (size_t j = 0; j < prev_tasks_number; j++) {
                if (current_tasks[i].xHandle == prev_tasks[j].xHandle) {
                    k = j;
                    break;
                }
            }

            // Check if matching task found
            if (k < MAX_NUMBER_OF_TASKS) {
                uint32_t task_elapsed_time = current_tasks[i].ulRunTimeCounter - prev_tasks[k].ulRunTimeCounter;
                uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);

                performance_result_entry_t* entry = &s_result.entries[s_result.entries_number++];
                entry->task_handle = current_tasks[i].xHandle;
                entry->task_number = current_tasks[i].xTaskNumber;
                entry->percentage = percentage_time;
            }
        }
        xSemaphoreGive(s_mutex);

        memcpy(prev_tasks, current_tasks, sizeof(prev_tasks));
        prev_tasks_number = current_tasks_number;
        prev_run_time = current_run_time;
    }
}

esp_err_t performance_init(void)
{
    s_result.entries_number = 0;
    s_mutex = xSemaphoreCreateMutex();
    xTaskCreate(performance_task, "performance", 1024 * 4, (void*)0, 9, NULL);

    return ESP_OK;
}

void performance_fetch(performance_fetch_cb_t cb, void* ctx)
{
    char str[128];

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_result.entries_number; i++) {
        const performance_result_entry_t* entry = &s_result.entries[i];

        snprintf(str, sizeof(str), "%s-%i", pcTaskGetName(entry->task_handle), entry->task_number);

        cb((performance_entry_t) {
               .task_name = str,
               .percentage = entry->percentage,
           },
            ctx);
    }
    xSemaphoreGive(s_mutex);
}