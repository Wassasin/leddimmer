#pragma once

#include <esp_err.h>
#include <stdbool.h>

#include "drivers.h"

typedef enum {
    STATE_OFF,
    STATE_UVLO,
    STATE_RAMPING,
    STATE_ENABLED,
    STATE_DISCHARGING,
    STATE_FAULT_COOLDOWN,
    STATE_FAULT_BLOCKED,
} power_logic_state_e;

typedef power_logic_state_e power_logic_states_t[DRIVERS_COUNT];

esp_err_t power_init(void);
esp_err_t power_startup(void);

bool power_high_side_unlocked(uint8_t driver_i);
bool power_low_side_unlocked(uint8_t driver_i);

void power_get_logic_states(power_logic_states_t* logic_states_out);
const char* power_logic_state_to_str(power_logic_state_e state);