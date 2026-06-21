#ifndef CORE1_TASK_H
#define CORE1_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "solver.h"

typedef struct {
    float   target_range_m;
    float   delta_height_m;
    float   site_altitude_m;
    float   temperature_c;
    float   wind_speed_ms;
    float   wind_dir_deg;
    float   bearing_deg;
    uint8_t projectile;
    uint8_t angle_branch;
    bool    charge_auto;
    uint8_t fixed_charge;
} solve_request_t;

void core1_entry(void);
void core1_launch(void);
bool core1_request_solve(solve_request_t *req);
bool core1_result_ready(void);
solver_result_t core1_get_result(void);

#endif
