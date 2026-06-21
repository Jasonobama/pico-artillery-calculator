#ifndef SOLVER_H
#define SOLVER_H

#include <stdbool.h>
#include <stdint.h>

#define BRANCH_LOW  0
#define BRANCH_HIGH 1

typedef struct {
    bool    reachable;
    float   elevation_deg;
    uint8_t charge_number;
} solver_result_t;

solver_result_t solve_ballistics(
    float range_m, float delta_h_m,
    float altitude_m, float temperature_c,
    float wind_speed_ms, float wind_dir_deg,
    float bearing_deg, uint8_t projectile,
    uint8_t branch);

solver_result_t solve_ballistics_fixed_charge(
    float range_m, float delta_h_m,
    float altitude_m, float temperature_c,
    float wind_speed_ms, float wind_dir_deg,
    float bearing_deg, uint8_t projectile,
    uint8_t branch, uint8_t charge);

#endif
