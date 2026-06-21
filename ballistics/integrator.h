#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <stdbool.h>

void rk4_step(float state[4], float dt, float mass, float area,
              float rho, float sound_speed, float wx);

float rk4_trajectory(float elevation_rad, float v0,
                     float mass, float area,
                     float rho, float sound_speed,
                     float wx, float delta_h,
                     bool *reached);

#endif
