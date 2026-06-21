#ifndef GUN_DATA_H
#define GUN_DATA_H

#include <stdint.h>

#define CHARGE_LEVELS 5

#define PROJECTILE_HE 0
#define PROJECTILE_AP 1

#define MASS_HE 819.0f
#define MASS_AP 1000.0f

#define CALIBER_RADIUS 0.21f
#define PROJECTILE_AREA 0.138544f

#define MAX_ELEVATION_DEG 65.0f

extern const float muzzle_velocity_he[CHARGE_LEVELS];
extern const float muzzle_velocity_ap[CHARGE_LEVELS];

float gun_mass(uint8_t projectile);
float gun_muzzle_velocity(uint8_t charge, uint8_t projectile);

#endif
