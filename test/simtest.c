#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "ballistics/atmosphere.h"
#include "ballistics/drag.h"
#include "ballistics/gun_data.h"
#include "ballistics/integrator.h"
#include "ballistics/solver.h"

typedef struct {
    int    id;
    const char *name;
    float  range;
    float  dh;
    float  alt;
    float  temp;
    float  wind_spd;
    float  wind_dir;
    float  bearing;
    uint8_t ammo;
    uint8_t branch;
    uint8_t expect_chg;
    float  expect_elo;
    float  expect_ehi;
} test_t;

static const test_t tests[] = {
    {1, "flat",      5000,    0,   0, 15, 0, 0,  90, 0, 0, 2, 22, 27},
    {2, "uphill AP", 4000,  500, 200,  5, 0, 0,  45, 1, 0, 2, 26, 32},
    {3, "downhill",  3500, -500, 500, 20, 0, 0, 180, 0, 0, 1,  8, 14},
    {4, "long",      8000,    0,   0, 15, 0, 0,   0, 0, 0, 5, 29, 34},
    {5, "high ang",  1500,    0, 100, 25, 0, 0, 270, 0, 1, 1, 63, 65},
    {6, "out range",12000,    0,   0, 15, 0, 0,   0, 0, 0, 0,  0,  0},
    {7, "cold",      3000,    0,   0,-20, 0, 0,   0, 0, 0, 1, 16, 24},
};

static void run_test(const test_t *t) {
    solver_result_t r = solve_ballistics(
        t->range, t->dh, t->alt, t->temp,
        t->wind_spd, t->wind_dir, t->bearing,
        t->ammo, t->branch);

    printf("Test %d %-12s", t->id, t->name);
    if (r.reachable) {
        int ok = (r.charge_number == t->expect_chg &&
                  r.elevation_deg >= t->expect_elo &&
                  r.elevation_deg <= t->expect_ehi);
        printf("CHG=%d  ELEV=%.1f  %s",
               r.charge_number, r.elevation_deg,
               ok ? "✓" : "✗ MISMATCH");
        if (!ok) {
            printf(" (want CHG=%d, %.0f~%.0f°)",
                   t->expect_chg, t->expect_elo, t->expect_ehi);
        }
    } else {
        if (t->expect_chg == 0)
            printf("OUT OF RANGE  ✓");
        else
            printf("OUT OF RANGE  ✗ (want CHG=%d)", t->expect_chg);
    }
    printf("\n");
}

static void sweep_max_ranges(void) {
    printf("\n--- Max Range Sweep (HE, flat, sea level, 15C) ---\n");
    for (int c = 1; c <= 5; c++) {
        float best_r = 0, best_a = 0;
        for (float a = 1.0f; a <= 65.0f; a += 1.0f) {
            bool reached;
            float r = rk4_trajectory(
                a * 3.1415926f / 180.0f,
                gun_muzzle_velocity(c, 0),
                gun_mass(0), PROJECTILE_AREA,
                atm_density(0, 15), atm_sound_speed(15),
                0.0f, 0.0f, &reached);
            if (reached && r > best_r) { best_r = r; best_a = a; }
        }
        printf("  CHG %d (%.0f m/s): max_range=%.0fm at %.0f deg\n",
               c, gun_muzzle_velocity(c, 0), best_r, best_a);
    }
}

static void test_fixed_charges(void) {
    printf("\n--- Fixed Charge Tests (5000m, HE, flat) ---\n");
    for (int c = 1; c <= 5; c++) {
        solver_result_t r = solve_ballistics_fixed_charge(
            5000, 0, 0, 15, 0, 0, 90, 0, 0, c);
        if (r.reachable)
            printf("  CHG %d FIXED: ELEV=%.1f\n", c, r.elevation_deg);
        else
            printf("  CHG %d FIXED: OUT OF RANGE\n", c);
    }
}

static void test_wind(void) {
    printf("\n--- Wind Tests (5000m, HE, flat, 10m/s) ---\n");
    solver_result_t r;

    r = solve_ballistics(5000, 0, 0, 15, 10, 90, 90, 0, 0);
    printf("  Headwind (10m/s): %s CHG=%d ELEV=%.1f\n",
           r.reachable ? "OK" : "FAIL", r.charge_number, r.elevation_deg);

    r = solve_ballistics(5000, 0, 0, 15, 10, 270, 90, 0, 0);
    printf("  Tailwind (10m/s): %s CHG=%d ELEV=%.1f\n",
           r.reachable ? "OK" : "FAIL", r.charge_number, r.elevation_deg);

    r = solve_ballistics(5000, 0, 0, 15, 0, 0, 90, 0, 0);
    printf("  No wind:          %s CHG=%d ELEV=%.1f\n",
           r.reachable ? "OK" : "FAIL", r.charge_number, r.elevation_deg);
}

int main(void) {
    printf("=== Ballistics Solver Verification ===\n\n");

    printf("Atmosphere: rho(0,15)=%.4f  sound(15)=%.1f\n",
           atm_density(0, 15), atm_sound_speed(15));
    printf("Atmosphere: rho(500,20)=%.4f  sound(20)=%.1f\n",
           atm_density(500, 20), atm_sound_speed(20));
    printf("Atmosphere: rho(0,-20)=%.4f  sound(-20)=%.1f\n",
           atm_density(0, -20), atm_sound_speed(-20));

    printf("\n--- Standard Tests ---\n");
    for (int i = 0; i < 7; i++) run_test(&tests[i]);

    printf("\n--- AP Uphill Max Ranges (4000m target, dH=500, 200m, 5C) ---\n");
    float rho2 = atm_density(200, 5);
    float snd2 = atm_sound_speed(5);
    for (int c = 1; c <= 5; c++) {
        float v0 = gun_muzzle_velocity(c, 1);
        float best_r = -1, best_a = 0;
        for (float a = 5; a <= 65; a += 5) {
            bool reached;
            float r = rk4_trajectory(a * 3.1415926f / 180.0f,
                                     v0, gun_mass(1), PROJECTILE_AREA,
                                     rho2, snd2, 0, 500, &reached);
            if (reached && r > best_r) { best_r = r; best_a = a; }
        }
        printf("  AP CHG%d (%4.0f m/s): max=%.0fm at %.0f deg  %s\n",
               c, v0, best_r, best_a, best_r >= 4000 ? "REACH" : "FAIL");
    }

    sweep_max_ranges();
    test_fixed_charges();
    test_wind();

    printf("\n=== Done ===\n");
    return 0;
}
