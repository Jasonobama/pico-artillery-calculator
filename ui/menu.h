#ifndef MENU_H
#define MENU_H

typedef enum {
    FIELD_SCENARIO,
    FIELD_CHARGE_MODE,
    FIELD_AMMO,
    FIELD_DIST,
    FIELD_BRG,
    FIELD_ALT,
    FIELD_DH,
    FIELD_TEMP,
    FIELD_WIND_SPD,
    FIELD_WIND_DIR,
    FIELD_CONFIRM,
    FIELD_RESULT,
    NUM_FIELDS
} menu_field_t;

void menu_init(void);
void menu_process_key(char key);
void menu_update_display(void);

#endif
