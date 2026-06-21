#include <stdio.h>
#include "pico/stdlib.h"
#include "pico_keypad4x4.h"
#include "lcd1602_i2c.h"
#include "menu.h"
#include "core1_task.h"

int main(void) {
    stdio_init_all();

    core1_launch();

    uint columns[4] = {18, 19, 20, 21};
    uint rows[4] = {10, 11, 12, 13};

    char matrix[16] = {
        '1', '2', '3', 'A',
        '4', '5', '6', 'B',
        '7', '8', '9', 'C',
        '*', '0', '#', 'D'
    };

    pico_keypad_init(columns, rows, matrix);

    menu_init();

    char key;
    while (true) {
        key = pico_keypad_get_key();
        if (key != '\0') {
            printf("Key pressed: %c\n", key);
            menu_process_key(key);
        }
        busy_wait_us(50000);
    }

    return 0;
}
