/**
 * @file pico_keypad4x4.c
 * @brief 4x4 matrix keypad scanner with debounce for Raspberry Pi Pico.
 *
 * @version 0.3
 * @author Oswaldo Hernandez, Owen Jauregui
 * @contact oswahdez00@gmail.com
 *
 * This module scans a 4-row by 4-column matrix keypad by driving row lines
 * high one at a time and reading the column inputs. A press is detected when
 * a row-drive-high propagates through a closed switch to its column pin.
 *
 * Debounce strategy:
 *   - A pre-scan check: if no column is high, no key is pressed; reset state.
 *   - Row drive settle time: 10 ms busy-wait per row during sequential scan.
 *   - One-shot per press: the same key must change to be reported again
 *     (stored in static last_key). This prevents key-repeat on continuous hold.
 *
 * Pin direction convention:
 *   - Rows: outputs (driven high sequentially during scan).
 *   - Columns: inputs (read to detect which column received the high).
 *
 * IRQ support: column pins can be configured for GPIO interrupt on edge
 *   detection (GPIO_IRQ_LEVEL_HIGH = 0x8). When enabled, any column going
 *   high triggers the user callback, eliminating the need for polling.
 */

#include "pico_keypad4x4.h"

#define GPIO_INPUT false   /**< RP2040 GPIO direction: input */
#define GPIO_OUTPUT true   /**< RP2040 GPIO direction: output */

uint _columns[4];          /**< GPIO pin numbers for the 4 columns */
uint _rows[4];             /**< GPIO pin numbers for the 4 rows */
char _matrix_values[16];   /**< Key character assignments in row-major order */

uint all_columns_mask = 0x0; /**< Bitmask of all column pins for fast GPIO read masking */
uint column_mask[4];          /**< Individual bitmask for each column pin */

/**
 * @brief Initialize the 4x4 keypad hardware and store configuration.
 *
 * Copies the pin assignments and key map into static storage. Configures
 * column pins as inputs and row pins as outputs (initially driven high so
 * no column can be pulled high through an open switch). Computes GPIO
 * bitmasks for efficient scanning via gpio_get_all().
 *
 * @param columns       Array of 4 GPIO pins connected to keypad columns.
 * @param rows          Array of 4 GPIO pins connected to keypad rows.
 * @param matrix_values Array of 16 characters assigning labels to each
 *                      key position in row-major order (row0 col0..col3,
 *                      row1 col0..col3, ...).
 */
void pico_keypad_init(uint columns[4], uint rows[4], char matrix_values[16]) {

    for (int i = 0; i < 16; i++) {
        _matrix_values[i] = matrix_values[i]; /**< Copy key legend map */
    }

    for (int i = 0; i < 4; i++) {

        _columns[i] = columns[i];
        _rows[i] = rows[i];

        gpio_init(_columns[i]);
        gpio_init(_rows[i]);

        gpio_set_dir(_columns[i], GPIO_INPUT);  /**< Columns read switch state */
        gpio_set_dir(_rows[i], GPIO_OUTPUT);    /**< Rows are driven to scan */

        gpio_put(_rows[i], 1);  /**< Default rows high (active-low logic) */

        all_columns_mask = all_columns_mask + (1 << _columns[i]); /**< Accumulate bitmask of all column GPIOs */
        column_mask[i] = 1 << _columns[i];                        /**< Individual bitmask for col i */
    }
}

/**
 * @brief Scan the keypad matrix and return the currently pressed key.
 *
 * Algorithm:
 *   1. Quick pre-check: read all column pins. If none are high, no key is
 *      pressed. Reset the debounce state (last_key = 0) and return 0.
 *   2. Drive all rows low to discharge any floating state.
 *   3. Sequential row scan: drive each row high one at a time, wait 10 ms
 *      for signal settling, then read columns. Store the first row that
 *      produces a non-zero column reading and break.
 *   4. Restore all rows to high (idle state).
 *   5. Match the column bitmask to determine which column key was pressed.
 *   6. One-shot filter: if the detected key matches last_key, suppress
 *      repeat and return 0. Otherwise update last_key and return the key.
 *
 * @return The character associated with the pressed key, or 0 (NUL) if no
 *         key is pressed or the same key is still held from last call.
 */
char pico_keypad_get_key(void) {
    static char last_key = 0; /**< Stores previous key for one-shot debounce */
    int row;
    uint32_t cols;

    cols = gpio_get_all();
    cols = cols & all_columns_mask; /**< Isolate column pins from all GPIO state */
    
    if (cols == 0x0) {
        last_key = 0;  /**< No key pressed: reset debounce memory */
        return 0;
    }

    for (int j = 0; j < 4; j++) {
        gpio_put(_rows[j], 0); /**< Discharge all rows before scan */
    }

    for (row = 0; row < 4; row++) {

        gpio_put(_rows[row], 1); /**< Drive current row high */

        busy_wait_us(10000);     /**< 10 ms settle time for RC rise through switch */

        cols = gpio_get_all();
        gpio_put(_rows[row], 0); /**< Drive row low again before reading next */
        cols = cols & all_columns_mask;
        if (cols != 0x0) {
            break; /**< Found the row with a pressed key */
        }   
    }

    for (int i = 0; i < 4; i++) {
        gpio_put(_rows[i], 1); /**< Restore all rows to idle high */
    }

    char current_key = 0;

    /** Decode column: match the single bit set in cols against column masks */
    if (cols == column_mask[0]) {
        current_key = (char)_matrix_values[row * 4 + 0];
    }
    else if (cols == column_mask[1]) {
        current_key = (char)_matrix_values[row * 4 + 1];
    }
    else if (cols == column_mask[2]) {
        current_key = (char)_matrix_values[row * 4 + 2];
    }
    else if (cols == column_mask[3]) {
        current_key = (char)_matrix_values[row * 4 + 3];
    }
    else {
        last_key = 0;  /**< Noise or multiple columns: treat as no press */
        return 0;
    }

    if (current_key == last_key) {
        return 0; /**< One-shot: suppress repeat of same key */
    }
    last_key = current_key;
    return current_key;
}

/**
 * @brief Enable or disable GPIO interrupt on all column pins.
 *
 * When enabled, any column pin transitioning high (GPIO_IRQ_LEVEL_HIGH = 0x8)
 * triggers the provided callback. This allows interrupt-driven keypad scanning
 * instead of polling with pico_keypad_get_key().
 *
 * @param enable   true to enable IRQs, false to disable.
 * @param callback User function called on column pin interrupt.
 *                 Signature: void callback(uint gpio, uint32_t events).
 */
void pico_keypad_irq_enable(bool enable, gpio_irq_callback_t callback) {
    for (int i = 0; i < 4; i++) {
        gpio_set_irq_enabled_with_callback(_columns[i], 0x8, enable, callback);
    }
}
