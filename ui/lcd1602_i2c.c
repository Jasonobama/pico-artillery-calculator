/**
 * @file lcd1602_i2c.c
 * @brief I2C LCD1602 driver using PCF8574 I/O expander backpack in 4-bit mode.
 *
 * This module drives a standard HD44780-compatible 1602 LCD through a PCF8574
 * I2C-to-parallel backpack (common address 0x27). Communication uses the RP2040
 * I2C0 peripheral. The LCD is operated in 4-bit mode: each 8-bit data/command
 * byte is split into two 4-bit nibbles sent over the upper data lines (D4-D7).
 * The PCF8574 backlight pin (P3) is held high at all times for visibility.
 *
 * @note Timing constants follow HD44780 datasheet: 600 us for enable pulse
 *       and inter-nibble delay, 50 ms power-on stabilization.
 *
 * Hardware wiring (defaults, overridable via -D defines):
 *   SDA = GP4, SCL = GP5, I2C addr = 0x27, baud = 100 kHz
 */

#include "lcd1602_i2c.h"
#include "hardware/i2c.h"
#include <string.h>

/** @name HD44780 instruction register bits (8-bit mode command words) */
/** @{ */
#define LCD_CLEARDISPLAY   0x01  /**< Clear entire display, reset cursor */
#define LCD_ENTRYMODESET   0x04  /**< Entry mode set base command */
#define LCD_DISPLAYCONTROL 0x08  /**< Display on/off control base command */
#define LCD_FUNCTIONSET    0x20  /**< Function set base command */
/** @} */

/** @name Entry mode flags (OR with LCD_ENTRYMODESET) */
/** @{ */
#define LCD_ENTRYLEFT      0x02  /**< Increment cursor after write (left-to-right) */
/** @} */

/** @name Display control flags (OR with LCD_DISPLAYCONTROL) */
/** @{ */
#define LCD_BLINKON        0x01  /**< Enable cursor blink */
#define LCD_CURSORON       0x02  /**< Show underline cursor */
#define LCD_DISPLAYON      0x04  /**< Turn display on (no display if clear) */
/** @} */

/** @name Function set flags (OR with LCD_FUNCTIONSET) */
/** @{ */
#define LCD_2LINE          0x08  /**< 2-line mode (1602); clear for 1-line */
/** @} */

/** @name PCF8574 backpack bit assignments */
/** @{ */
#define LCD_BACKLIGHT      0x08  /**< P3 on PCF8574: backlight control (active high) */
#define LCD_ENABLE_BIT     0x04  /**< P2 on PCF8574: enable strobe (active high pulse) */
/** @} */

#define LCD_CHARACTER      1     /**< RS=1: send data to DDRAM (character) */
#define LCD_COMMAND        0     /**< RS=0: send instruction to IR (command) */
#define DELAY_US           600   /**< Enable pulse width + settling time (us) */

/**
 * @brief Write a single byte directly to the I2C expander.
 *
 * @param val 8-bit value to output on PCF8574 port pins.
 */
static inline void i2c_write_byte(uint8_t val) {
    i2c_write_blocking(LCD1602_I2C_PORT, LCD1602_I2C_ADDR, &val, 1, false);
}

/**
 * @brief Strobe the LCD enable pin high then low with timing delays.
 *
 * HD44780 latches data on the falling edge of E. This function:
 *   1. Sets E=1 on the expander (data must already be on the bus).
 *   2. Waits for the minimum enable pulse width.
 *   3. Sets E=0 to latch.
 *   4. Waits for the minimum inter-operation delay.
 *
 * @param val Current PCF8574 port state with data lines set but E=0.
 *            The backlight bit (P3) must already be included in val.
 */
static void lcd_toggle_enable(uint8_t val) {
    sleep_us(DELAY_US);
    i2c_write_byte(val | LCD_ENABLE_BIT);   /**< Raise E while holding data */
    sleep_us(DELAY_US);
    i2c_write_byte(val & ~LCD_ENABLE_BIT);  /**< Drop E to latch; data sampled on falling edge */
    sleep_us(DELAY_US);
}

/**
 * @brief Send one byte to the LCD as command (RS=0) or character data (RS=1).
 *
 * In 4-bit mode, the upper nibble is sent first (D7-D4), then the lower
 * nibble shifted into the upper position. RS is carried in bit 0 of the
 * PCF8574 port (P0). Backlight is held on for both nibbles.
 *
 * @param val  8-bit value to send to the LCD.
 * @param mode LCD_COMMAND (0) or LCD_CHARACTER (1); controls the RS line.
 */
static void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;      /**< Upper nibble + RS + BL */
    uint8_t low  = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT; /**< Lower nibble shifted up + RS + BL */
    i2c_write_byte(high);
    lcd_toggle_enable(high);   /**< Latch upper nibble */
    i2c_write_byte(low);
    lcd_toggle_enable(low);    /**< Latch lower nibble */
}

/**
 * @brief Initialize the I2C peripheral and LCD module.
 *
 * Sequence:
 *   1. Configure I2C0 with specified pins (SDA=GP4, SCL=GP5 by default) and
 *      enable internal pull-ups (required for I2C open-drain).
 *   2. Wait 50 ms for LCD power-on stabilization.
 *   3. Force 8-bit mode with three 0x03 writes (HD44780 "wake-up" sequence;
 *      if already in 4-bit mode, this is a no-op).
 *   4. Switch to 4-bit mode with 0x02.
 *   5. Set entry mode (left-to-right, no display shift).
 *   6. Set function (4-bit, 2-line, 5x8 dots).
 *   7. Turn display on (no cursor, no blink).
 *   8. Clear display.
 */
void lcd1602_init(void) {
    i2c_init(LCD1602_I2C_PORT, LCD1602_I2C_BAUD);
    gpio_set_function(LCD1602_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LCD1602_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LCD1602_SDA_PIN);   /**< I2C requires pull-ups on SDA/SCL */
    gpio_pull_up(LCD1602_SCL_PIN);
    sleep_ms(50);                    /**< Wait for LCD power-on (Vdd rise > 4.5V) */

    lcd_send_byte(0x03, LCD_COMMAND); /**< Wake-up sequence: send 0x03 three times */
    lcd_send_byte(0x03, LCD_COMMAND); /**< to force 8-bit mode regardless of */
    lcd_send_byte(0x03, LCD_COMMAND); /**< current state. */
    lcd_send_byte(0x02, LCD_COMMAND); /**< Switch to 4-bit mode */
    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND); /**< Entry: increment, no shift */
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);      /**< 4-bit, 2-line, 5x8 font */
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND); /**< Display on, cursor off */
    lcd1602_clear();
}

/**
 * @brief Clear the LCD display and home the cursor.
 *
 * Sends the clear display command (0x01). The HD44780 requires up to 1.52 ms
 * to execute this; timing is satisfied by the internal DELAY_US pacing in
 * lcd_toggle_enable().
 */
void lcd1602_clear(void) {
    lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}

/**
 * @brief Set the cursor to a specific line and column position.
 *
 * DDRAM addresses for a 1602:
 *   - Line 0: 0x00-0x0F (command base 0x80)
 *   - Line 1: 0x40-0x4F (command base 0xC0)
 *
 * @param line Display line: 0 for first row, 1 for second row.
 * @param pos  Column position (0-15). Behavior undefined if out of range.
 */
void lcd1602_set_cursor(int line, int pos) {
    int val = (line == 0) ? 0x80 + pos : 0xC0 + pos; /**< Set DDRAM address command */
    lcd_send_byte(val, LCD_COMMAND);
}

/**
 * @brief Write a single character to the LCD at the current cursor position.
 *
 * The cursor auto-increments after each character (entry mode set to left-to-right).
 *
 * @param c ASCII character to display. Must be in the HD44780 character ROM.
 */
void lcd1602_char(char c) {
    lcd_send_byte(c, LCD_CHARACTER);
}

/**
 * @brief Write a null-terminated string to the LCD.
 *
 * Characters are sent sequentially; no line-wrap is performed.
 * String must fit within the remaining characters on the current line,
 * or wrap behavior follows HD44780 internal DDRAM mapping (line 0 wraps
 * to line 1 at 0x40; line 1 does not wrap back to line 0).
 *
 * @param s Pointer to null-terminated C string.
 */
void lcd1602_string(const char *s) {
    while (*s) lcd1602_char(*s++);
}
