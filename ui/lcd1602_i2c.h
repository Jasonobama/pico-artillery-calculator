#ifndef LCD1602_I2C_H
#define LCD1602_I2C_H

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LCD1602_I2C_PORT
#define LCD1602_I2C_PORT    i2c0
#endif

#ifndef LCD1602_SDA_PIN
#define LCD1602_SDA_PIN     4
#endif

#ifndef LCD1602_SCL_PIN
#define LCD1602_SCL_PIN     5
#endif

#ifndef LCD1602_I2C_ADDR
#define LCD1602_I2C_ADDR    0x27
#endif

#ifndef LCD1602_I2C_BAUD
#define LCD1602_I2C_BAUD    100000
#endif

void lcd1602_init(void);
void lcd1602_clear(void);
void lcd1602_set_cursor(int line, int pos);
void lcd1602_string(const char *s);
void lcd1602_char(char c);

#ifdef __cplusplus
}
#endif

#endif
