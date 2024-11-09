#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "main.h"

#define SLAVE_ADDRESS_LCD 0x4E   // change this according to the I2C device

// Function prototypes
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_init (void);
void lcd_send_string (char *str);
void lcd_clear(void);
void lcd_put_cursor(int row, int col);

#endif /* INC_LCD_I2C_H_ */
