#include "hardware/i2c.h"

#ifndef SSD1306_H__
#define SSD1306_H__

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETSTARTLINE 0x40

void ssd1306_use_i2c(i2c_inst_t *i2c_instance);
void ssd1306_setup(void);
void ssd1306_update(void);
void ssd1306_clear(void);
void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color);
void ssd1306_command(unsigned char c);

#endif
