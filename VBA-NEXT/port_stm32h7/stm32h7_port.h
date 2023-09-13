#ifndef STM32H7_PORT_H
#define STM32H7_PORT_H

#ifdef __cplusplus
 extern "C" {
#endif

void lcd_init(void);
void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_write_fb(uint8_t *buf, int len);
uint32_t key_read(void);

#ifdef __cplusplus
}
#endif

#endif

