#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "spi_drv.h"
#include "stm32h7_port.h"

#define USE_HORIZONTAL      (0UL)

#define LCD_SPI_DEV         (0UL)
#define LCD_DELAY(ms)       osDelay(ms)
#define LCD_RST(v)          /* HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, (GPIO_PinState)v) */
#define LCD_DC(v)           HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_11, (GPIO_PinState)v)

#define KEY_UP()            HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12)
#define KEY_DOWN()          HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10)
#define KEY_RIGHT()         HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9)
#define KEY_LEFT()          HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11)
#define KEY_A()             HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_8)
#define KEY_B()             HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_13)
#define KEY_SELECT()        HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_5)
#define KEY_START()         HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11)

static void _lcd_write(uint8_t *buf, int len, int isCmd)
{
    spi_dev_cs(LCD_SPI_DEV, 0, 0);
    LCD_DC(!isCmd);
    spi_dev_transfer(LCD_SPI_DEV, buf, NULL, len);
    LCD_DC(1);
    spi_dev_cs(LCD_SPI_DEV, 0, 1);
}

static void _lcd_cmd8(uint8_t cmd)
{
    _lcd_write(&cmd, 1, 1);
}
static void _lcd_dat8(uint8_t dat)
{
    _lcd_write(&dat, 1, 0);
}
static void _lcd_dat16(uint16_t dat)
{
    uint8_t buf[2] = {dat >> 8, dat & 0xff};
    _lcd_write(buf, 2, 0);
}

void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (USE_HORIZONTAL == 0) {
        _lcd_cmd8(0x2a); /* Column Address set */
        _lcd_dat16(x1);
        _lcd_dat16(x2);
        _lcd_cmd8(0x2b); /* Row Address set */
        _lcd_dat16(y1);
        _lcd_dat16(y2);
        _lcd_cmd8(0x2c); /* Write to RAM */
    } else if (USE_HORIZONTAL == 1) {
        _lcd_cmd8(0x2a); /* Column Address set */
        _lcd_dat16(x1);
        _lcd_dat16(x2);
        _lcd_cmd8(0x2b); /* Row Address set */
        _lcd_dat16(y1 + 80);
        _lcd_dat16(y2 + 80);
        _lcd_cmd8(0x2c); /* Write to RAM */
    } else if (USE_HORIZONTAL == 2) {
        _lcd_cmd8(0x2a); /* Column Address set */
        _lcd_dat16(x1);
        _lcd_dat16(x2);
        _lcd_cmd8(0x2b); /* Row Address set */
        _lcd_dat16(y1);
        _lcd_dat16(y2);
        _lcd_cmd8(0x2c); /* Write to RAM */
    } else {
        _lcd_cmd8(0x2a); /* Column Address set */
        _lcd_dat16(x1 + 80);
        _lcd_dat16(x2 + 80);
        _lcd_cmd8(0x2b); /* Row Address set */
        _lcd_dat16(y1);
        _lcd_dat16(y2);
        _lcd_cmd8(0x2c); /* Write to RAM */
    }
}

void lcd_write_fb(uint8_t *buf, int len)
{
    spi_dev_cs(LCD_SPI_DEV, 0, 0);
    spi_dev_transfer(LCD_SPI_DEV, buf, NULL, len);
    spi_dev_cs(LCD_SPI_DEV, 0, 1);
}

void lcd_init(void)
{
    /* spi init */
    spi_dev_init(LCD_SPI_DEV);
    /* reset */
    LCD_DELAY(25);
    LCD_RST(0);
    LCD_DELAY(25);
    LCD_RST(1);
    LCD_DELAY(50);

    //************* Start Initial Sequence **********//
    _lcd_cmd8(0x36);
    if (USE_HORIZONTAL == 0) {
        _lcd_dat8(0x00);
    } else if (USE_HORIZONTAL == 1) {
        _lcd_dat8(0xC0);
    }else if (USE_HORIZONTAL == 2) {
        _lcd_dat8(0x70);
    } else {
        _lcd_dat8(0xA0);
    }

    _lcd_cmd8(0x3A);
    _lcd_dat8(0x05);

    _lcd_cmd8(0xB2);
    _lcd_dat8(0x0C);
    _lcd_dat8(0x0C);
    _lcd_dat8(0x00);
    _lcd_dat8(0x33);
    _lcd_dat8(0x33);

    _lcd_cmd8(0xB7);
    _lcd_dat8(0x35);

    _lcd_cmd8(0xBB);
    _lcd_dat8(0x19);

    _lcd_cmd8(0xC0);
    _lcd_dat8(0x2C);

    _lcd_cmd8(0xC2);
    _lcd_dat8(0x01);

    _lcd_cmd8(0xC3);
    _lcd_dat8(0x12);

    _lcd_cmd8(0xC4);
    _lcd_dat8(0x20);

    _lcd_cmd8(0xC6);
    _lcd_dat8(0x0F);

    _lcd_cmd8(0xD0);
    _lcd_dat8(0xA4);
    _lcd_dat8(0xA1);

    _lcd_cmd8(0xE0);
    _lcd_dat8(0xD0);
    _lcd_dat8(0x04);
    _lcd_dat8(0x0D);
    _lcd_dat8(0x11);
    _lcd_dat8(0x13);
    _lcd_dat8(0x2B);
    _lcd_dat8(0x3F);
    _lcd_dat8(0x54);
    _lcd_dat8(0x4C);
    _lcd_dat8(0x18);
    _lcd_dat8(0x0D);
    _lcd_dat8(0x0B);
    _lcd_dat8(0x1F);
    _lcd_dat8(0x23);

    _lcd_cmd8(0xE1);
    _lcd_dat8(0xD0);
    _lcd_dat8(0x04);
    _lcd_dat8(0x0C);
    _lcd_dat8(0x11);
    _lcd_dat8(0x13);
    _lcd_dat8(0x2C);
    _lcd_dat8(0x3F);
    _lcd_dat8(0x44);
    _lcd_dat8(0x51);
    _lcd_dat8(0x2F);
    _lcd_dat8(0x1F);
    _lcd_dat8(0x1F);
    _lcd_dat8(0x20);
    _lcd_dat8(0x23);
    _lcd_cmd8(0x21);

    _lcd_cmd8(0x11);  // Sleep out
    LCD_DELAY(120);   // Delay 120ms

    _lcd_cmd8(0x29);
}

uint32_t key_read(void)
{
    uint32_t ret = 0;

    if (0 == KEY_A()) ret |= 1 << 0;
    if (0 == KEY_B()) ret |= 1 << 1;
    if (0 == KEY_SELECT()) ret |= 1 << 2;
    if (0 == KEY_START()) ret |= 1 << 3;
    if (0 == KEY_RIGHT()) ret |= 1 << 4;
    if (0 == KEY_LEFT()) ret |= 1 << 5;
    if (0 == KEY_UP()) ret |= 1 << 6;
    if (0 == KEY_DOWN()) ret |= 1 << 7;

    return ret;
}

