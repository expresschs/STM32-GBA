#include <stddef.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "fatfs.h"
#include "gba.h"
#include "globals.h"
#include "stm32h7_port.h"
#include "stm32h7_emu.h"

#define FRAME_DRAW_EVT    (0x00000001U)
osEventFlagsId_t frame_evt = NULL;

int fps = 0;
int fps_tick = 0;
int frame_cnt = 0;

uint16_t framebuffer[240 * 160];
uint16_t port_pix[2 * PIX_BUFFER_SCREEN_WIDTH * 160];
//uint8_t port_bios[0x4000];
__attribute__((section (".RAM_D1_GBA"))) uint8_t port_vram[0x20000];
__attribute__((section (".RAM_D1_GBA"))) uint8_t port_workRAM[0x40000];
__attribute__((section (".RAM_D1_GBA"))) uint8_t port_oam[0x400];
__attribute__((section (".RAM_D1_GBA"))) uint8_t port_ioMem[0x400];
__attribute__((section (".RAM_D1_GBA"))) uint8_t port_internalRAM[0x8000];
__attribute__((section (".RAM_D1_GBA"))) uint8_t port_paletteRAM[0x400];
__attribute__((section (".RAM_D1_GBA"))) uint8_t libretro_save_buf[0x10000 + 0x2000];
__attribute__((section (".RAM_D1_MDMA"))) FATFS gba_fs;    /* File system object for SD logical drive */
__attribute__((section (".RAM_D1_MDMA"))) FIL gba_file;    /* File object for SD */

void systemMessage(const char *fmt, ...)
{
    /* TODO */
}

void systemDrawScreen(void)
{
    osEventFlagsSet(frame_evt, FRAME_DRAW_EVT);
}

void systemOnWriteDataToSoundBuffer(int16_t *finalWave, int length)
{
    /* TODO */
}

static void _emu_init(void)
{
    CPUSetupBuffers();
    CPUInit(NULL, false);
    CPUReset();
    SetFrameskip(0x2);
}

void stm32h7_draw_init(void)
{
    frame_evt = osEventFlagsNew(NULL);
    lcd_init();
    lcd_set_window(0, 0, 240 - 1, 320 - 1);
    lcd_write_fb((uint8_t*)framebuffer, sizeof(framebuffer)); /* one half */
    lcd_write_fb((uint8_t*)framebuffer, sizeof(framebuffer)); /* another half */
}

void stm32h7_draw_handle(void)
{
    uint16_t *src = pix;
    uint16_t *dst = framebuffer;
    int now = 0;
    int passed = 0;

    if (FRAME_DRAW_EVT == osEventFlagsWait(frame_evt, FRAME_DRAW_EVT, osFlagsWaitAny, osWaitForever)) {
        /* 1. update display */
        for (int y = 0; y < 160; y++) {
            for (int x = 0; x < 240; x++) {
                *dst = ((*src & 0xff00) >> 8) | ((*src & 0xff) << 8);
                dst++;
                src++;
            }
            src += 256 - 240;
        }
        lcd_set_window(0, 80, 240 - 1, 240 - 1);
        lcd_write_fb((uint8_t*)framebuffer, 240 * 160 * 2);
        /* 2. frames statistics*/
         if (0 == ++frame_cnt % 100) {
            now = HAL_GetTick();
            passed = now - fps_tick;
            fps_tick = now;
            fps = 100 * 1000 / passed;
        }
        /* 3. update keypad */
        joy = key_read();
        UpdateJoypad();
    }
}

static void _load_game(void)
{
    uint32_t br = 0;
    uint8_t *prom = (uint8_t *)0xC0000000;

    if (FR_OK == (retSD = f_mount(&gba_fs, (TCHAR const*)SDPath, 1))) {
        if (FR_OK == (retSD = f_open(&gba_file, "test.gba", FA_READ ))) {
            if (FR_OK == (retSD = f_read(&gba_file, prom, f_size(&gba_file), &br))) {
                rom = prom;
            }
            retSD = f_close(&gba_file);
        }
    }
}

void stm32h7_emu_init(void)
{
    //bios = port_bios;
    vram = port_vram;
    pix = port_pix;
    oam = port_oam;
    ioMem = port_ioMem;
    internalRAM = port_internalRAM;
    workRAM = port_workRAM;
    paletteRAM = port_paletteRAM;
    _load_game();
    _emu_init();
    fps_tick = HAL_GetTick();
}

void stm32h7_emu_handle(void)
{
    while (1) {
        CPULoop();
    }
}

