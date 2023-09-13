#include "stm32h7xx_hal.h"
#include "spi_def.h"
#include "spi_cfg.h"

#define SPI_BASE_DMA_ADDR           (0x30000000UL)

/* SPI5 define */
#define SPI5_LINE_PH5               (0U)
#define SPI5_CACHE_SIZE             (38 * 1024UL)

extern SPI_HandleTypeDef hspi5;
__attribute__((section (".RAM_D3"))) static uint8_t spi5_tx_buf[SPI5_CACHE_SIZE] = {0};

static inline void _spi5_select(uint8_t line, uint8_t cs)
{
    if (SPI5_LINE_PH5 == line) {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, (GPIO_PinState)cs);
    }
}

static inline void _spi5_setup(uint8_t line)
{
    if (SPI5_LINE_PH5 == line) {
        /* TODO: Setup Params, Baudrate Phase Polarity ...ETC... */
    } else {
        /* Nothing To Do */
    }
}


spi_ctx g_cfg_tbl[] = {
    {&hspi5, _spi5_select, _spi5_setup, spi5_tx_buf, NULL, 0, SPI5_CACHE_SIZE, NULL, NULL},
}; 

void spi_cfg_get(void **cfg, uint16_t *size)
{
    *cfg = (void *)g_cfg_tbl;
    *size = sizeof(g_cfg_tbl)/sizeof(g_cfg_tbl[0]);

    return;
}

