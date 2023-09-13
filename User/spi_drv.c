#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "common.h"
#include "spi_def.h"
#include "spi_cfg.h"
#include "spi_drv.h"

#define SPI_CS_LINE_MAX             (8U)

#define SPI_TX_COMPLETE             (0x00000001U)
#define SPI_RX_COMPLETE             (0x00000002U)
#define SPI_TXRX_COMPLETE           (0x00000004U)
#define SPI_TRANSFER_ERROR          (0x00000008U)
#define SPI_TRANSFER_MASK           (0xFFFFFFFFU)

typedef struct {
    spi_ctx *ctx;
    uint16_t num;
} spi_dev;

static spi_dev g_devs;

void spi_init(void)
{
    spi_cfg_get((void *)&g_devs.ctx, &g_devs.num);

    return;
}

uint8_t spi_dev_init(uint8_t dev)
{
    uint8_t ret = RET_ERROR;

    if (dev >= g_devs.num) {
        goto exit;
    }
    //memset(&ctx_tbl[dev], 0, sizeof(spi_ctx));
    if (NULL == (g_devs.ctx[dev].mutex = osMutexNew(NULL))) {
        goto exit;
    }
    if (NULL == (g_devs.ctx[dev].event = osEventFlagsNew(NULL))) {
        goto exit;
    }
    osEventFlagsClear(g_devs.ctx[dev].event, SPI_TRANSFER_MASK);
    ret = RET_OK;;

exit:
    return ret;
}

uint8_t spi_dev_cs(uint8_t dev, uint8_t line, uint8_t state)
{
    uint8_t ret = RET_ERROR;

    if (dev >= g_devs.num || line >= SPI_CS_LINE_MAX) {
        goto exit;
    }
    if (0 == state) {
        osMutexAcquire(g_devs.ctx[dev].mutex, osWaitForever);
        g_devs.ctx[dev].setup(line);
        g_devs.ctx[dev].select(line, 0);
    } else {
        g_devs.ctx[dev].select(line, 1);
        osMutexRelease(g_devs.ctx[dev].mutex);
    }
    ret = RET_OK;

exit:
    return ret;
}

uint8_t spi_dev_transfer(uint8_t dev, uint8_t *send, uint8_t *recv, uint32_t size)
{
    uint8_t ret = RET_ERROR;
    uint32_t evt = osFlagsError;

    if ((dev >= g_devs.num) || (0 == size)) {
        goto exit;
    }

    while (size > 0) {
        if (size > g_devs.ctx[dev].cache_size) {
            g_devs.ctx[dev].size = g_devs.ctx[dev].cache_size;
        } else {
            g_devs.ctx[dev].size = size;
        }
        if (NULL != send) {
            memcpy(g_devs.ctx[dev].tx, send, g_devs.ctx[dev].size);
            send += g_devs.ctx[dev].size;
        }
        if (NULL == send && NULL != recv) {
            if (HAL_OK != HAL_SPI_Receive_DMA(g_devs.ctx[dev].handle, g_devs.ctx[dev].rx, g_devs.ctx[dev].size)) {
                goto exit;
            }
        } else if (NULL != send && NULL == recv) {
            if (HAL_OK != HAL_SPI_Transmit_DMA(g_devs.ctx[dev].handle, g_devs.ctx[dev].tx, g_devs.ctx[dev].size)) {
                goto exit;
            }
        } else if (NULL != send && NULL != recv) {
            if (HAL_OK != HAL_SPI_TransmitReceive_DMA(g_devs.ctx[dev].handle, g_devs.ctx[dev].tx, g_devs.ctx[dev].rx, g_devs.ctx[dev].size)) {
                goto exit;
            }
        } else {
            goto exit;
        }
        evt = osEventFlagsWait(g_devs.ctx[dev].event, SPI_TX_COMPLETE|SPI_RX_COMPLETE|SPI_TXRX_COMPLETE|SPI_TRANSFER_ERROR, osFlagsWaitAny, osWaitForever);
        if (SPI_TX_COMPLETE == evt || SPI_RX_COMPLETE == evt || SPI_TXRX_COMPLETE == evt) {
            if (NULL != recv) {
                memcpy(recv, g_devs.ctx[dev].rx, g_devs.ctx[dev].size);
                recv += g_devs.ctx[dev].size;
            }
        } else {
            goto exit;
        }
        size -= g_devs.ctx[dev].size;
    }
    ret = RET_OK;
    
exit:
    return ret;
}

uint8_t spi_dev_deinit(uint8_t dev)
{
    uint8_t ret = RET_ERROR;

    if (dev >= g_devs.num) {
        goto exit;
    }
    osMutexDelete(g_devs.ctx[dev].mutex);
    osEventFlagsDelete(g_devs.ctx[dev].event);
    ret = RET_OK;;

exit:
    return ret;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t idx = 0;

    for (; idx < g_devs.num; idx++) {
        if (hspi == g_devs.ctx[idx].handle) {
            osEventFlagsSet(g_devs.ctx[idx].event, SPI_RX_COMPLETE);
            return;
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t idx = 0;

    for (; idx < g_devs.num; idx++) {
        if (hspi == g_devs.ctx[idx].handle) {
            osEventFlagsSet(g_devs.ctx[idx].event, SPI_TX_COMPLETE);
            return;
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t idx = 0;

    for (; idx < g_devs.num; idx++) {
        if (hspi == g_devs.ctx[idx].handle) {
            osEventFlagsSet(g_devs.ctx[idx].event, SPI_TXRX_COMPLETE);
            return;
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t idx = 0;

    for (; idx < g_devs.num; idx++) {
        if (hspi == g_devs.ctx[idx].handle) {
            osEventFlagsSet(g_devs.ctx[idx].event, SPI_TRANSFER_ERROR);
            return;
        }
    }
}

