#ifndef __SPI_DEF_H
#define __SPI_DEF_H

#include "cmsis_os2.h"

typedef struct {
    void *handle;
    void(*select)(uint8_t, uint8_t);
    void(*setup)(uint8_t);

    uint8_t *tx;
    uint8_t *rx;
    uint16_t size;
    uint16_t cache_size;

    osMutexId_t mutex;
    osEventFlagsId_t event;
} spi_ctx;

#endif

