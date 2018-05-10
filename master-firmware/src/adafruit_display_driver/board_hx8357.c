#include <ch.h>
#include <hal.h>
#include <gfx.h>
#include "board_hx8357.h"

#ifndef GFX_USE_GDISP
#error
#endif

#define LCD_SPID SPID4
#define LCD_PORT GPIOE
#define LCD_RS_PIN 11

void hx8357_init_board(void *g)
{
    (void) g;
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .cr1 = SPI_CR1_BR_2,
    };
    spiStart(&LCD_SPID, &spi_cfg);
}

void hx8357_acquire_bus(void *g)
{
    (void) g;
    spiAcquireBus(&LCD_SPID);
}

void hx8357_release_bus(void *g)
{
    (void) g;
    spiReleaseBus(&LCD_SPID);
}

void hx8357_write_index(void *g, uint16_t index)
{
    (void) g;
    uint8_t val = index;

    palClearPad(LCD_PORT, LCD_RS_PIN);
    spiSend(&LCD_SPID, 1, &val);
}

void hx8357_write_data(void *g, uint16_t data)
{
    (void) g;
    uint8_t val = data;

    palSetPad(LCD_PORT, LCD_RS_PIN);
    spiSend(&LCD_SPID, 1, &val);
}

#define CACHE_SIZE (100 * 2)
static int cache_depth = 0;
static uint8_t cache[CACHE_SIZE];

void hx8357_write_cache(GDisplay *g, uint16_t c)
{
    cache[cache_depth ++] = c >> 8;
    cache[cache_depth ++] = c & 0xff;

    if (cache_depth == CACHE_SIZE) {
        hx8357_flush(g);
    }
}

void hx8357_flush(GDisplay *g)
{
    (void) g;
    if (cache_depth) {
        palSetPad(LCD_PORT, LCD_RS_PIN);
        spiSend(&LCD_SPID, cache_depth, &cache[0]);
    }
    cache_depth = 0;
}
