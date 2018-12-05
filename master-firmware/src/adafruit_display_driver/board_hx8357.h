/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include <ch.h>
#include <hal.h>


void hx8357_init_board(void *g);
void hx8357_acquire_bus(void *g);
void hx8357_release_bus(void *g);
void hx8357_write_index(void *g, uint16_t index);
void hx8357_write_data(void *g, uint16_t data);
void hx8357_write_cache(void *g, uint16_t c);
void hx8357_flush(void *g);

#endif /* _GDISP_LLD_BOARD_H */
