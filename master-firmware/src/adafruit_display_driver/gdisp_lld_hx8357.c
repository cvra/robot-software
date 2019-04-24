#include <gfx.h>

#if defined(GDISP_SCREEN_HEIGHT) || defined(GDISP_SCREEN_WIDTH)
#error "Cannot set screen size for this driver."
#endif

#define GDISP_DRIVER_VMT GDISPVMT_HX8357
#include "gdisp_lld_config.h"
#include "src/gdisp/gdisp_driver.h"

#include "board_hx8357.h"

#define GDISP_SCREEN_HEIGHT 480
#define GDISP_SCREEN_WIDTH 320
#include "HX8357.h"

static void set_viewport(GDisplay* g);
static void write_reg(GDisplay* g, uint16_t index, uint16_t data);
static void write_cache_data16(GDisplay* g, uint16_t data);

LLDSPEC gBool gdisp_lld_init(GDisplay* g)
{
    // No private area for this controller
    g->priv = 0;

    // Initialise the board interface
    hx8357_init_board(g);

    //// Get the bus for the following initialisation commands
    hx8357_acquire_bus(g);
    hx8357_write_index(g, HX8357_SWRESET);
    gfxSleepMilliseconds(500);

    // setextc
    hx8357_write_index(g, HX8357D_SETC);
    hx8357_write_data(g, 0xFF);
    hx8357_write_data(g, 0x83);
    hx8357_write_data(g, 0x57);
    gfxSleepMilliseconds(300);

    // setRGB which also enables SDO
    hx8357_write_index(g, HX8357_SETRGB);
    hx8357_write_data(g, 0x80); // enable SDO pin!
    hx8357_write_data(g, 0x0);
    hx8357_write_data(g, 0x06);
    hx8357_write_data(g, 0x06);

    hx8357_write_index(g, HX8357D_SETCOM);
    hx8357_write_data(g, 0x25); // -1.52V

    hx8357_write_index(g, HX8357_SETOSC);
    hx8357_write_data(g, 0x68); // Normal mode 70Hz, Idle mode 55 Hz

    hx8357_write_index(g, HX8357_SETPANEL); // Set Panel
    hx8357_write_data(g, 0x05); // BGR, Gate direction swapped

    hx8357_write_index(g, HX8357_SETPWR1);
    hx8357_write_data(g, 0x00); // Not deep standby
    hx8357_write_data(g, 0x15); // BT
    hx8357_write_data(g, 0x1C); // VSPR
    hx8357_write_data(g, 0x1C); // VSNR
    hx8357_write_data(g, 0x83); // AP
    hx8357_write_data(g, 0xAA); // FS

    hx8357_write_index(g, HX8357D_SETSTBA);
    hx8357_write_data(g, 0x50); // OPON normal
    hx8357_write_data(g, 0x50); // OPON idle
    hx8357_write_data(g, 0x01); // STBA
    hx8357_write_data(g, 0x3C); // STBA
    hx8357_write_data(g, 0x1E); // STBA
    hx8357_write_data(g, 0x08); // GEN

    hx8357_write_index(g, HX8357D_SETCYC);
    hx8357_write_data(g, 0x02); // NW 0x02
    hx8357_write_data(g, 0x40); // RTN
    hx8357_write_data(g, 0x00); // DIV
    hx8357_write_data(g, 0x2A); // DUM
    hx8357_write_data(g, 0x2A); // DUM
    hx8357_write_data(g, 0x0D); // GDON
    hx8357_write_data(g, 0x78); // GDOFF

    hx8357_write_index(g, HX8357D_SETGAMMA);
    hx8357_write_data(g, 0x02);
    hx8357_write_data(g, 0x0A);
    hx8357_write_data(g, 0x11);
    hx8357_write_data(g, 0x1d);
    hx8357_write_data(g, 0x23);
    hx8357_write_data(g, 0x35);
    hx8357_write_data(g, 0x41);
    hx8357_write_data(g, 0x4b);
    hx8357_write_data(g, 0x4b);
    hx8357_write_data(g, 0x42);
    hx8357_write_data(g, 0x3A);
    hx8357_write_data(g, 0x27);
    hx8357_write_data(g, 0x1B);
    hx8357_write_data(g, 0x08);
    hx8357_write_data(g, 0x09);
    hx8357_write_data(g, 0x03);
    hx8357_write_data(g, 0x02);
    hx8357_write_data(g, 0x0A);
    hx8357_write_data(g, 0x11);
    hx8357_write_data(g, 0x1d);
    hx8357_write_data(g, 0x23);
    hx8357_write_data(g, 0x35);
    hx8357_write_data(g, 0x41);
    hx8357_write_data(g, 0x4b);
    hx8357_write_data(g, 0x4b);
    hx8357_write_data(g, 0x42);
    hx8357_write_data(g, 0x3A);
    hx8357_write_data(g, 0x27);
    hx8357_write_data(g, 0x1B);
    hx8357_write_data(g, 0x08);
    hx8357_write_data(g, 0x09);
    hx8357_write_data(g, 0x03);
    hx8357_write_data(g, 0x00);
    hx8357_write_data(g, 0x01);

    hx8357_write_index(g, HX8357_COLMOD);
    hx8357_write_data(g, 0x55); // 16 bit

    hx8357_write_index(g, HX8357_MADCTL);
    hx8357_write_data(g, 0xC0);

    hx8357_write_index(g, HX8357_TEON); // TE off
    hx8357_write_data(g, 0x00);

    hx8357_write_index(g, HX8357_TEARLINE); // tear line
    hx8357_write_data(g, 0x00);
    hx8357_write_data(g, 0x02);

    hx8357_write_index(g, HX8357_SLPOUT); // Exit Sleep
    gfxSleepMilliseconds(100);

    hx8357_write_index(g, HX8357_DISPON); // display on

    // Release the bus
    hx8357_release_bus(g);

    /* Initialise the GDISP structure */
    g->g.Width = GDISP_SCREEN_WIDTH;
    g->g.Height = GDISP_SCREEN_HEIGHT;
    g->g.Orientation = gOrientation0;
    g->g.Powermode = gPowerOn;
    g->g.Backlight = 100;
    g->g.Contrast = 100;
    return TRUE;
}

LLDSPEC void gdisp_lld_write_start(GDisplay* g)
{
    hx8357_acquire_bus(g);
    set_viewport(g);
    hx8357_write_index(g, 0x2C);
}
LLDSPEC void gdisp_lld_write_color(GDisplay* g)
{
    uint16_t color = gdispColor2Native(g->p.color);
    write_cache_data16(g, color);
}
LLDSPEC void gdisp_lld_write_stop(GDisplay* g)
{
    hx8357_release_bus(g);
    hx8357_flush(g);
}

LLDSPEC void gdisp_lld_control(GDisplay* g)
{
    switch (g->p.x) {
        case GDISP_CONTROL_ORIENTATION:
            if (g->g.Orientation == (gOrientation)g->p.ptr) {
                return;
            }
            switch ((gOrientation)g->p.ptr) {
                /* X and Y axes non-inverted */
                case gOrientation0:
                    hx8357_acquire_bus(g);
                    write_reg(g, HX8357_MADCTL, HX8357_MADCTL_MY | HX8357_MADCTL_MX);
                    hx8357_release_bus(g);
                    g->g.Height = GDISP_SCREEN_HEIGHT;
                    g->g.Width = GDISP_SCREEN_WIDTH;
                    break;

                /* Invert X and Y axes */
                case gOrientation90:
                    hx8357_acquire_bus(g);
                    write_reg(g, HX8357_MADCTL, HX8357_MADCTL_MY | HX8357_MADCTL_MV);
                    hx8357_release_bus(g);
                    g->g.Height = GDISP_SCREEN_WIDTH;
                    g->g.Width = GDISP_SCREEN_HEIGHT;
                    break;

                case gOrientation180:
                    hx8357_acquire_bus(g);
                    write_reg(g, HX8357_MADCTL, 0x00);
                    hx8357_release_bus(g);
                    g->g.Height = GDISP_SCREEN_HEIGHT;
                    g->g.Width = GDISP_SCREEN_WIDTH;
                    break;

                case gOrientation270:
                    hx8357_acquire_bus(g);
                    write_reg(g, HX8357_MADCTL, HX8357_MADCTL_MX | HX8357_MADCTL_MV);
                    hx8357_release_bus(g);
                    g->g.Height = GDISP_SCREEN_WIDTH;
                    g->g.Width = GDISP_SCREEN_HEIGHT;
                    break;

                default:
                    return;
            }
            g->g.Orientation = (gOrientation)g->p.ptr;
            return;

        default:
            return;
    }
}

static void set_viewport(GDisplay* g)
{
    hx8357_write_index(g, 0x2A);

    hx8357_write_data(g, (g->p.x >> 8));
    hx8357_write_data(g, (uint8_t)g->p.x);
    hx8357_write_data(g, (g->p.x + g->p.cx - 1) >> 8);
    hx8357_write_data(g, (uint8_t)(g->p.x + g->p.cx - 1));

    hx8357_write_index(g, 0x2B);
    hx8357_write_data(g, (g->p.y >> 8));
    hx8357_write_data(g, (uint8_t)g->p.y);
    hx8357_write_data(g, (g->p.y + g->p.cy - 1) >> 8);
    hx8357_write_data(g, (uint8_t)(g->p.y + g->p.cy - 1));
    hx8357_write_index(g, 0x2C);
}

static void write_reg(GDisplay* g, uint16_t index, uint16_t data)
{
    hx8357_write_index(g, index);
    hx8357_write_data(g, data);
}

static void write_cache_data16(GDisplay* g, uint16_t data)
{
    hx8357_write_cache(g, data);
}
