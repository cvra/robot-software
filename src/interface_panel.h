#ifndef INTERFACE_PANEL_H
#define INTERFACE_PANEL_H

#include <lwip/api.h>

/* Host / port press to send the callback on button press to. */
#define BUTTON_PRESS_CALLBACK_PORT 20002
#define BUTTON_PRESS_CALLBACK_HOST(addr) LWIP_GATEWAY(addr)

#ifdef __cplusplus
extern "C" {
#endif

void interface_panel_init(void);

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_PANEL_H */
