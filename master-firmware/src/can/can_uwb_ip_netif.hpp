#pragma once

#include <ch.h>

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>

#include <lwip/err.h>

int can_uwb_ip_netif_init(uavcan::INode& node);
int can_uwb_ip_netif_spin(uavcan::INode& node);

extern "C" {
#endif

event_source_t* can_uwb_ip_netif_get_event_source(void);
err_t lwip_uwb_ip_netif_init(struct netif* interface);
bool lwip_uwb_ip_read(struct netif* netif, struct pbuf** pbuf);

#ifdef __cplusplus
}
#endif
