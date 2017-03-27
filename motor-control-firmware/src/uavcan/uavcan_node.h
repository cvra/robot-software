#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct uavcan_node_arg {
    const char *node_name;
    uint8_t node_id:7;
};

void uavcan_node_start(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* UAVCAN_NODE_H */