#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    INIT,
    READY,
    RUNNING,
    ARRIVED
};
extern int electron_state;

void uavcan_start(unsigned int node_id, const char* node_name);

#ifdef __cplusplus
}
#endif

#endif
