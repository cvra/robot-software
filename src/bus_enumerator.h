/*

# Bus Enumerator

this module maps CAN ids to string id and vice versa
It also associates a driver (void*) to each known CAN node
This allows to use actuators by name without modifying the master fw

the motor manager adds nodes using bus_enumerator_add_node() when he needs them
for a trajectory. The bus enumerator then finds the node on the CAN-bus and
associates the ID.

 */

#include <stdint.h>


#define BUS_ENUMERATOR_CAN_ID_NOT_SET   0xFF


typedef struct {
    char *str_id;
    uint8_t can_id;
    void* driver;

} bus_enumerator_entry_t;

typedef struct {
    bus_enumerator_entry_t *buffer;
    uint16_t buffer_len;
} bus_enumerator_t;

void bus_enumerator_init(bus_enumerator_t *en, bus_enumerator_entry_t *buffer, uint16_t buffer_len);

// only a reference of str_id is stored
void bus_enumerator_add_node(bus_enumerator_t *en, const char *str_id, void *driver);

// called by the CAN driver
void bus_enumerator_update_node_info(bus_enumerator_t *en, const char *str_id, uint8_t can_id);

int bus_enumerator_get_can_id(bus_enumerator_t *en, const char *str_id);
void *bus_enumerator_get_driver(bus_enumerator_t *en, const char *str_id);
const char *bus_enumerator_get_str_id(bus_enumerator_t *en, uint8_t can_id);



