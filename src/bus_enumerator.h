#ifndef BUS_ENUMERATOR_H
#define BUS_ENUMERATOR_H

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


#ifdef __cplusplus
extern "C" {
#endif

#define BUS_ENUMERATOR_CAN_ID_NOT_SET       0xFF
#define BUS_ENUMERATOR_STRING_ID_NOT_FOUND  0xFE
#define BUS_ENUMERATOR_INDEX_NOT_FOUND      0xFFFF


typedef struct {
    const char *str_id;
    uint8_t can_id;
    void* driver;

} bus_enumerator_entry_t;

// the allocated buffer is internally split in two for bi-directional mapping
struct bus_enumerator_entry_allocator {
    bus_enumerator_entry_t str_to_can;
    bus_enumerator_entry_t can_to_str;
};

typedef struct {
    bus_enumerator_entry_t *str_to_can;
    bus_enumerator_entry_t *can_to_str;
    uint16_t buffer_len;
    uint16_t nb_entries_str_to_can;
    uint16_t nb_entries_can_to_str;
} bus_enumerator_t;

void bus_enumerator_init(bus_enumerator_t *en,
                         struct bus_enumerator_entry_allocator *buffer,
                         uint16_t buffer_len);

// only a reference of str_id is stored
void bus_enumerator_add_node(bus_enumerator_t *en, const char *str_id, void *driver);

// called by the CAN driver
void bus_enumerator_update_node_info(bus_enumerator_t *en, const char *str_id, uint8_t can_id);

uint16_t bus_enumerator_get_number_of_entries(bus_enumerator_t *en);

uint8_t bus_enumerator_get_can_id(bus_enumerator_t *en, const char *str_id);
void *bus_enumerator_get_driver(bus_enumerator_t *en, const char *str_id);
void *bus_enumerator_get_driver_by_can_id(bus_enumerator_t *en, uint8_t can_id);
const char *bus_enumerator_get_str_id(bus_enumerator_t *en, uint8_t can_id);



#ifdef __cplusplus
}
#endif

#endif /* BUS_ENUMERATOR_H */
