
#include <string.h>
#include "bus_enumerator.h"


static uint16_t midpoint(uint16_t min, uint16_t max)
{
    if (min < max) {
        return (min + max) / 2;
    } else {
        return min;
    }
}

static uint16_t index_by_str_id(const bus_enumerator_t *en, const char *str_id)
{
    uint16_t imin = 0, imax = en->nb_entries_str_to_can, imid;

    while (imax > imin) {
        imid = midpoint(imin, imax);

        if (strcmp(str_id, en->str_to_can[imid].str_id) == 0) {
            return imid;
        } else if (strcmp(str_id, en->str_to_can[imid].str_id) > 0) {
            imin = imid + 1;
        } else {
            imax = imid;
        }
    }

    return BUS_ENUMERATOR_INDEX_NOT_FOUND;
}

static uint16_t index_by_can_id(const bus_enumerator_t *en, uint8_t can_id)
{
    uint16_t imin = 0, imax = en->nb_entries_can_to_str, imid;

    while (imax > imin) {
        imid = midpoint(imin, imax);

        if (can_id == en->can_to_str[imid].can_id) {
            return imid;
        } else if (can_id > en->can_to_str[imid].can_id) {
            imin = imid + 1;
        } else {
            imax = imid;
        }
    }

    return BUS_ENUMERATOR_INDEX_NOT_FOUND;
}

void bus_enumerator_init(bus_enumerator_t *en,
                         struct bus_enumerator_entry_allocator *buffer,
                         uint16_t buffer_len)
{
    en->str_to_can = (bus_enumerator_entry_t*)buffer;
    en->can_to_str = (bus_enumerator_entry_t*)buffer + buffer_len;
    en->buffer_len = buffer_len;
    en->nb_entries_str_to_can = 0;
    en->nb_entries_can_to_str = 0;
}

void bus_enumerator_add_node(bus_enumerator_t *en, const char *str_id, void *driver)
{
    if (en->nb_entries_str_to_can < en->buffer_len) {
        uint16_t imin = 0, imax = en->nb_entries_str_to_can, imid;

        while (imin != imax) {
            imid = midpoint(imin, imax);

            if (strcmp(str_id, en->str_to_can[imid].str_id) > 0) {
                imin = imid + 1;
            } else {
                imax = imid;
            }
        }

        memmove(&en->str_to_can[imin + 1],
                &en->str_to_can[imin],
                (en->nb_entries_str_to_can - imin) * sizeof(bus_enumerator_entry_t));

        en->str_to_can[imin].str_id = str_id;
        en->str_to_can[imin].driver = driver;
        en->str_to_can[imin].can_id = BUS_ENUMERATOR_CAN_ID_NOT_SET;

        en->nb_entries_str_to_can++;
    }
}

void bus_enumerator_update_node_info(bus_enumerator_t *en, const char *str_id, uint8_t can_id)
{
    uint16_t index;

    index = index_by_str_id(en, str_id);

    if (index != BUS_ENUMERATOR_INDEX_NOT_FOUND &&
        en->str_to_can[index].can_id == BUS_ENUMERATOR_CAN_ID_NOT_SET) {

        // set CAN ID in string_ID->CAN_ID buffer
        en->str_to_can[index].can_id = can_id;

        // insert enumerator entry in CAN_ID->string_ID buffer
        uint16_t imin = 0, imax = en->nb_entries_can_to_str, imid;

        while (imin != imax) {
            imid = midpoint(imin, imax);

            if (can_id > en->can_to_str[imid].can_id) {
                imin = imid + 1;
            } else {
                imax = imid;
            }
        }

        memmove(&en->can_to_str[imin + 1],
                &en->can_to_str[imin],
                (en->nb_entries_can_to_str - imin) * sizeof(bus_enumerator_entry_t));

        memcpy(&en->can_to_str[imin],
               &en->str_to_can[index],
               sizeof(bus_enumerator_entry_t));

        en->nb_entries_can_to_str++;
    }
}

uint16_t bus_enumerator_get_number_of_entries(bus_enumerator_t *en)
{
    return en->nb_entries_str_to_can;
}

uint8_t bus_enumerator_get_can_id(bus_enumerator_t *en, const char *str_id)
{
    uint16_t index;
    index = index_by_str_id(en, str_id);

    if (index != BUS_ENUMERATOR_INDEX_NOT_FOUND) {
        return en->str_to_can[index].can_id;
    } else {
        return BUS_ENUMERATOR_STRING_ID_NOT_FOUND;
    }
}

void *bus_enumerator_get_driver(bus_enumerator_t *en, const char *str_id)
{
    uint16_t index;
    index = index_by_str_id(en, str_id);

    if (index != BUS_ENUMERATOR_INDEX_NOT_FOUND) {
        return en->str_to_can[index].driver;
    } else {
        return NULL;
    }
}

void *bus_enumerator_get_driver_by_can_id(bus_enumerator_t *en, uint8_t can_id)
{
    uint16_t index;
    index = index_by_can_id(en, can_id);

    if (index != BUS_ENUMERATOR_INDEX_NOT_FOUND) {
        return en->can_to_str[index].driver;
    } else {
        return NULL;
    }
}

const char *bus_enumerator_get_str_id(bus_enumerator_t *en, uint8_t can_id)
{
    uint16_t index;
    index = index_by_can_id(en, can_id);

    if (index != BUS_ENUMERATOR_INDEX_NOT_FOUND) {
        return en->can_to_str[index].str_id;
    } else {
        return NULL;
    }
}
