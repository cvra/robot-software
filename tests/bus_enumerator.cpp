#include "CppUTest/TestHarness.h"
#include "../src/bus_enumerator.h"

TEST_GROUP(BusEnumeratorTestGroup)
{
    bus_enumerator_t en;

    static const uint16_t buffer_len = 21;
    bus_enumerator_entry_t *buffer = NULL;
    void setup(void)
    {
        // allocate the list of bus enumerators
        buffer = (bus_enumerator_entry_t*)malloc(buffer_len * sizeof(bus_enumerator_entry_t));
    }
};

TEST(BusEnumeratorTestGroup, Init)
{
    bus_enumerator_init(&en, buffer, buffer_len);

    POINTERS_EQUAL(buffer, en.buffer);
    CHECK_EQUAL(buffer_len, en.buffer_len);
}

TEST(BusEnumeratorTestGroup, AddNode)
{
    bus_enumerator_add_node(&en, "ID", (void*)NULL);
}
