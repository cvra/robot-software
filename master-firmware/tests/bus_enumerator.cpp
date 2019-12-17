#include "CppUTest/TestHarness.h"
#include "can/bus_enumerator.h"

#define SMALL_STR_ID "bar"
#define MEDIUM_STR_ID "foo"
#define LARGE_STR_ID "zajebiscie"

#define SMALL_CAN_ID 1
#define MEDIUM_CAN_ID 13
#define LARGE_CAN_ID 42

#define DRIVER_POINTER (void*)42

TEST_GROUP (BusEnumeratorTestGroup) {
    bus_enumerator_t en;

    static const uint16_t buffer_len = 21;
    bus_enumerator_entry_allocator* buffer = NULL;

    void setup(void)
    {
        // allocate the list of bus enumerators
        buffer = (struct bus_enumerator_entry_allocator*)
            malloc(buffer_len * sizeof(struct bus_enumerator_entry_allocator));
        bus_enumerator_init(&en, buffer, buffer_len);
    }
};

TEST(BusEnumeratorTestGroup, Init)
{
    POINTERS_EQUAL(buffer, en.str_to_can);
    POINTERS_EQUAL((bus_enumerator_entry_t*)buffer + buffer_len, en.can_to_str);
    CHECK_EQUAL(buffer_len, en.buffer_len);
    CHECK_EQUAL(0, en.nb_entries_str_to_can);
    CHECK_EQUAL(0, en.nb_entries_can_to_str);
}

TEST(BusEnumeratorTestGroup, AddNode)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    CHECK_EQUAL(1, en.nb_entries_str_to_can);

    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    CHECK_EQUAL(BUS_ENUMERATOR_CAN_ID_NOT_SET, en.str_to_can[0].can_id);
    POINTERS_EQUAL(DRIVER_POINTER, en.str_to_can[0].driver);
}

TEST(BusEnumeratorTestGroup, AddTwoNodes)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);

    CHECK_EQUAL(2, en.nb_entries_str_to_can);

    // first node
    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    CHECK_EQUAL(BUS_ENUMERATOR_CAN_ID_NOT_SET, en.str_to_can[0].can_id);
    POINTERS_EQUAL(DRIVER_POINTER, en.str_to_can[0].driver);

    // second node
    STRCMP_EQUAL(LARGE_STR_ID, en.str_to_can[1].str_id);
    CHECK_EQUAL(BUS_ENUMERATOR_CAN_ID_NOT_SET, en.str_to_can[1].can_id);
    POINTERS_EQUAL(DRIVER_POINTER, en.str_to_can[1].driver);
}

TEST(BusEnumeratorTestGroup, AddTwoNodesWithFlip)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    CHECK_EQUAL(2, en.nb_entries_str_to_can);

    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    STRCMP_EQUAL(LARGE_STR_ID, en.str_to_can[1].str_id);
}

TEST(BusEnumeratorTestGroup, AddThreeNodes)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, MEDIUM_STR_ID, DRIVER_POINTER);

    CHECK_EQUAL(3, en.nb_entries_str_to_can);

    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    STRCMP_EQUAL(MEDIUM_STR_ID, en.str_to_can[1].str_id);
    STRCMP_EQUAL(LARGE_STR_ID, en.str_to_can[2].str_id);
}

TEST(BusEnumeratorTestGroup, UpdateNodeInfo)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);

    CHECK_EQUAL(1, en.nb_entries_str_to_can);
    CHECK_EQUAL(1, en.nb_entries_can_to_str);

    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    CHECK_EQUAL(LARGE_CAN_ID, en.str_to_can[0].can_id);

    CHECK_EQUAL(LARGE_CAN_ID, en.can_to_str[0].can_id);
    STRCMP_EQUAL(SMALL_STR_ID, en.can_to_str[0].str_id);
    POINTERS_EQUAL(DRIVER_POINTER, en.can_to_str[0].driver);
}

TEST(BusEnumeratorTestGroup, UpdateNodeInfoForTwo)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, LARGE_STR_ID, SMALL_CAN_ID);
    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);

    CHECK_EQUAL(2, en.nb_entries_str_to_can);
    CHECK_EQUAL(2, en.nb_entries_can_to_str);

    // string->CAN side
    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    CHECK_EQUAL(LARGE_CAN_ID, en.str_to_can[0].can_id);
    STRCMP_EQUAL(LARGE_STR_ID, en.str_to_can[1].str_id);
    CHECK_EQUAL(SMALL_CAN_ID, en.str_to_can[1].can_id);

    // CAN->string side
    CHECK_EQUAL(SMALL_CAN_ID, en.can_to_str[0].can_id);
    STRCMP_EQUAL(LARGE_STR_ID, en.can_to_str[0].str_id);
    CHECK_EQUAL(LARGE_CAN_ID, en.can_to_str[1].can_id);
    STRCMP_EQUAL(SMALL_STR_ID, en.can_to_str[1].str_id);
}

TEST(BusEnumeratorTestGroup, UpdateNodeInfoWorksOnlyOnce)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);
    bus_enumerator_update_node_info(&en, SMALL_STR_ID, SMALL_CAN_ID);

    STRCMP_EQUAL(SMALL_STR_ID, en.str_to_can[0].str_id);
    CHECK_EQUAL(LARGE_CAN_ID, en.str_to_can[0].can_id);

    STRCMP_EQUAL(SMALL_STR_ID, en.can_to_str[0].str_id);
    CHECK_EQUAL(LARGE_CAN_ID, en.can_to_str[0].can_id);
}

TEST(BusEnumeratorTestGroup, GetNumberOfEntries)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    CHECK_EQUAL(2, bus_enumerator_total_nodes_count(&en));
}

TEST(BusEnumeratorTestGroup, ZeroDiscoveredEntriesAtFirst)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    CHECK_EQUAL(0, bus_enumerator_discovered_nodes_count(&en));
}

TEST(BusEnumeratorTestGroup, CanDiscoverOneEntry)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);
    CHECK_EQUAL(1, bus_enumerator_discovered_nodes_count(&en));

    /* Should not change when the same node is updated. */
    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);
    CHECK_EQUAL(1, bus_enumerator_discovered_nodes_count(&en));
}

TEST(BusEnumeratorTestGroup, GetCanId)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    CHECK_EQUAL(BUS_ENUMERATOR_CAN_ID_NOT_SET, bus_enumerator_get_can_id(&en, SMALL_STR_ID));
}

TEST(BusEnumeratorTestGroup, GetCanIdWithSeveralEntries)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, MEDIUM_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);
    bus_enumerator_update_node_info(&en, MEDIUM_STR_ID, MEDIUM_CAN_ID);
    bus_enumerator_update_node_info(&en, LARGE_STR_ID, SMALL_CAN_ID);

    CHECK_EQUAL(LARGE_CAN_ID, bus_enumerator_get_can_id(&en, SMALL_STR_ID));
    CHECK_EQUAL(MEDIUM_CAN_ID, bus_enumerator_get_can_id(&en, MEDIUM_STR_ID));
    CHECK_EQUAL(SMALL_CAN_ID, bus_enumerator_get_can_id(&en, LARGE_STR_ID));
}

TEST(BusEnumeratorTestGroup, GetDriver)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, NULL);
    bus_enumerator_add_node(&en, MEDIUM_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, NULL);

    POINTERS_EQUAL(DRIVER_POINTER, bus_enumerator_get_driver(&en, MEDIUM_STR_ID));
}

TEST(BusEnumeratorTestGroup, GetStringId)
{
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, MEDIUM_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, SMALL_STR_ID, LARGE_CAN_ID);
    bus_enumerator_update_node_info(&en, MEDIUM_STR_ID, MEDIUM_CAN_ID);
    bus_enumerator_update_node_info(&en, LARGE_STR_ID, SMALL_CAN_ID);

    STRCMP_EQUAL(MEDIUM_STR_ID, bus_enumerator_get_str_id(&en, MEDIUM_CAN_ID));
}

TEST_GROUP (BusEnumeratorBufferLengthTestGroup) {
    bus_enumerator_t en;

    static const uint16_t buffer_len = 2;
    bus_enumerator_entry_allocator* buffer = NULL;

    void setup(void)
    {
        // allocate the list of bus enumerators
        buffer = (struct bus_enumerator_entry_allocator*)
            malloc(buffer_len * sizeof(struct bus_enumerator_entry_allocator));
        bus_enumerator_init(&en, buffer, buffer_len);
    }
};

TEST(BusEnumeratorBufferLengthTestGroup, NoBufferOverflow)
{
    bus_enumerator_add_node(&en, SMALL_STR_ID, DRIVER_POINTER);
    bus_enumerator_add_node(&en, MEDIUM_STR_ID, DRIVER_POINTER);
    // this node shouldn't be added
    bus_enumerator_add_node(&en, LARGE_STR_ID, DRIVER_POINTER);

    bus_enumerator_update_node_info(&en, SMALL_STR_ID, SMALL_CAN_ID);
    bus_enumerator_update_node_info(&en, MEDIUM_STR_ID, MEDIUM_CAN_ID);
    // should be NOP
    bus_enumerator_update_node_info(&en, LARGE_STR_ID, SMALL_CAN_ID);

    CHECK_EQUAL(2, bus_enumerator_total_nodes_count(&en));

    STRCMP_EQUAL(SMALL_STR_ID, en.can_to_str[0].str_id);
}
