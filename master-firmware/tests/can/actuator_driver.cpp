#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "can/actuator_driver.h"
#include "can/bus_enumerator.h"
#include <parameter/parameter.h>
#include <memory>
#include <vector>

TEST_GROUP (ActuatorDriverTestGroup) {
    actuator_driver_t drv;
    bus_enumerator_t be;
    parameter_namespace_t ns;

    std::array<bus_enumerator_entry_allocator, 1> be_allocator;
    const char* name = "actuator-front-left";

    void setup() override
    {
        parameter_namespace_declare(&ns, nullptr, nullptr);
        bus_enumerator_init(&be, be_allocator.data(), be_allocator.size());

        actuator_driver_init(&drv, &be, &ns, name);
    }
};

TEST(ActuatorDriverTestGroup, CanRegisterCANName)
{
    STRCMP_EQUAL(name, drv.can_name);

    /* checks that this is known to the bus enumerator */
    auto p = bus_enumerator_get_driver(&be, name);
    POINTERS_EQUAL(&drv, p);
}

TEST(ActuatorDriverTestGroup, RegisterCreatesParameters)
{
    auto p = parameter_namespace_find(&ns, name);
    CHECK_TRUE(p != nullptr);

    std::vector<std::string> expected_params = {"high_position", "table_pickup_position", "reef_pickup_position", "pump_pwm"};

    for (const auto param_name : expected_params) {
        auto* param = parameter_find(p, param_name.c_str());
        CHECK_TRUE_TEXT(param != nullptr, param_name.c_str());
        CHECK_TRUE_TEXT(parameter_defined(param), param_name.c_str());
    }
}

TEST(ActuatorDriverTestGroup, IsInHighPositionByDefault)
{
    CHECK_EQUAL(POSITION_HIGH, drv.actuator_position);
}

TEST(ActuatorDriverTestGroup, CanSetArmPosition)
{
    actuator_driver_set_position(&drv, POSITION_REEF_PICKUP);
    CHECK_EQUAL(POSITION_REEF_PICKUP, drv.actuator_position);
}

TEST(ActuatorDriverTestGroup, CanTurnPumpOn)
{
    CHECK_FALSE(drv.pump_enabled[PUMP_TOP]);
    actuator_driver_pump_set(&drv, PUMP_TOP, true);
    CHECK_TRUE(drv.pump_enabled[PUMP_TOP]);
}

TEST(ActuatorDriverTestGroup, CanPrepareMessageForUAVCAN)
{
    int node_id;
    float pump[2];
    bool solenoid[2];
    float position;

    auto prepare_msg = [&]() {
        return actuator_driver_prepare_uavcan_msg(&drv, &be, &node_id, &pump[0], &pump[1], &solenoid[0], &solenoid[1], &position);
    };

    {
        auto ready = prepare_msg();
        CHECK_FALSE_TEXT(ready, "should not return ready until bus_enumerator knows this board.");
    }

    // Now the board is discovered
    bus_enumerator_update_node_info(&be, name, 42);

    {
        actuator_driver_pump_set(&drv, PUMP_TOP, true);

        auto ready = prepare_msg();
        CHECK_TRUE_TEXT(ready, "returns true once bus_enumerators knows about it");

        DOUBLES_EQUAL(0.3, pump[0], 0.01);
        DOUBLES_EQUAL(0.0, pump[1], 0.01);

        CHECK_TRUE(solenoid[0]);
        CHECK_FALSE(solenoid[1]);
        CHECK_EQUAL(parameter_scalar_get(&drv.high_position), position);
    }

    {
        actuator_driver_set_position(&drv, POSITION_REEF_PICKUP);
        prepare_msg();
        CHECK_EQUAL(parameter_scalar_get(&drv.reef_pickup_position), position);
    }

    {
        actuator_driver_set_position(&drv, POSITION_TABLE_PICKUP);
        prepare_msg();
        CHECK_EQUAL(parameter_scalar_get(&drv.table_pickup_position), position);
    }
}
