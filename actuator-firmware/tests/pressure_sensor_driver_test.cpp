#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../src/pressure_sensor.h"

void mock_select(void* ptr)
{
    mock("mpr").actualCall("select").withPointerParameter("ptr", ptr);
}

void mock_unselect(void* ptr)
{
    mock("mpr").actualCall("unselect").withPointerParameter("ptr", ptr);
}

void mock_transmit(void* arg, const uint8_t* tx, uint8_t* rx, size_t n)
{
    mock("mpr")
        .actualCall("transmit")
        .withPointerParameter("ptr", arg)
        .withMemoryBufferParameter("tx", tx, n)
        .withOutputParameter("rx", rx);
}

TEST_GROUP (PressureSensorDriverTestGroup) {
    mpr_driver_t drv;
    void setup()
    {
        drv.arg = (void*)0x1234;
        drv.select = mock_select;
        drv.unselect = mock_unselect;
        drv.transmit = mock_transmit;
    }
};

TEST(PressureSensorDriverTestGroup, StartMeasurementSendsTheRightCommand)
{
    uint8_t expected_cmd[] = {0xaa, 0x00, 0x00};
    uint8_t result[] = {0, 0, 0}; // does not matter
    mock("mpr").expectOneCall("select").withPointerParameter("ptr", drv.arg);
    mock("mpr")
        .expectOneCall("transmit")
        .withPointerParameter("ptr", drv.arg)
        .withMemoryBufferParameter("tx", expected_cmd, sizeof(expected_cmd))
        .withOutputParameterReturning("rx", result, sizeof(result));
    mock("mpr").expectOneCall("unselect").withPointerParameter("ptr", drv.arg);

    mpr_start_measurement(&drv);
}

TEST(PressureSensorDriverTestGroup, ReadsBusyFlag)
{
    uint8_t expected_cmd[] = {0xf0};

    // busy flag and poweron flag set
    uint8_t result[] = {0x60};

    mock("mpr").expectOneCall("select").withPointerParameter("ptr", drv.arg);
    mock("mpr")
        .expectOneCall("transmit")
        .withPointerParameter("ptr", drv.arg)
        .withMemoryBufferParameter("tx", expected_cmd, sizeof(expected_cmd))
        .withOutputParameterReturning("rx", result, sizeof(result));
    mock("mpr").expectOneCall("unselect").withPointerParameter("ptr", drv.arg);

    auto status = mpr_read_status(&drv);
    CHECK_EQUAL(0x60, status);
}

TEST(PressureSensorDriverTestGroup, ReadConversionCommand)
{
    uint8_t expected_cmd[] = {0xf0, 0x00, 0x00, 0x00};
    uint8_t result[] = {0x00, 0x12, 0x34, 0x56};

    mock("mpr").expectOneCall("select").withPointerParameter("ptr", drv.arg);
    mock("mpr")
        .expectOneCall("transmit")
        .withPointerParameter("ptr", drv.arg)
        .withMemoryBufferParameter("tx", expected_cmd, sizeof(expected_cmd))
        .withOutputParameterReturning("rx", result, sizeof(result));
    mock("mpr").expectOneCall("unselect").withPointerParameter("ptr", drv.arg);

    auto data = mpr_read_data(&drv);
    CHECK_EQUAL(0x123456, data);
}

TEST(PressureSensorDriverTestGroup, PoweredOffDeviceIsAnError)
{
    CHECK_TRUE(mpr_status_is_error(0x00));
}

TEST(PressureSensorDriverTestGroup, BusyDeviceIsNotAnError)
{
    /* 0x60 means no error, busy */
    CHECK_FALSE(mpr_status_is_error(0x60));
}

TEST(PressureSensorDriverTestGroup, MemoryErrorIsAnError)
{
    /* 0x64 means memory error, busy */
    CHECK_TRUE(mpr_status_is_error(0x64));
}

TEST(PressureSensorDriverTestGroup, MathSaturationIsAnError)
{
    /* 0x61 means saturation error, busy */
    CHECK_TRUE(mpr_status_is_error(0x61));
}

TEST(PressureSensorDriverTestGroup, CheckIfBusy)
{
    CHECK_TRUE(mpr_status_is_busy(0x60));
    CHECK_FALSE(mpr_status_is_busy(0x40));
}

TEST_GROUP (ConversionTestGroup) {
};

TEST(ConversionTestGroup, ConvertMinPressure)
{
    uint32_t original = 0x19999A;
    float pa = mpr_pressure_raw_to_pascal(original);
    DOUBLES_EQUAL(0., pa, 0.001);
}

TEST(ConversionTestGroup, ConvertMaxPressure)
{
    uint32_t original = 0xe66666;
    float pa = mpr_pressure_raw_to_pascal(original);
    float pmax_in_pascal = 103421;
    DOUBLES_EQUAL(pmax_in_pascal, pa, 0.001);
}
