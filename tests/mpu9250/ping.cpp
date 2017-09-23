#include "mpu9250.h"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

TEST_GROUP(MPU9250Ping)
{
    mpu9250_t dev;
    SPIDriver spi;
    uint8_t cmd = 0xf5;
    uint8_t reply;

    void setup()
    {
        mpu9250_init(&dev, &spi);

        mock("spi").expectOneCall("select").withPointerParameter("drv", &spi);
        mock("spi").expectOneCall("send")
            .withPointerParameter("drv", &spi)
            .withMemoryBufferParameter("buf", &cmd, 1);
        mock("spi").expectOneCall("receive")
            .withPointerParameter("drv", &spi)
            .withParameter("n", 1)
            .withOutputParameterReturning("buf", &reply, 1);
        mock("spi").expectOneCall("unselect").withPointerParameter("drv", &spi);
    }
};

TEST(MPU9250Ping, CorrectID)
{
    reply = 0x71;
    CHECK_TRUE(mpu9250_ping(&dev));
}

TEST(MPU9250Ping, WrongId)
{
    reply = 0x00;
    CHECK_FALSE(mpu9250_ping(&dev));
}
