#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <vector>
#include <array>
#include <cstdint>
#include "mpu9250.h"

TEST_GROUP(MPU9250Configuration)
{
    mpu9250_t dev;
    SPIDriver drv;

    std::vector<std::array<uint8_t, 2> > commands;

    void setup()
    {
        mpu9250_init(&dev, &drv);
        commands.reserve(100);
    }

    void expect_write(uint8_t reg, uint8_t val)
    {
        std::array<uint8_t, 2> cmd{{reg, val}};

        commands.push_back(cmd);

        mock("spi").expectOneCall("select").withPointerParameter("drv", &drv);
        mock("spi").expectOneCall("send").withPointerParameter("drv", &drv)
            .withMemoryBufferParameter("buf", commands.back().data(), 2);
        mock("spi").expectOneCall("unselect").withPointerParameter("drv", &drv);
    }

};

TEST(MPU9250Configuration, Reset)
{
    expect_write(107, 0x80); // hardware reset
    mpu9250_reset(&dev);
}

TEST(MPU9250Configuration, FixedConfiguration)
{
    expect_write(25, 7); // prescaler
    expect_write(26, 0x00); // general config
    expect_write(27, 0x08); // gyro config
    expect_write(28, (1 << 3)); // accelerometer config
    expect_write(29, 0x00); // accelerometer config 2
    expect_write(35, 0xf8); // FIFO enable for temp, gyro & accel
    expect_write(55, 0x20); // IRQ active high, push pull, latched
    expect_write(56, 0x01); // interrupt on data ready

    // Enable FIFO and reset its content
    expect_write(106, 0x44);

    mpu9250_configure(&dev);
}
