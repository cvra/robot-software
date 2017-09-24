#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <vector>
#include <array>
#include <cstdint>

#include "mpu9250.h"

TEST_GROUP(MPU9250Init)
{
    mpu9250_t dev;
};

TEST(MPU9250Init, SetSPIDriver)
{
    SPIDriver drv;
    mpu9250_init(&dev, &drv);
    POINTERS_EQUAL(&drv, dev.spi);
}

TEST_GROUP(MPU9250Protocol)
{
    mpu9250_t dev;
    SPIDriver drv;

    std::vector<std::array<uint8_t, 2> > writes;
    std::vector<std::array<uint8_t, 2> > reads;

    void setup()
    {
        mpu9250_init(&dev, &drv);
        writes.reserve(100);
        reads.reserve(100);
    }

    void expect_write(uint8_t reg, uint8_t val)
    {
        std::array<uint8_t, 2> cmd{{reg, val}};

        writes.push_back(cmd);

        mock("spi").expectOneCall("select").withPointerParameter("drv", &drv);
        mock("spi").expectOneCall("send").withPointerParameter("drv", &drv)
            .withMemoryBufferParameter("buf", writes.back().data(), 2);
        mock("spi").expectOneCall("unselect").withPointerParameter("drv", &drv);
    }

    void expect_read(uint8_t reg, uint8_t val)
    {
        reg |= 0x80;
        std::array<uint8_t, 2> cmd{{reg, val}};

        reads.push_back(cmd);
        mock("spi").expectOneCall("select").withPointerParameter("drv", &drv);
        mock("spi").expectOneCall("send").withPointerParameter("drv", &drv)
            .withMemoryBufferParameter("buf", &(reads.back().data()[0]), 1);
        mock("spi").expectOneCall("receive")
            .withPointerParameter("drv", &drv)
            .withParameter("n", 1)
            .withOutputParameterReturning("buf", &(reads.back().data()[1]), 1);
        mock("spi").expectOneCall("unselect").withPointerParameter("drv", &drv);
    }


};

TEST(MPU9250Protocol, Reset)
{
    expect_write(107, 0x80); // hardware reset
    mpu9250_reset(&dev);
}

TEST(MPU9250Protocol, FixedConfiguration)
{
    expect_write(25, 3); // prescaler
    expect_write(26, 0x01); // general config
    expect_write(27, 0x08); // gyro config
    expect_write(28, (1 << 3)); // accelerometer config
    expect_write(29, 0x00); // accelerometer config 2
    expect_write(55, 0x20); // IRQ active high, push pull, latched
    expect_write(56, 0x01); // interrupt on data ready

    // Enable FIFO and reset its content
    expect_write(106, 0x44);

    mpu9250_configure(&dev);
}

TEST(MPU9250Protocol, CorrectID)
{
    // Correct ID in the who am i register
    expect_read(117, 0x71);
    CHECK_TRUE(mpu9250_ping(&dev));
}

TEST(MPU9250Protocol, WrongId)
{
    // invalid id
    expect_read(117, 42);
    CHECK_FALSE(mpu9250_ping(&dev));
}

TEST(MPU9250Protocol, ReadInterrupts)
{
    // Reads the interrupt status register
    expect_read(58, 0x42);
    auto irqs = mpu9250_interrupt_read_and_clear(&dev);
    CHECK_EQUAL(0x42, irqs);
}

TEST(MPU9250Protocol, ReadGyroData)
{
    const float gain = 1 / 65.5 * 3.14 / 180;

    int16_t x = 1000, y = 500, z = -200;
    expect_read(67, (x >> 8));
    expect_read(68, (x & 0xff));
    expect_read(69, (y >> 8));
    expect_read(70, (y & 0xff));
    expect_read(71, (z >> 8));
    expect_read(72, (z & 0xff));

    float mes_x, mes_y, mes_z;
    mpu9250_gyro_read(&dev, &mes_x, &mes_y, &mes_z);

    DOUBLES_EQUAL(x * gain, mes_x, 0.01);
    DOUBLES_EQUAL(y * gain, mes_y, 0.01);
    DOUBLES_EQUAL(z * gain, mes_z, 0.01);
}

TEST(MPU9250Protocol, ReadAccData)
{
    const float gain = 9.81 / 8192;

    int16_t x = 1000, y = 500, z = -200;

    expect_read(59, (x >> 8));
    expect_read(60, (x & 0xff));
    expect_read(61, (y >> 8));
    expect_read(62, (y & 0xff));
    expect_read(63, (z >> 8));
    expect_read(64, (z & 0xff));

    float mes_x, mes_y, mes_z;
    mpu9250_acc_read(&dev, &mes_x, &mes_y, &mes_z);

    DOUBLES_EQUAL(x * gain, mes_x, 0.01);
    DOUBLES_EQUAL(y * gain, mes_y, 0.01);
    DOUBLES_EQUAL(z * gain, mes_z, 0.01);
}
