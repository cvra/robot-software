#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <vector>
#include <array>
#include <cstdint>

#include "mpu9250.h"

TEST_GROUP (MPU9250Init) {
    mpu9250_t dev;
};

TEST(MPU9250Init, SetSPIDriver)
{
    SPIDriver drv;
    mpu9250_init(&dev, &drv);
    POINTERS_EQUAL(&drv, dev.spi);
}

TEST_GROUP (MPU9250Protocol) {
    mpu9250_t dev;
    SPIDriver drv;

    std::vector<std::array<uint8_t, 2>> writes;
    std::vector<std::array<uint8_t, 2>> reads;

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
        mock("spi").expectOneCall("send").withPointerParameter("drv", &drv).withMemoryBufferParameter("buf", writes.back().data(), 2);
        mock("spi").expectOneCall("unselect").withPointerParameter("drv", &drv);
    }

    void expect_read(uint8_t reg, uint8_t val)
    {
        reg |= 0x80;
        std::array<uint8_t, 2> cmd{{reg, val}};

        reads.push_back(cmd);
        mock("spi").expectOneCall("select").withPointerParameter("drv", &drv);
        mock("spi").expectOneCall("send").withPointerParameter("drv", &drv).withMemoryBufferParameter("buf", &(reads.back().data()[0]), 1);
        mock("spi").expectOneCall("receive").withPointerParameter("drv", &drv).withParameter("n", 1).withOutputParameterReturning("buf", &(reads.back().data()[1]), 1);
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

TEST(MPU9250Protocol, ReadTemperatureData)
{
    int16_t mes = 1000;
    expect_read(65, (mes >> 8));
    expect_read(66, (mes & 0xff));

    float temp = mpu9250_temp_read(&dev);
    DOUBLES_EQUAL(1000 / 333.87 + 21, temp, 0.01);
}

TEST(MPU9250Protocol, CanInitMagnetometer)
{
    // The magnetometer is on the external I2C bus of the MPU9250, therefore we
    // configure it through the MPU.
    // Fortunately it does not require much.
    //
    // We need to write the device address to I2C_SLV0_ADDR, then the register
    // number to I2C_SLV0_REG then the data to I2C_SLV0_DO
    expect_read(106, 3); // check that we are only altering the content of user ctrl
    expect_write(106, (1 << 5) + (1 << 4) + 3); // mst_en and if_dis are set

    expect_write(36, 0x50); // delay data ready interrupt until sensor was read

    expect_write(37, 0x0c); // magnetometer addr.
    expect_write(38, 0x0b); // control register 2
    expect_write(99, 0x01); // reset
    expect_write(39, 0x81); // write 1 byte

    expect_write(37, 0x0c); // magnetometer addr.
    expect_write(38, 0x0a); // control register 1
    expect_write(99, 0x16); // enable continous mode 2 (100 hz), 16 bit
    expect_write(39, 0x81); // write 1 byte

    expect_write(37, 0x0c + (1 << 7)); // magnetometer addr (read bit set)
    expect_write(38, 0x03); // data register

    // read 7 bytes and store them. we read 7 bytes because the magnetometer
    // requires us to read the status byte between each measurement
    expect_write(39, (1 << 7) + 0x7);

    mock("ch").ignoreOtherCalls();

    mpu9250_enable_magnetometer(&dev);
}

TEST(MPU9250Protocol, ReadMagnetometer)
{
    float sensitivity = (4912.0f / 32760);
    for (int i = 0; i < 6; i++) {
        expect_read(73 + i, i);
    }

    float x, y, z;
    mpu9250_mag_read(&dev, &x, &y, &z);

    DOUBLES_EQUAL(0x0001 * sensitivity, x, 0.1);
    DOUBLES_EQUAL(0x0203 * sensitivity, y, 0.1);
    DOUBLES_EQUAL(0x0405 * sensitivity, z, 0.1);
}
