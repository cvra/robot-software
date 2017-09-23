#include "mpu9250.h"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

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

