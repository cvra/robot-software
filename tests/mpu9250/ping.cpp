#include "mpu9250.h"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

TEST_GROUP(MPU9250Ping)
{
    mpu9250_t dev;
};

TEST(MPU9250Ping, CorrectID)
{
    mpu9250_ping(&dev);

}
