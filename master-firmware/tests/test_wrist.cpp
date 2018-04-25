#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <hand/wrist.h>

namespace
{
void set_servo_pos(void* s, float value)
{
    *(float *)s = value;
}
}

TEST_GROUP(AWrist)
{
    wrist_t wrist;
    float servo_pos = 0.f;

    void setup()
    {
        wrist_init(&wrist);
        wrist_set_servo_callback(&wrist, &set_servo_pos, &servo_pos);
        wrist_set_servo_range(&wrist, -1.0, 1.0);
    }
};

TEST(AWrist, initializesInHorizontalPosition)
{
    CHECK_EQUAL(WRIST_HORIZONTAL, wrist.state);
}

TEST(AWrist, movesToHorizontalPosition)
{
    wrist_set_horizontal(&wrist);

    CHECK_EQUAL(WRIST_HORIZONTAL, wrist.state);
    DOUBLES_EQUAL(-1.f, servo_pos, 1e-7);
}

TEST(AWrist, movesToVerticalPosition)
{
    wrist_set_vertical(&wrist);

    CHECK_EQUAL(WRIST_VERTICAL, wrist.state);
    DOUBLES_EQUAL(1.f, servo_pos, 1e-7);
}
