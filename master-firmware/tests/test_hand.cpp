#include "CppUTest/TestHarness.h"
#include <CppUTestExt/MockSupport.h>

#include "hand/hand.h"

void set_voltage(void* m, float value)
{
    *(float*)m = value;
}

TEST_GROUP (AHand) {
    hand_t hand;
    float voltage;

    void setup()
    {
        hand_init(&hand);
        hand_set_pump_callback(&hand, set_voltage, &voltage);
    }
};

TEST(AHand, startsWithDisabledPump){
    CHECK_EQUAL(hand.pump_state, PUMP_OFF)}

TEST(AHand, canTurnPumpOn)
{
    hand_set_pump(&hand, PUMP_ON);

    CHECK_EQUAL(hand.pump_state, PUMP_ON);
    CHECK(voltage > 0);
}

TEST(AHand, canTurnPumpOnReverse)
{
    hand_set_pump(&hand, PUMP_REVERSE);

    CHECK_EQUAL(hand.pump_state, PUMP_REVERSE);
    CHECK(voltage < 0);
}

TEST(AHand, canTurnPumpBackOff)
{
    hand_set_pump(&hand, PUMP_ON);
    hand_set_pump(&hand, PUMP_OFF);

    CHECK_EQUAL(hand.pump_state, PUMP_OFF);
    CHECK_EQUAL(voltage, 0);
}
