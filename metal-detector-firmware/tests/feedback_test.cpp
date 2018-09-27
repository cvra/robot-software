#include "CppUTest/TestHarness.h"
#include "../src/feedback.h"
#include "../src/feedback.c"

#include <limits.h>

TEST_GROUP(FeedbackDeltaAccumulator)
{};

TEST(FeedbackDeltaAccumulator, PositiveDelta)
{
    uint16_t encoder_value = 200;
    uint16_t previous_encoder_value = 100;
    uint16_t p = 5;

    CHECK_EQUAL(100, compute_delta_accumulator_bounded(
                encoder_value,
                previous_encoder_value
            ));

    CHECK_EQUAL(500, compute_delta_accumulator_periodic(
                encoder_value,
                previous_encoder_value,
                p
            ));
}


TEST(FeedbackDeltaAccumulator, NegativeDelta)
{
    uint16_t encoder_value = 100;
    uint16_t previous_encoder_value = 200;
    uint16_t p = 5;

    CHECK_EQUAL(-100, compute_delta_accumulator_bounded(
                encoder_value,
                previous_encoder_value
            ));

    CHECK_EQUAL(-500, compute_delta_accumulator_periodic(
                encoder_value,
                previous_encoder_value,
                p
            ));
}

TEST(FeedbackDeltaAccumulator, Overflow)
{
    uint16_t encoder_value = 10;
    uint16_t previous_encoder_value = UINT16_MAX - 5;
    uint16_t p = 2;

    CHECK_EQUAL(16, compute_delta_accumulator_bounded(
                encoder_value,
                previous_encoder_value
            ));

    CHECK_EQUAL(32, compute_delta_accumulator_periodic(
                encoder_value,
                previous_encoder_value,
                p
            ));
}

TEST(FeedbackDeltaAccumulator, Underflow)
{
    uint16_t encoder_value = UINT16_MAX - 5;
    uint16_t previous_encoder_value = 10;
    uint16_t p = 2;

    CHECK_EQUAL(-16, compute_delta_accumulator_bounded(
                encoder_value,
                previous_encoder_value
            ));

    CHECK_EQUAL(-32, compute_delta_accumulator_periodic(
                encoder_value,
                previous_encoder_value,
                p
            ));
}

TEST(FeedbackDeltaAccumulator, MaxDelta)
{
    uint16_t encoder_value = INT16_MAX;
    uint16_t previous_encoder_value = 0;
    uint16_t p = 2;

    CHECK_EQUAL(INT16_MAX, compute_delta_accumulator_bounded(
                encoder_value,
                previous_encoder_value
            ));

    CHECK_EQUAL(INT16_MAX * 2, compute_delta_accumulator_periodic(
                encoder_value,
                previous_encoder_value,
                p
            ));
}


TEST_GROUP(FeedbackAccumulatorOverflow)
{ };

TEST(FeedbackAccumulatorOverflow, NoOverflow)
{
    int32_t accumulator = 128;
    uint32_t ticks_per_rev = 256;
    uint16_t q = 1;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(128, accumulator);
}

TEST(FeedbackAccumulatorOverflow, FullTurn)
{
    int32_t accumulator = 256;
    uint32_t ticks_per_rev = 256;
    uint16_t q = 1;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(0, accumulator);
}

TEST(FeedbackAccumulatorOverflow, Overflow)
{
    int32_t accumulator = 264;
    uint32_t ticks_per_rev = 200;
    uint16_t q = 1;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(64, accumulator);
}

TEST(FeedbackAccumulatorOverflow, Underflow)
{
    int32_t accumulator = -56;
    uint32_t ticks_per_rev = 256;
    uint16_t q = 1;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(200, accumulator);
}

TEST(FeedbackAccumulatorOverflow, NoOverflowWithQ)
{
    int32_t accumulator = 400;
    uint32_t ticks_per_rev = 100;
    uint16_t q = 5;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(400, accumulator);
}

TEST(FeedbackAccumulatorOverflow, FullTurnWithQ)
{
    int32_t accumulator = 500;
    uint32_t ticks_per_rev = 100;
    uint16_t q = 5;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(0, accumulator);
}

TEST(FeedbackAccumulatorOverflow, OverflowWithQ)
{
    int32_t accumulator = 550;
    uint32_t ticks_per_rev = 100;
    uint16_t q = 5;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(50, accumulator);
}

TEST(FeedbackAccumulatorOverflow, UnderflowWithQ)
{
    int32_t accumulator = -50;
    uint32_t ticks_per_rev = 100;
    uint16_t q = 5;

    periodic_accumulator_overflow(&accumulator, ticks_per_rev, q);

    CHECK_EQUAL(450, accumulator);
}


TEST_GROUP(FeedbackPosition)
{ };

TEST(FeedbackPosition, FullTurnPeriodic)
{
    int32_t accumulator = 200;
    uint32_t ticks_per_rev = 100;
    uint16_t q = 2;

    DOUBLES_EQUAL(6.28318530718,
            compute_encoder_position_periodic(
                accumulator,
                ticks_per_rev,
                q
            ),
            1e-6);
}

TEST(FeedbackPosition, FullTurnBounded)
{
    int32_t accumulator = 100;
    uint32_t ticks_per_rev = 200;
    uint16_t p = 4;
    uint16_t q = 2;

    DOUBLES_EQUAL(6.28318530718,
            compute_encoder_position_bounded(
                accumulator,
                ticks_per_rev,
                p,
                q
            ),
            1e-6);
}


TEST_GROUP(FeedbackVelocity)
{ };

TEST(FeedbackVelocity, HalfTurnPerSecPeriodic)
{
    int32_t delta_accumulator = 200;
    uint32_t ticks_per_rev = 100;
    uint16_t q = 2;
    float delta_t = 2.0f;

    DOUBLES_EQUAL(3.14159265359,
            compute_encoder_velocity_periodic(
                delta_accumulator,
                ticks_per_rev,
                q,
                delta_t
            ),
            1e-6);
}

TEST(FeedbackVelocity, HalfTurnPerSecBounded)
{
    int32_t delta_accumulator = 100;
    uint32_t ticks_per_rev = 200;
    uint16_t p = 4;
    uint16_t q = 2;
    float delta_t = 2.0f;

    DOUBLES_EQUAL(3.14159265359,
            compute_encoder_velocity_bounded(
                delta_accumulator,
                ticks_per_rev,
                p,
                q,
                delta_t
            ),
            1e-6);
}

TEST_GROUP(Feedback)
{
    struct feedback_s feedback;

    void setup(void)
    {
        feedback.primary_encoder.accumulator = 0;
        feedback.primary_encoder.previous = 0;
        feedback.primary_encoder.transmission_p = 3;
        feedback.primary_encoder.transmission_q = 2;
        feedback.primary_encoder.ticks_per_rev = 1024;
    }
};
