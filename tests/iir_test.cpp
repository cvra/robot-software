#include "CppUTest/TestHarness.h"
#include "../iir.h"

TEST_GROUP(IIRFilter)
{
    filter_iir_t iir_filter;
    const float b[4] = {0.1, 0.2, 0.3, 0.4};
    const float a[3] = {0.1, 0.1, 0.1};
    float buffer[3];

    void setup(void)
    {
        filter_iir_init(&iir_filter, b, a, 3, buffer);
    }
};

TEST(IIRFilter, IIRFilterInit)
{
    int i;
    for (i = 0; i < iir_filter.n; i++) {
        DOUBLES_EQUAL(0, iir_filter.d[i], 1.0e-9);
    }

    for (i = 0; i < iir_filter.n; i++) {
        DOUBLES_EQUAL(a[i], iir_filter.a[i], 1.0e-9);
    }

    for (i = 0; i < iir_filter.n + 1; i++) {
        DOUBLES_EQUAL(b[i], iir_filter.b[i], 1.0e-9);
    }
}

TEST(IIRFilter, ApplyStep)
{
    float x = 1.f;
    float y1, y2, y3;

    y1 = filter_iir_apply(&iir_filter, x);
    DOUBLES_EQUAL(b[0] * x, y1, 1.0e-9);
    y2 = filter_iir_apply(&iir_filter, x);
    DOUBLES_EQUAL(b[0] * x + b[1] * x - a[0] * y1, y2, 1.0e-9);
    y3 = filter_iir_apply(&iir_filter, x);
    DOUBLES_EQUAL(b[0]*x + b[1]*x + b[2]*x - a[0]*y2 - a[1]*y1, y3, 1.0e-9);

}

TEST(IIRFilter, StepResponse)
{
    // in matlab: y=step(filt([0.1, 0.2, 0.3, 0.4], [1, 0.1, 0.1, 0.1]), [0:20])
    DOUBLES_EQUAL(0.1000, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.2900, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.5610, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.9049, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.8244, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7710, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7500, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7655, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7714, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7713, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7688, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7691, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7693, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7693, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
    DOUBLES_EQUAL(0.7692, filter_iir_apply(&iir_filter, 1), 1.0e-4);
}
