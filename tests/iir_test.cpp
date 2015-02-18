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
    DOUBLES_EQUAL(b[0] * x, y1, 1.0e-7);
    y2 = filter_iir_apply(&iir_filter, x);
    DOUBLES_EQUAL(b[0] * x + b[1] * x - a[0] * y1, y2, 1.0e-7);
    y3 = filter_iir_apply(&iir_filter, x);
    DOUBLES_EQUAL(b[0]*x + b[1]*x + b[2]*x - a[0]*y2 - a[1]*y1, y3, 1.0e-7);

}

TEST(IIRFilter, StepResponse)
{
    // in matlab: y=step(filt([0.1, 0.2, 0.3, 0.4], [1, 0.1, 0.1, 0.1]), [0:20])
    DOUBLES_EQUAL(0.100000000000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.290000000000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.561000000000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.904900000000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.824410000000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.770969000000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.749972100000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.765464890000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.771359401000000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.771320360900000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769185534810000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.768813470329000, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769068063396100, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769293293146490, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769282517312841, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769235612614457, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769218857692621, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769226301238008, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769231922845491, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769232291822388, filter_iir_apply(&iir_filter, 1), 1.0e-7);
    DOUBLES_EQUAL(0.769230948409411, filter_iir_apply(&iir_filter, 1), 1.0e-7);
}
