#include "CppUTest/TestHarness.h"
#include "../iir.h"

TEST_GROUP(IIRFilter)
{
    filter_iir_t iir_filter;
    float buffer[3];
    const float b[4] = {7.f, 6.f, 5.f, 4.f};
    const float a[3] = {3.f, 2.f, 1.f};

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
