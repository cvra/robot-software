#include "CppUTest/TestHarness.h"
#include "../iir.h"

TEST_GROUP(IIRFilter)
{
    filter_iir_t iir_filter;
    void setup(void)
    {
        float buffer[3];
        const float b[4] = {7.f, 6.f, 5.f, 4.f};
        const float a[3] = {3.f, 2.f, 1.f};

        filter_iir_init(&iir_filter, b, a, 3, buffer);
    }
};

TEST(IIRFilter, IIRFilterInit)
{
    int i;
    for (i = 0; i < iir_filter.n; i++) {
        DOUBLES_EQUAL(0, iir_filter.d[i], 1.0e-9);
    }
}
