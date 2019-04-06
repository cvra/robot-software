#include <CppUTest/TestHarness.h>

#include "../system.h"

namespace {
struct PerfectSystem : public golem::System<PerfectSystem, int, int> {
    int hidden{};

    int measure_feedback() const
    {
        return hidden;
    }
    void apply_input(const int& in)
    {
        hidden = in;
    }
};
} // namespace

TEST_GROUP (APerfectGolemSystem) {
};

TEST(APerfectGolemSystem, measuredFeedbackMatchesAppliedInput)
{
    PerfectSystem sys;

    sys.apply(42);

    CHECK_EQUAL(sys.measure(), 42);
}
