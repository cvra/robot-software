#include <CppUTest/TestHarness.h>

#include "../system.h"

namespace {
template <typename T>
struct PerfectSystem {
    T hidden{};

    T measure() const
    {
        return hidden;
    }
    void apply(const T& in)
    {
        hidden = in;
    }
};
} // namespace

TEST_GROUP (APerfectGolemSystem) {
};

TEST(APerfectGolemSystem, measuredFeedbackMatchesAppliedInput)
{
    golem::System<int, int> sys = PerfectSystem<int>();

    sys.apply(42);

    CHECK_EQUAL(sys.measure(), 42);
}
