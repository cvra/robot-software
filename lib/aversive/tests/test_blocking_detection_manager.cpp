#include <CppUTest/TestHarness.h>

extern "C" {
#include <aversive/blocking_detection_manager/blocking_detection_manager.h>
}

#define ERROR_THRESHOLD 10
#define ERROR_COUNT_THRESHOLD 1

TEST_GROUP (ABlockingDetectionManager) {
    blocking_detection blocking_detection_manager;

    void setup()
    {
        bd_init(&blocking_detection_manager);
        bd_set_thresholds(&blocking_detection_manager, ERROR_THRESHOLD, ERROR_COUNT_THRESHOLD);
    }
};

TEST(ABlockingDetectionManager, isNotBlockedWhenErrorThresholdIsZero)
{
    const auto errorAboveThreshold = ERROR_THRESHOLD + 1;
    bd_set_thresholds(&blocking_detection_manager, 0, ERROR_COUNT_THRESHOLD);

    bd_manage(&blocking_detection_manager, errorAboveThreshold);

    CHECK_FALSE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isNotBlockedWhenErrorCountThresholdIsZero)
{
    const auto errorAboveThreshold = ERROR_THRESHOLD + 1;
    bd_set_thresholds(&blocking_detection_manager, ERROR_THRESHOLD, 0);

    bd_manage(&blocking_detection_manager, errorAboveThreshold);

    CHECK_FALSE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isNotBlockedWhenErrorIsZero)
{
    const auto nullError = 0;

    bd_manage(&blocking_detection_manager, nullError);

    CHECK_FALSE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isBlockedWhenErrorAboveThreshold)
{
    const auto errorAboveThreshold = ERROR_THRESHOLD + 1;

    bd_manage(&blocking_detection_manager, errorAboveThreshold);

    CHECK_TRUE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isBlockedWhenErrorAboveThresholdForRepeatedNumberOfTimes)
{
    const auto errorAboveThreshold = ERROR_THRESHOLD + 1;
    const auto countThreshold = 42;
    bd_set_thresholds(&blocking_detection_manager, ERROR_THRESHOLD, countThreshold);

    for (int i = 0; i < countThreshold; i++) {
        bd_manage(&blocking_detection_manager, errorAboveThreshold);
    }

    CHECK_TRUE(bd_get(&blocking_detection_manager));
}
