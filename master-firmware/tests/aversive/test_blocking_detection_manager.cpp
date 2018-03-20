#include <CppUTest/TestHarness.h>

extern "C" {
#include "control_system_manager/control_system_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
}

#define ERROR_THRESHOLD         10
#define ERROR_COUNT_THRESHOLD   1

TEST_GROUP(ABlockingDetectionManager)
{
    blocking_detection blocking_detection_manager;
    cs control_system;

    void setup()
    {
        cs_init(&control_system);
        bd_init(&blocking_detection_manager, &control_system);
        bd_set_thresholds(&blocking_detection_manager, ERROR_THRESHOLD, ERROR_COUNT_THRESHOLD);
    }
};

TEST(ABlockingDetectionManager, isNotBlockedWhenErrorThresholdIsZero)
{
    control_system.error_value = ERROR_THRESHOLD + 1;
    bd_set_thresholds(&blocking_detection_manager, 0, ERROR_COUNT_THRESHOLD);

    bd_manage(&blocking_detection_manager);

    CHECK_FALSE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isNotBlockedWhenErrorCountThresholdIsZero)
{
    control_system.error_value = ERROR_THRESHOLD + 1;
    bd_set_thresholds(&blocking_detection_manager, ERROR_THRESHOLD, 0);

    bd_manage(&blocking_detection_manager);

    CHECK_FALSE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isNotBlockedWhenErrorIsZero)
{
    control_system.error_value = 0;

    bd_manage(&blocking_detection_manager);

    CHECK_FALSE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isBlockedWhenErrorAboveThreshold)
{
    control_system.error_value = ERROR_THRESHOLD + 1;

    bd_manage(&blocking_detection_manager);

    CHECK_TRUE(bd_get(&blocking_detection_manager));
}

TEST(ABlockingDetectionManager, isBlockedWhenErrorAboveThresholdForRepeatedNumberOfTimes)
{
    control_system.error_value = ERROR_THRESHOLD + 1;
    const auto countThreshold = 42;
    bd_set_thresholds(&blocking_detection_manager, ERROR_THRESHOLD, countThreshold);

    for (int i = 0; i < countThreshold; i++) {
        bd_manage(&blocking_detection_manager);
    }

    CHECK_TRUE(bd_get(&blocking_detection_manager));
}
