#include <CppUTest/TestHarness.h>
#include <string>
#include <version/version.h>

TEST_GROUP (VersionTestGroup) {
};

TEST(VersionTestGroup, GitCommitIsCorrectLength)
{
    std::string software_version{software_version_str};
    CHECK_TRUE(0 < software_version.length());
}
