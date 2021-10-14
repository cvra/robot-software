#include <cstdlib>
#include <webots/Supervisor.hpp>
#include <iostream>
#include <string>
#include <absl/time/clock.h>
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/flags/usage.h>

ABSL_FLAG(std::string, output_dir, "", "Path to where the output should be put.");
ABSL_FLAG(std::string, webot_name, "supervisor", "Name of the supervisor node in webots.");
ABSL_FLAG(absl::Duration, record_duration, absl::Seconds(100), "How long to record the simulation before aborting.");

using namespace webots;

void set_robot(std::string name)
{
    static char env[200];
    sprintf(env, "WEBOTS_ROBOT_NAME=%s", name.c_str());
    putenv(env);
}

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Record Webots output in HTML format");
    absl::ParseCommandLine(argc, argv);

    if (absl::GetFlag(FLAGS_output_dir).empty()) {
        std::cerr << "Must provide -output_dir, aborting." << std::endl;
        return 1;
    }

    set_robot(absl::GetFlag(FLAGS_webot_name));
    Supervisor supervisor;

    supervisor.animationStartRecording(absl::GetFlag(FLAGS_output_dir) + "/animation.html");

    absl::SleepFor(absl::GetFlag(FLAGS_record_duration));

    supervisor.animationStopRecording();

    return 0;
}
