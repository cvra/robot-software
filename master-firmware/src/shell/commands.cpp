#include "commands.h"

void command_reboot(std::ostream& stream, int /* argc */, const char** /*argv*/)
{
    stream << "Exiting now, the restart will be handled by systemd" << std::endl;
    exit(1);
}

absl::flat_hash_map<std::string, TerminalHandler> shell_commands{
    {"reboot", command_reboot},
};
