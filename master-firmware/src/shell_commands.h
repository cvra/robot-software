#ifndef SHELL_COMMANDS_H
#define SHELL_COMMANDS_H

#include <shell.h>

/* Must define shell_commands final element */
#define SHELL_COMMAND_END() \
__attribute__((section(".shell_commands.1"))) \
ShellCommand __cmd_final = {NULL, NULL}

#define SHELL_COMMAND(NAME, CHP, ARGC, ARGV) \
static void __cmd_##NAME (BaseSequentialStream *, int, char *[]); \
/* shell command list entry */ \
__attribute__((section(".shell_commands.0."#NAME))) \
ShellCommand __cmd_##NAME##_entry = {#NAME, __cmd_##NAME}; \
/* shell command function */ \
static void __cmd_##NAME (BaseSequentialStream *CHP, int ARGC, char *ARGV[])

#ifdef __cplusplus
extern "C" {
#endif

extern ShellCommand __shell_commands_start[];

#ifdef __cplusplus
}
#endif

#endif /* SHELL_COMMANDS_H */