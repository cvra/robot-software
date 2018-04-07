#ifndef SHELLCONF_H
#define SHELLCONF_H

/* Shell related config */
#define SHELL_MAX_ARGUMENTS             5
#define SHELL_MAX_LINE_LENGTH           100
#define SHELL_USE_HISTORY               TRUE
#define SHELL_USE_ESC_SEQ               TRUE
#define SHELL_MAX_HIST_BUFF             4*SHELL_MAX_LINE_LENGTH
#define SHELL_PROMPT_STR                "uwb-beacon> "
#define SHELL_USE_COMPLETION            TRUE

#define SHELL_CMD_THREADS_ENABLED       TRUE
#define SHELL_CMD_TEST_ENABLED          FALSE

#endif /* SHELLCONF_H */
