#ifndef COMMANDS_H_
#define COMMANDS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

void shell_spawn(BaseSequentialStream* stream);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H_ */
