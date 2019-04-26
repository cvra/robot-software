#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

void debug_msg(const char* fmt, ...);
void debug_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H */
