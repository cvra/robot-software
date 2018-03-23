#ifndef GUI_H
#define GUI_H

#include <error/error.h>
#include <stdarg.h>

void gui_start(void);
void gui_log_console(struct error *e, va_list args);

#endif
