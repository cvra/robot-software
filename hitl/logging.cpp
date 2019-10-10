#include <cstdio>
#include <cstdarg>
#include "error/error.h"
#include "absl/flags/flag.h"
#include "absl/synchronization/mutex.h"

ABSL_FLAG(bool, verbose, false, "Enable verbose logging.");
static ABSL_CONST_INIT absl::Mutex logging_lock(absl::kConstInit);

static void log_message(struct error* e, ...)
{
    absl::MutexLock l(&logging_lock);
    va_list va;
    va_start(va, e);
    printf("%s ", error_severity_get_name(e->severity));
    vprintf(e->text, va);
    printf("\n");
    va_end(va);

    if (e->severity == ERROR_SEVERITY_ERROR) {
        exit(1);
    }
}

void logging_init()
{
    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    if (absl::GetFlag(FLAGS_verbose)) {
        error_register_debug(log_message);
    }
}
