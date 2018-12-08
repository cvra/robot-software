#ifndef TRACE_H
#define TRACE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef TRACE_BUFFER_SIZE
#define TRACE_BUFFER_SIZE 200
#endif

extern const char* event_names[];

enum {
    TRACE_TYPE_STRING,
    TRACE_TYPE_ADDRESS,
    TRACE_TYPE_SCALAR,
    TRACE_TYPE_INTEGER,
};

struct trace_event {
    uint32_t event_id : 8;
    uint32_t type : 2;
    uint32_t timestamp : 22;
    union {
        void* address;
        const char* string;
        int32_t integer;
        float scalar;
    } data;
};

struct trace_buffer_struct {
    bool active;
    size_t write_index;
    size_t nb_events;
    struct trace_event data[TRACE_BUFFER_SIZE];
};

#ifdef __cplusplus
extern "C" {
#endif

void trace(uint8_t event);
void trace_address(uint8_t event, void* p);
void trace_string(uint8_t event, const char* str);
void trace_scalar(uint8_t event_id, float f);
void trace_integer(uint8_t event_id, int32_t i);
void trace_init(void);
void trace_enable(void);
void trace_disable(void);
void trace_clear(void);
void trace_print(void (*print_fn)(void*, const char*, ...), void* arg);

/* Porting functions */
extern int32_t trace_lock(void);
extern void trace_unlock(int32_t status);
extern int32_t trace_timestamp_ms_get(void);

/* Event names */
extern const char* trace_point_names[];

#ifdef __cplusplus
}
#endif

#endif /* TRACE_H */
