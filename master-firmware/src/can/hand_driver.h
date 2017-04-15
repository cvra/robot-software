#ifndef HAND_DRIVER_H
#define HAND_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool object_present[4];
    bool object_color[4];
} hand_sensors_t;

int hand_io_init(void);

void hand_set_fingers(const char *hand_id, bool open_0, bool open_1, bool open_2, bool open_3);

#ifdef __cplusplus
}
#endif

#endif /* HAND_DRIVER_H */
