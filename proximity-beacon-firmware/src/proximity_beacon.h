#ifndef PROXIMITY_BEACON_H
#define PROXIMITY_BEACON_H

#ifdef __cplusplus
extern "C" {
#endif

struct proximity_beacon_signal {
    float start_angle;  // [rad]
    float length;       // [rad]
};

void proximity_beacon_init(void);
struct proximity_beacon_signal *proximity_beacon_signal_get(void);
void proximity_beacon_signal_delete(struct proximity_beacon_signal *sp);

#ifdef __cplusplus
}
#endif

#endif /* PROXIMITY_BEACON_H */
