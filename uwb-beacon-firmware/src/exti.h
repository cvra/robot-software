#ifndef EXTI_H
#define EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

extern event_source_t imu_event;
extern event_source_t uwb_event;

/** Starts the external interrupt processing service. */
void exti_start(void);

#ifdef __cplusplus
}
#endif

#endif /* EXTI_H */
