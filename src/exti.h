#ifndef EXTI_H
#define EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

#define EXTI_EVENT_IMU_INT     (1 << 0)
#define EXTI_EVENT_UWB_INT     (1 << 1)

extern event_source_t exti_events;

/** Starts the external interrupt processing service. */
void exti_start(void);


#ifdef __cplusplus
}
#endif



#endif /* EXTI_H */
