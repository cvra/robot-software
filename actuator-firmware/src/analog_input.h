#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/** Configures the ADC peripherals */
void analog_start(void);

/* Reads the analog inputs, and fills the provided array with the voltage from
 * 0 to 6.6 [V]. */
void analog_input_read(float voltages[2]);

#ifdef __cplusplus
}
#endif
#endif
