#include "feedback.h"


void feedback_compute(struct feedback_s *feedback)
{
    switch (feedback->input_selection) {
        case FEEDBACK_POTENTIOMETER : {
            float position =
                feedback->potentiometer.gain * feedback->input.potentiometer
                - feedback->potentiometer.zero;

            feedback->output.velocity =
                (position - feedback->output.position)
                / feedback->input.delta_t;

            feedback->output.position = position;
            break;
        }
    }
}
