#include "feedback.h"


void feedback_compute(struct feedback_s *feedback)
{
    switch (feedback->input_selection) {
        case FEEDBACK_RPM :
            /* Call other module that updates the period on each interrupt
             * (light barrier crossing) and integrates the position assuming
             * constant velocity.
             * The period is assumed being constant as long as the time since
             * the last interrupt is less than the previous period.
             * When more time than the previous period has passed (i.e. the
             * actuator is decelerating), the speed is updated to the _maximal_
             * possible speed (1 / [time since last interrupt]) and the
             * position stops moving (SBB clock style).
             */
            break;
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
