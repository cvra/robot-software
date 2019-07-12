/** \file control_system_manager.h
 * \brief Interface for the control_system manager module.
 *
 * This module provide functions to control and regulate a system.
 */

#ifndef _CONTROL_SYSTEM_MANAGER_
#define _CONTROL_SYSTEM_MANAGER_

#include <stdint.h>

/** \addtogroup Regulation
 * Ce module s'occupe de fournir un wrapper aux filtres (PID et trapeze).
 * Il s'occupe aussi de lire les entrees des capteurs et de balancer la
 * puissance sur la sortie du regulateur.
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/** The data structure used by the control_system_manager module */
struct cs {
    int32_t (*consign_filter)(void*, int32_t); /**< Callback function for the consign filter, eg: ramp. */
    void* consign_filter_params; /**< Parameter for consign_filter, will be passed as 1st param. */

    int32_t (*correct_filter)(void*, int32_t); /**< Callback function for the correct filter, eg: PID. */
    void* correct_filter_params; /**< Parameter for correct_filter, will be passed as 1st param. */

    int32_t (*feedback_filter)(void*, int32_t); /**< Callback function for the feedback filter, eg: low pass. */
    void* feedback_filter_params; /**< Parameter for feedback_filter, will be passed as 1st param. */

    /** Callback function for the output filter, eg: torque limiter or inertia adapter.
     * @note This may not be the best way to do it. */
    int32_t (*output_filter)(void*, int32_t);
    void* output_filter_params; /**< Parameter for output_filter, will be passed as 1st param. */

    int32_t (*process_out)(void*); /**< Callback function to get process out, eg : encoder value. */
    void* process_out_params; /**< Parameter for process_out, will be passed as 1st param. */

    void (*process_in)(void*, int32_t); /**< Callback function to set process in, eg: PWM. */
    void* process_in_params; /**< Parameter for process_out, will be passed as 1st param. */

    int32_t consign_value; /**< Consign value for the control system. */
    /** Feedback from the process after going through feedback_filter() */
    int32_t filtered_feedback_value;

    /** Consign for the system after going through consign_filter. This will be feeded to correct_filter. */
    int32_t filtered_consign_value;
    int32_t error_value; /**< Error value. This is filtered_consign_value - filtered_feedback_value. */
    int32_t out_value; /**< Output of correct_filter, as sent to process_in. */
    int enabled; /**< =1 if the control system is enabled, 0 otherwise. */
};

/******* - Prototyping - *******/

/** Initiate the control_system structure by setting all fields to NULL.
 * @param [in] cs A cs structure instance.
 */
void cs_init(struct cs* cs);

/** Set the cs consign_filter fields in the cs structure.
 * @param [in] cs A cs structure instance.
 * @param [in] *consign_filter The consign filter function, eg quadramp_do_filter().
 * @param [in] *consign_filter_params The first parameter of consign_filter.
 */
void cs_set_consign_filter(struct cs* cs,
                           int32_t (*consign_filter)(void*, int32_t),
                           void* consign_filter_params);

/** Set the cs correct_filter fields in the cs structure.
 * @param [in] cs A cs structure instance.
 * @param [in] *correct_filter The correct filter function, eg pid_do_filter().
 * @param [in] *correct_filer_params The first parameter of consign_filter.
 *
 */
void cs_set_correct_filter(struct cs* cs,
                           int32_t (*correct_filter)(void*, int32_t),
                           void* correct_filer_params);

/** Set the cs feedback_filter fields in the cs structure.
 * @param [in] cs A cs structure instance.
 * @param [in] *feedback_filter The feedback filter function.
 * @param [in] *feedback_filer_params The first parameter of feedback_filter.
 */
void cs_set_feedback_filter(struct cs* cs,
                            int32_t (*feedback_filter)(void*, int32_t),
                            void* feedback_filer_params);

/** Set the cs output_filter fields in the cs structure.
 * @param [in] cs A cs structure instance.
 * @param [in] *output_filter The output filter function.
 * @param [in] *output_filter_params The first parameter of output_filter.
 */
void cs_set_output_filter(struct cs* cs,
                          int32_t (*output_filter)(void*, int32_t),
                          void* output_filer_params);

/** Set the cs process_in fields in the cs structure.
 * @param [in] cs A cs structure instance.
 * @param [in] *process_in The process in callback, eg : set_pwm().
 * @param [in] *process_in_params The first argument of process_in.
 */
void cs_set_process_in(struct cs* cs,
                       void (*process_in)(void*, int32_t),
                       void* process_in_params);

/** Set the cs process_out fields in the cs structure.
 * @param [in] cs A cs structure instance.
 * @param [in] *process_out The process out callback, eg : set_pwm().
 * @param [in] *process_out_params The first argument of process_out.
 */
void cs_set_process_out(struct cs* cs,
                        int32_t (*process_out)(void*),
                        void* process_out_params);

/** \brief This function do the main loop of the control system process.
 *
 * - Save the consign in the structure.
 * - Apply the consign filter to the consign.
 * - Read the process out
 * - Apply the feedback filter to the process out
 * - Substract filtered consign to filtered process out.
 * - Save the result in error_value and apply the correct filter.
 * - Save the filtered result and send it to process_in().
 * - Return this result.
 *
 *   @param [in] cs A cs structure instance.
 *   @param [in] consign The consign of the control system.
 *
 *   @returns The value that was put at the process input.
 */
int32_t cs_do_process(struct cs* cs, int32_t consign);

/** Apply cs_do_process() to the structure cs
 *  @param [in] cs A cs structure instance, cast to void *.
 *  @note This is the same as cs_do_process except it takes the consign from
 *  the structure field.
 */
void cs_manage(void* cs);

/** Return the last output sent to process.
 * @param [in] cs A cs structure instance.
 * @returns Last output of the control system, as sent to process in. */
int32_t cs_get_out(struct cs* cs);

/** Return the last calculated error.
 * @param [in] cs A cs structure instance.
 * @returns The last error of the control system.
 */
int32_t cs_get_error(struct cs* cs);

/** Return the current consign.
 * @param [in] cs A cs structure instance.
 * @returns Consign of the control system.
 */
int32_t cs_get_consign(struct cs* cs);

/** Return the current consign, after filter.
 * @param [in] cs A control system instance.
 * @returns The last consign after the filter (eg: once processed by the ramp.)
 */
int32_t cs_get_filtered_consign(struct cs* cs);

/** Return the last feedback value, after filter.
 * @param [in] cs A control system instance.
 * @returns The last feedback of the filter after the feedback filter.
 */
int32_t cs_get_filtered_feedback(struct cs* cs);

/** Gets the feedback value, with no filter
 *
 * @warning This function calls the process_out callback, so be careful if it has
 * side effects.
 *
 * @param [in] cs A control system instance.
 * @returns The unfiltered feeback.
 */
int32_t cs_get_feedback(struct cs* cs);

/** @brief Set system consign.
 *
 * This function change the consign of the control system, but it does not
 * change process input until cs_manage() is called.
 *
 * @param [in] cs A control system instance.
 * @param [in] v The new consign.
 */
void cs_set_consign(struct cs* cs, int32_t v);

/** Disables the control system (it will output zero).
 * @param [in] cs A control system instance.
 * @sa cs_enable().
 */
void cs_disable(struct cs* cs);

/** Enables the control system (return to normal operation).
 * @param [in] cs A control system instance.
 * @sa cs_disable().
 */
void cs_enable(struct cs* cs);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _CONTROL_SYSTEM_MANAGER_ */
