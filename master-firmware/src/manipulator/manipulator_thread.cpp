#include <ch.h>
#include <hal.h>
#include <numeric>

#include <algorithm>

#include <error/error.h>
#include <golem/golem.h>
#include <parameter/parameter.h>

#include "priorities.h"
#include "main.h"
#include "config.h"

#include "manipulator/manipulator.h"
#include "manipulator/manipulator_thread.h"
#include "dijkstra.hpp"

#include "manipulator/controller.h"
#include "manipulator/hw.h"
#include "manipulator/state_estimator.h"

#include "usbconf.h"
#include "motor_manager.h"
#include "robot_helpers/math_helpers.h"
#include "control/Arm2DCtrl.hpp"
#include "control/LinearSystem.hpp"
#include "control/LQRController.hpp"
#include "control/RefTracking.hpp"
#include "manipulator/kinematics.h"
#include "protobuf/manipulator.pb.h"

#define MANIPULATOR_FREQUENCY 50
#define MANIPULATOR_TRAJECTORY_FREQUENCY 50

#define MANIPULATOR_THREAD_STACKSIZE 16384
#define MANIPULATOR_TRAJECTORY_THREAD_STACKSIZE 1024

#define TRANSMISSION_INERTIA 0.4
#define GRAVITY_ACCEL 9.81

#define PID 0
#define LQR 1

using manipulator::Angles;
using manipulator::ArmLengths;
using manipulator::Pose2D;

namespace {
  
motor_driver_t* get_motor_driver(const char* name)
{
    motor_driver_t* motor = (motor_driver_t*)bus_enumerator_get_driver(motor_manager.bus_enumerator, name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", name);
    }
    return motor;
}

float constrainAngle(float x)
{
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

float read_motor_position(const char* name)
{
    return motor_driver_get_and_clear_stream_value(get_motor_driver(name), MOTOR_STREAM_POSITION);
}

float read_motor_velocity(const char* name)
{
    return motor_driver_get_and_clear_stream_value(get_motor_driver(name), MOTOR_STREAM_VELOCITY);
}

void init_LQR(LQRController<6, 3, 10> &lqr,
              parameter_namespace_t* control_params)
{
    static LinearSystem<6, 3> system;
    float delta = 1e-9f; // small delta to avoid singular matrix
    float torque_cst[2] = {2.9f,2.0f}; // motor: 2232 U009 SR and 1524 U012 SR

    system.A << delta,  1.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    delta,  0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    delta,  1.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    delta,  0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    delta,  1.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    delta;

    system.B << 0.0,           0.0,           0.0,
                torque_cst[0], 0.0,           0.0,
                0.0,           0.0,           0.0,
                0.0,           torque_cst[0], 0.0,
                0.0,           0.0,           0.0,
                0.0,           0.0,           torque_cst[1];

    lqr.sampling_period = 1.0 / MANIPULATOR_FREQUENCY;
    lqr.system = discretize(system, lqr.sampling_period);

    lqr.Q << parameter_scalar_get(parameter_find(control_params, "Q_1")), 0, 0, 0, 0, 0,
             0, parameter_scalar_get(parameter_find(control_params, "Q_2")), 0, 0, 0, 0,
             0, 0, parameter_scalar_get(parameter_find(control_params, "Q_3")), 0, 0, 0,
             0, 0, 0, parameter_scalar_get(parameter_find(control_params, "Q_4")), 0, 0,
             0, 0, 0, 0, parameter_scalar_get(parameter_find(control_params, "Q_5")), 0,
             0, 0, 0, 0, 0, parameter_scalar_get(parameter_find(control_params, "Q_6"));

    lqr.R << parameter_scalar_get(parameter_find(control_params, "R_1")), 0, 0,
             0, parameter_scalar_get(parameter_find(control_params, "R_2")), 0,
             0, 0, parameter_scalar_get(parameter_find(control_params, "R_3"));

    lqr.N = system.B;
}
} // namespace

// RAII to manage locks on the manipulator
class ManipulatorLockGuard {
private:
    mutex_t* lock;

public:
    ManipulatorLockGuard(void* l)
        : lock((mutex_t*)l)
    {
        chMtxLock(lock);
    }
    ~ManipulatorLockGuard()
    {
        chMtxUnlock(lock);
    }
};

MUTEX_DECL(right_lock);
manipulator::Manipulator<ManipulatorLockGuard> right_arm{{0, 0, 0}, &right_lock};

void manipulator_angles(float* angles)
{
    auto measured = right_arm.angles();
    std::copy_n(std::begin(measured), 3, angles);
}

void manipulator_angles_set(float q1, float q2, float q3)
{
    Angles input = {q1, q2, q3};
    right_arm.apply(input);
}

void manipulator_angles_wait_for_traj_end(uint16_t timeout_ms)
{
    uint16_t time_since_start_ms = 0;
    const uint16_t poll_time_step_ms = 10;

    while (!right_arm.reached_target() && time_since_start_ms < timeout_ms) {
        chThdSleepMilliseconds(poll_time_step_ms);
        time_since_start_ms += poll_time_step_ms;
    }
}

void manipulator_angles_goto_timeout(float q1, float q2, float q3, uint16_t timeout_ms)
{
    manipulator_angles_set(q1, q2, q3);
    manipulator_angles_wait_for_traj_end(timeout_ms);
}

bool manipulator_goto(manipulator_state_t target)
{
    int n = pathfinding::dijkstra(right_arm.nodes, MANIPULATOR_COUNT,
                                  right_arm.nodes[right_arm.state],
                                  right_arm.nodes[target]);

    pathfinding::Node<manipulator::Point>* node = &right_arm.nodes[right_arm.state];

    for (int i = 0; i < n; i++) {
        node = node->path_next;
        manipulator_angles_set(node->data.angles[0], node->data.angles[1], node->data.angles[2]);
        manipulator_angles_wait_for_traj_end(MANIPULATOR_DEFAULT_TIMEOUT_MS);
    }

    right_arm.state = target;
    return true;
}

void manipulator_gripper_set(gripper_state_t state)
{
    if (state == GRIPPER_ACQUIRE) {
        right_arm.gripper.acquire();
    } else if (state == GRIPPER_RELEASE) {
        right_arm.gripper.release();
    } else {
        right_arm.gripper.disable();
    }
}

static THD_FUNCTION(manipulator_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t* right_arm_params = parameter_namespace_find(&master_config, "arms/right");
    parameter_namespace_t* control_params = parameter_namespace_find(&master_config, "main_arm/control");
    parameter_namespace_t* l_params = parameter_namespace_find(&master_config, "main_arm/length");
    parameter_namespace_t* ref1_params = parameter_namespace_find(&master_config, "main_arm/ref1");
    parameter_namespace_t* ref2_params = parameter_namespace_find(&master_config, "main_arm/ref2");
    parameter_namespace_t* ref3_params = parameter_namespace_find(&master_config, "main_arm/ref3");
    parameter_namespace_t* mass_params = parameter_namespace_find(&master_config, "main_arm/mass");
    parameter_namespace_t* puck_params = parameter_namespace_find(&master_config, "main_arm/puck");

    /* Setup and advertise manipulator state topic */
    static TOPIC_DECL(manipulator_topic, Manipulator);
    messagebus_advertise_topic(&bus, &manipulator_topic.topic, "/manipulator");
    
    /* PID init */
    Manipulator manipulator_state = Manipulator_init_zero;

    right_arm.set_lengths({
        config_get_scalar("master/arms/right/lengths/l1"),
        config_get_scalar("master/arms/right/lengths/l2"),
        config_get_scalar("master/arms/right/lengths/l3"),
    });
    right_arm.set_offsets({
        config_get_scalar("master/arms/right/offsets/q1"),
        config_get_scalar("master/arms/right/offsets/q2"),
        config_get_scalar("master/arms/right/offsets/q3"),
    });
    right_arm.set_tolerance({
        config_get_scalar("master/arms/right/tolerance/q1"),
        config_get_scalar("master/arms/right/tolerance/q2"),
        config_get_scalar("master/arms/right/tolerance/q3"),
    });
    
    /* LQR init */
    
    const float link_lengths[3] = {
        parameter_scalar_get(parameter_find(l_params, "segment_1")),
        parameter_scalar_get(parameter_find(l_params, "segment_2")),
        parameter_scalar_get(parameter_find(l_params, "segment_3"))
    };

    const float mass[3] = {
        parameter_scalar_get(parameter_find(mass_params, "segment_1")),
        parameter_scalar_get(parameter_find(mass_params, "segment_2")),
        parameter_scalar_get(parameter_find(mass_params, "segment_3"))
    };

    static LQRController<6, 3, 10> lqr;
    init_LQR(lqr,control_params);

    static std::array<Eigen::Matrix<float, 3, 6>, 10> gain = lqr.gain();

    Eigen::Matrix<float, 6, 1> error;
    float init_traj_angle[3];
    float max_accel = 13.f;

    init_traj_angle[0] = read_motor_position("theta-1");
    init_traj_angle[1] = read_motor_position("theta-2");
    init_traj_angle[2] = read_motor_position("theta-3");

    Eigen::Matrix<float, 2, 1> r1{init_traj_angle[0],0.f};
    Eigen::Matrix<float, 2, 1> r2{init_traj_angle[1],0.f};
    Eigen::Matrix<float, 2, 1> r3{init_traj_angle[2],0.f};

    RefTracking<2> ref1{r1,max_accel};
    RefTracking<2> ref2{r2,max_accel};
    RefTracking<2> ref3{r3,max_accel};

    float state[6] = {0.f,0.f,0.f,0.f,0.f,0.f};
    float t[3] = {0.f,0.f,0.f};
    float estimate_time_to_target[3] = {0.f,0.f,0.f};
    float segment_inertia[3] = 
    {
        (mass[0]+mass[1]+mass[2]) * (link_lengths[0]+link_lengths[1]+link_lengths[2]) * (link_lengths[0]+link_lengths[1]+link_lengths[2]) / 3.f,
        (mass[1]+mass[2]) * (link_lengths[1]+link_lengths[2]) * (link_lengths[1]+link_lengths[2]) / 3.f,
        mass[2] * link_lengths[2] * link_lengths[2] / 3.f
    };

    Eigen::Matrix<float, 6, 1> control_input;
    float torque_input_motor[3] = {0.f,0.f,0.f};
    float gravity_compensation[3] = {0.f,0.f,0.f};
    float puck_compensation[3] = {0.f,0.f,0.f};
    float puck_mass = 0.f;
    int ctrl = PID;
  
    NOTICE("Start manipulator thread");
    while (true) {
        /* check if param update */
        if (parameter_namespace_contains_changed(right_arm_params)) {
            right_arm.set_offsets({
                config_get_scalar("master/arms/right/offsets/q1"),
                config_get_scalar("master/arms/right/offsets/q2"),
                config_get_scalar("master/arms/right/offsets/q3"),
            });
            right_arm.set_tolerance({
                config_get_scalar("master/arms/right/tolerance/q1"),
                config_get_scalar("master/arms/right/tolerance/q2"),
                config_get_scalar("master/arms/right/tolerance/q3"),
            });

            right_arm.gripper.configure(config_get_scalar("master/arms/right/gripper/release"),
                                        config_get_scalar("master/arms/right/gripper/acquire"));
        }
        if (parameter_namespace_contains_changed(ref1_params)) {
            r1 << parameter_scalar_get(parameter_find(ref1_params, "angle")) - state[0], 0;
            t[0] = 0.;
            estimate_time_to_target[0] = ref1.getTimeToTarget(0);
            init_traj_angle[0] = state[0];
            ref1 = RefTracking<2>(r1,max_accel);
        }
        if (parameter_namespace_contains_changed(ref2_params)) {
            r2 << parameter_scalar_get(parameter_find(ref2_params, "angle")) - state[2], 0;
            t[1] = 0.;
            estimate_time_to_target[1] = ref2.getTimeToTarget(0);
            init_traj_angle[1] = state[2];
            ref2 = RefTracking<2>(r2,max_accel);
        }
        if (parameter_namespace_contains_changed(ref3_params)) {
            r3 << parameter_scalar_get(parameter_find(ref3_params, "angle")) - state[4], 0;
            t[2] = 0.;
            estimate_time_to_target[2] = ref3.getTimeToTarget(0);
            init_traj_angle[2] = state[4];
            ref3 = RefTracking<2>(r3,max_accel);
        }
        if (parameter_namespace_contains_changed(puck_params)) {
            puck_mass = parameter_scalar_get(parameter_find(puck_params, "mass"));
        }
        if (parameter_namespace_contains_changed(control_params)) {
            ctrl = parameter_scalar_get(parameter_find(control_params, "ctrl"));
        }

        /* Select PID or LQR controller */
        
        if(ctrl == PID)
        {
            Pose2D pose = right_arm.update();
            Angles input = right_arm.compute_control();
            // right_arm.apply(input);

            manipulator_state.pose.x = pose.x;
            manipulator_state.pose.y = pose.y;
            manipulator_state.pose.heading = pose.heading;
            manipulator_state.position.q1 = right_arm.angles()[0];
            manipulator_state.position.q2 = right_arm.angles()[1];
            manipulator_state.position.q3 = right_arm.angles()[2];
            manipulator_state.control.q1 = right_arm.sys.last_raw[0];
            manipulator_state.control.q2 = right_arm.sys.last_raw[1];
            manipulator_state.control.q3 = right_arm.sys.last_raw[2];
            messagebus_topic_publish(&manipulator_topic.topic, &manipulator_state, sizeof(manipulator_state));
        }
        else if(ctrl == LQR)
        {
            state[0] = constrainAngle(read_motor_position("theta-1"));
            state[1] = read_motor_velocity("theta-1");
            state[2] = constrainAngle(read_motor_position("theta-2"));
            state[3] = read_motor_velocity("theta-2");
            state[4] = constrainAngle(read_motor_position("theta-3"));
            state[5] = read_motor_velocity("theta-3");
          
            error(0) = init_traj_angle[0] + ref1.r(t[0])(0) - state[0];
            error(1) = ref1.v(t[0])(0) - state[1];
            error(2) = init_traj_angle[1] + ref2.r(t[1])(0) - state[2];
            error(3) = ref2.v(t[1])(0) - state[3];
            error(4) = init_traj_angle[2] + ref3.r(t[2])(0) - state[4];
            error(5) = ref3.v(t[2])(0) - state[5];
          
            // set control law
            control_input = TRANSMISSION_INERTIA*(lqr.system.B * gain[0] * error);

            gravity_compensation[0] = segment_inertia[0]*(GRAVITY_ACCEL / (link_lengths[0]+link_lengths[1]+link_lengths[2])) * sinf(state[0]);
            gravity_compensation[1] = segment_inertia[1]*(GRAVITY_ACCEL / (link_lengths[1]+link_lengths[2])) * sinf(state[2]);
            gravity_compensation[2] = segment_inertia[2]*(GRAVITY_ACCEL / link_lengths[2]) * sinf(state[4]);
            
            for(int i=0; i<3; i++){
              puck_compensation[i] = link_lengths[i]*link_lengths[i]*puck_mass;
              torque_input_motor[i] = control_input(2*i+1) + gravity_compensation[i] + puck_compensation[i];
            }
            
            motor_manager_set_torque(&motor_manager, "theta-1", math_clamp_value_f(torque_input_motor[0], -2.8, 2.8));
            motor_manager_set_torque(&motor_manager, "theta-2", math_clamp_value_f(torque_input_motor[1], -2.8, 2.8));
            motor_manager_set_torque(&motor_manager, "theta-3", math_clamp_value_f(torque_input_motor[2], -2.6, 2.6));

            t[0] += lqr.sampling_period;
            t[1] += lqr.sampling_period;
            t[2] += lqr.sampling_period;

            /* Ensure proper convergence to the target angle in case tuning is approximative */
            
            if(abs(t[0] - estimate_time_to_target[0]) < 2*lqr.sampling_period)
            {
                r1 << parameter_scalar_get(parameter_find(ref1_params, "angle")) - state[0], 0;
                estimate_time_to_target[0] = ref1.getTimeToTarget(0);
                t[0] = estimate_time_to_target[0]/4.f;
                init_traj_angle[0] = state[0];
                ref1 = RefTracking<2>(r1,max_accel);
            }
            if(abs(t[1] - estimate_time_to_target[1]) < 2*lqr.sampling_period)
            {
                r2 << parameter_scalar_get(parameter_find(ref2_params, "angle")) - state[2], 0;
                estimate_time_to_target[1] = ref2.getTimeToTarget(0);
                t[1] = estimate_time_to_target[1]/4.f;
                init_traj_angle[1] = state[2];
                ref2 = RefTracking<2>(r2,max_accel);
            }

            if(abs(t[2] - estimate_time_to_target[2]) < 2*lqr.sampling_period)
            {
                r3 << parameter_scalar_get(parameter_find(ref3_params, "angle")) - state[4], 0;
                estimate_time_to_target[2] = ref3.getTimeToTarget(0);
                t[2] = estimate_time_to_target[2]/4.f;
                init_traj_angle[2] = state[4];
                ref3 = RefTracking<2>(r3,max_accel);
            }
        }
        else{
          WARNING("Wrong controller param. Expecting %d for PID or %d for LQR but received %d.\n",PID,LQR,ctrl);
        }

        chThdSleepMilliseconds(1000 / MANIPULATOR_FREQUENCY);
    }
}

static THD_FUNCTION(manipulator_trajectory_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    NOTICE("Start manipulator trajectory manager thread");
    while (true) {
        chThdSleepMilliseconds(1000 / MANIPULATOR_TRAJECTORY_FREQUENCY);
    }
}

void manipulator_start(void)
{
    static THD_WORKING_AREA(manipulator_thd_wa, MANIPULATOR_THREAD_STACKSIZE);
    chThdCreateStatic(manipulator_thd_wa,
                      sizeof(manipulator_thd_wa),
                      MANIPULATOR_THREAD_PRIO,
                      manipulator_thd,
                      NULL);

    static THD_WORKING_AREA(manipulator_trajectory_thd_wa, MANIPULATOR_TRAJECTORY_THREAD_STACKSIZE);
    chThdCreateStatic(manipulator_trajectory_thd_wa,
                      sizeof(manipulator_trajectory_thd_wa),
                      MANIPULATOR_TRAJECTORY_THREAD_PRIO,
                      manipulator_trajectory_thd,
                      NULL);
}
