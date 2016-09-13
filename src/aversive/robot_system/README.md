Robot system
============

Polar control vs per-wheel control
----------------------------------
Per-wheel control is the most widely used method of controlling the robot in the
Eurobot contest : Each wheel of the robot has its own control loop and
trajectory is controlled by giving the same consign to both wheel (to go
straight) and two consigns with opposite signs (to turn). This method is really
easy to use, but it has a few shortcomings :

* It can only be used to do straight lines and turns without moving (or it gets
  really complicated to use it for something else).
* It is pretty hard to tune, as the inertia in rotation is not the same as the
  inertia in translation.
* It makes some advanced moves, such as border alignment, pretty difficult.

Polar control is another control architecture that tries to fix the above
issues. The main idea is to have a loop to control robot's orientation
and another loop to control how far the robot has travelled. Then the outputs of
both control loops are *mixed* before being sent to the real motors.

Example usage of the polar control
----------------------------------
Suppose that we want to align the robot with a wall, or with the table border.
In a standard (per-wheel) configuration, this would be pretty tricky to do,
requiring complex control laws. In a polar control configuration, it becomes
really easy to do :

1. Disable angular control system. The robot can now rotate freely around its
   center axis.
2. Make the robot go backward. It will go back, hit the wall. Now the real magic
   happens : The robot will rotate around its center, aligning itself with the
   wall.
3. Re-enable angle control loop and go on with the rest of the program, knowing
   that you are well aligned with you reference.

Module usage
------------
The module was made so it can be used as a process input output function for a
control system, making its usage pretty straightforward. First of all, create a
robot system instance :

    struct rs myrs;
    rs_init(&myrs);

Then we will tell the rs module to use external encoders, instead of motors
encoders. The motors encoders can be used to detect wheel slippery, as Microb
Technology did in 2010. At the CVRA we don't use them.

	rs_set_flags(&myrs, RS_USE_EXT);

Then connect it to your *physical* motors (in this case using PWM0 and PWM5 on
address HEXMOTORCONTROLLER_BASE).

	rs_set_left_pwm(&robot.rs, cvra_dc_set_pwm0, HEXMOTORCONTROLLER_BASE);
	rs_set_right_pwm(&robot.rs, cvra_dc_set_pwm5, HEXMOTORCONTROLLER_BASE);

The last argument will be passed to your function as first argument, so the call
to `cvra_dc_set_pwm0` looks like this :

    cvra_dc_set_pwm0(HEXMOTORCONTROLLER_BASE, value);

Finally connect to encoders. There is a pitfall here : each encoder will get a
gain. This mean the value coming from the real encoder will be multiplied by
this constant before being mixed. This is used to compensate for small
differences between left and right wheel (which can cause huge error in
odometry). The gain can be negative if the encoder's A and B channels are
swapped.

	rs_set_left_ext_encoder(&myrs, cvra_dc_get_encoder0, HEXMOTORCONTROLLER_BASE, 1.0007);
	rs_set_right_ext_encoder(&myrs, cvra_dc_get_encoder5, HEXMOTORCONTROLLER_BASE,-0.9993);

Now you can use the robot system in a control system manager (see
`modules/control_system_manager` for informations about how to use it).

    /* Angle */
	cs_set_process_in(&angle_cs, rs_set_angle, &myrs);
	cs_set_process_out(&angle_cs, rs_get_ext_angle, &myrs);

    /* Distance */
	cs_set_process_in(&distance_cs, rs_set_distance, &myrs);
	cs_set_process_out(&distance_cs, rs_get_ext_distance, &myrs);
