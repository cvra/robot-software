Control system manager
======================
This module is responsible for implementing a closed loop control system.

What is closed loop control ?
-----------------------------

According to Wikipedia :
> In a closed-loop control system, a sensor monitors the system output (the car's
> speed) and feeds the data to a controller which adjusts the control (the
> throttle position) as necessary to maintain the desired system output (match the
> car's speed to the reference speed.) Now, when the car goes uphill, the decrease
> in speed is measured, and the throttle position changed to increase engine
> power, speeding up the vehicle. Feedback from measuring the car's speed has
> allowed the controller to dynamically compensate for changes to the car's speed.
> It is from this feedback that the paradigm of the control loop arises: the
> control affects the system output, which in turn is measured and looped back to
> alter the control.

So, a closed loop control system is an algorithm to drive some kind of process
to go and stay at some state (the *consign*), which is the same as keeping the
*error* as low as possible.

Another goal of a control system is for it to be *robust* which means it should
react well to small perturbations on the robot. It should also be *stable*,
which means it will not enter oscillation, or worst, divergate.


How does it work ?
------------------
The classic closed control loop architecture contains a *controller* and a
*process*. The controller is fed with the error (the difference between the
wanted state and the measured state) and its output is fed into the process.
This typical architecture can be seen below.

![General architecture of a closed loop control system](https://raw.github.com/antoinealb/modules/master/modules/control_system_manager/general_architecture.png)

The controller can be very specific, or a general algorithm tuned for your
needs, for example a PID (see `modules/pid`).

How is it implemented ?
-----------------------
When it gets to pratical implementation, things become slightly more
complicated. We added several other filters to the design (apart from the
controller) to make things easier to use in a robot. The good thing is, those
filters are not mandatory, as the module will ignore them if you don't use them.

So the added filters are :
* A consign filter, which is applied to the consign before using it to compute
  the error. This filter is useful, for example, if you want a consign that
  varies with a maximum speed and acceleration, to avoid wheel slipping. Such
  a filter is implemented in `modules/quadramp`.
* A feedback filter, which is applied to the process state measurement (called
  process output in the code) before using it to compute the error. It can be
  used if the process output is noisy and you want to remove that noise before
  computing the error. This filter is not used on the CVRA robots, therefore it
  might contains bugs.
* A process input filter. This filter was added mainly to develop a torque
  limiting application at the CVRA, but this feature was removed from the robot.

So the complete control system architecture can be seen below. Most filters are
shown with their most common application in a robot / motor control.

![Implementation of the control system manager](https://raw.github.com/antoinealb/modules/master/modules/control_system_manager/implementation.png)

How to use it ?
---------------
In this example, we will create a simple control system, to which we will add
a ramp later.

Before we start, you will need to provide the control system manager 2
functions, one to give an input to the process and one to read its output. The
function prototypes must be :

    int32_t read_myprocess_out(void *param);
    void write_myprocess_in(void *param, int32_t value);


So first of all, let's declare and initialize the control system manager :

    struct cs mycs; /* cs stands for Control System. */
    cs_init(&mycs);

Now that we have a control system, it would be better if it would be connected
to our process. Here both functions don't use a parameter, so we just use NULL.
Otherwise, the third argument of `cs_set_process_*` would be passed to our
process in/out function as first parameter.
    
    cs_set_process_in(&mycs, write_myprocess_in, NULL);
    cs_set_process_out(&mycs, read_myprocess_out, NULL);


For the correct filter (also called the controller), we will use a simple PID
controller. See `modules/pid` for details about this type of filters. The tuning
parameters are only given here as an example. Fine tuning is required for a
specific process

    struct pid_filter mypid;
    int K=10, I=0, D=3;
    pid_init(&mypid);
    pid_set_gains(&mypid, K, I, D);
	pid_set_maximums(&mypid, 0, 5000, 30000); 
	pid_set_out_shift(&mypid, 10);

Now we want to tell the control system manager to use our new PID as a
controller. This is done in a single function call :

    cs_set_correct_filter(&mycs, pid_do_filter, &mypid);

Finally, we have to call the control system manager at a fixed frequency and it
will handle all the details automatically, like reading the process output and
applying the filters in the right order. In this example, the frequency is about
100 Hz, but in a real application, you might want to do this in an interrupt to
get a stable frequency.

    while(1) {
        cs_manage(&mycs);
        wait_ms(10);
    }

### Changing consign
Now we got a control system that can prevent the system from moving around the
zero position. What if we want to use our control system to move the system to a
new state ? For this example, we will move the system to a new position after a
second (1000 ms). Replace the while loop above with the following code.

    int time=0;
    int new_consign_value = 300;
    while(1) {
        if(time == 1000)
            cs_set_consign(&mycs, new_consign_value);
        cs_manage(&mycs);
        wait_ms(10);
        time += 10;
    }

### Limiting rate of consign change
So now our controller is able to move the system to a new state, but it will do
it immediately. What if we want to limit the rate of change of the system, for
example to limit maximum speed and acceleration in a robot ? This requires a
*consign filter* to be added to the system. In this example, we will use the
`modules/quadramp` filter, which limits the first and second derivative of the
consign. In our example robot position control, this would limit the speed and
the acceleration of the robot.

First of all let's create the filter. Put the following code after the
`cs_set_correct_filter` call.

    struct quadramp_filter myqr;
    quadramp_init(&myqr);

We will now configure the quadramp with a maximum acceleration of 1 and a
maximum speed of 10. See the quadramp documentation for details.

    quadramp_set_2nd_order_vars(&myqr, 1, 1);
    quadramp_set_1st_order_vars(&myqr, 10, 10);

The last step is to tell the control system manager to use the quadramp as a
consign filter.

    cs_set_consign_filter(&mycs, quadramp_do_filter, &myqr);

Now the system would accelerate slowly, reach a maximum speed, and then brake
slowly to reach the target.

As you can see, the architecture with 3 separate filters (consign, correct and
feeback) allows for a very flexible architecture and good code reuse.
    
