from collections import namedtuple
import cvra_rpc.message
from threading import Lock
import time
import math

WheelbaseTrajectoryPoint = namedtuple('WheelbaseTrajectoryPoint',
                                      ['x', 'y', 'v', 'theta', 'omega'])
WheelbaseTrajectory = namedtuple('WheelbaseTrajectory',
                                 ['start', 'dt', 'points'])
TrajectoryPoint = namedtuple('TrajectoryPoint',
                             ['position', 'speed', 'acceleration', 'torque'])

class Trajectory(namedtuple('Trajectory', ['start', 'dt', 'points'])):
    @classmethod
    def from_setpoint(cls, point, start, dt, duration):
        if isinstance(point, PositionSetpoint):
            point = TrajectoryPoint(point.value, 0, 0, 0)
        elif isinstance(point, SpeedSetpoint):
            point = TrajectoryPoint(0, point.value, 0, 0)
        elif isinstance(point, TorqueSetpoint):
            point = TrajectoryPoint(0, 0, 0, point.value)

        length = int(duration / dt)

        return cls(start=start, dt=dt, points=(point, ) * length)


Setpoint = namedtuple('Setpoint', ['value'])

class PositionSetpoint(Setpoint):
    pass


class SpeedSetpoint(Setpoint):
    pass


class TorqueSetpoint(Setpoint):
    pass


def update_actuator(setpoints, name, new):
    """
    Update the setpoints dictionnary with the given new traj.
    """
    if name not in setpoints:
        setpoints[name] = new
        return

    old = setpoints[name]

    if isinstance(old, WheelbaseTrajectory):
        # If the old setpoint is a trajectory for the wheelbase, we can
        # only merge it if the new trajectory is made for the wheelbase.
        if not isinstance(new, WheelbaseTrajectory):
            raise ValueError("Wheelbase can only updated with Wheelbase")

        setpoints[name] = trajectory_merge(old, new)

    elif isinstance(new, Setpoint):
        # If the new trajectory is a setpoint, apply it immediately.
        # The motor board will generate a ramp
        setpoints[name] = new

    elif isinstance(old, Trajectory):
        # If the old was a trajectory, simply merge it
        setpoints[name] = trajectory_merge(old, new)

    elif isinstance(old, Setpoint):
        # Finally, if the old was a setpoint, convert it into a trajectory,
        # then merge it
        start = time.time()
        old = Trajectory.from_setpoint(old, start, new.dt,
                                           new.start - start)
        setpoints[name] = trajectory_merge(old, new)


def trajectory_merge(first, second):
    if first.dt != second.dt:
        raise ValueError("Can only merge trajectories with same samplerate.")

    dt = first.dt

    # Small helper function to convert second to a discreet number of samples
    seconds_to_samples = lambda t: int(t / dt)

    start_index = seconds_to_samples(second.start - first.start)

    # Repeat last point of first trajectory until the start of the second
    padding_len = second.start - (first.start + dt * len(first.points))
    padding = first.points[-1:] * seconds_to_samples(padding_len)

    points = first.points[:start_index] + padding + second.points

    start = min(first.start, second.start)

    return Trajectory(start, first.dt, points)


def trajectory_gc(trajectory, date):
    """
    Frees up memory by deleting the part of trajectory after the given date.
    """


    skipped_points = int((date - trajectory.start) / trajectory.dt)
    skipped_points = max(0, skipped_points)
    date = max(date, trajectory.start)

    TrajectoryType = type(trajectory)
    return TrajectoryType(date, trajectory.dt, trajectory.points[skipped_points:])


def trajectory_to_chunks(traj, chunk_length):
    TrajType = type(traj)
    for i in range(0, len(traj.points), chunk_length):
        yield TrajType(traj.start + traj.dt * i,
                       traj.dt,
                       traj.points[i:i+chunk_length])


def trajectory_get_state(traj, date):
    index = round((date - traj.start) / traj.dt)

    # If past the last point, return last point
    index = min(index, len(traj.points) - 1)

    return traj.points[index]


class ActuatorPublisher:
    """
    This class encapsulates a trajectory collection (dict) in a thread safe
    way. It is the only public API of the project, and should be subclassed to
    implement various transports.
    """
    def __init__(self):
        self.trajectories = {}
        self.lock = Lock()

    def update_actuator(self, name, newtraj):
        with self.lock:
            update_actuator(self.trajectories, name, newtraj)

    def get_state(self, name, date):
        """
        Gets the expected state of the actuator at a given date in the future.
        Accessing a date in the past might trigger a KeyError if the trajectory
        has already been garbage collected.
        """
        with self.lock:
            traj = self.trajectories[name]

        if isinstance(traj, Setpoint):
            return traj

        return trajectory_get_state(traj, date)

    def gc(self, date):
        """
        Tries to free some memory by deleting all trajectory points before
        the given date.
        """
        with self.lock:
            for name, old in self.trajectories.items():
                if isinstance(old, Setpoint):
                    continue
                self.trajectories[name] = trajectory_gc(old, date)

    def publish(self, date):
        """
        This method is responsible for sending out the different actuator
        setpoints to the robot. It must be subclassed for every transport
        (SimpleRPC, ROS, UART, whatever).
        """
        pass

class SimpleRPCActuatorPublisher(ActuatorPublisher):
    """
    This class implements actuator setpoint publishing via simple RPC.
    """
    def __init__(self, target, *args, **kwargs):
        """
        Constructor. Target is a tuple (host, port) describing where the
        SimpleRPC commands should be sent.
        """
        super().__init__(*args, **kwargs)
        self.target = target

    def publish(self, date):
        commands = {PositionSetpoint: 'actuator_position',
                    SpeedSetpoint: 'actuator_velocity',
                    TorqueSetpoint: 'actuator_torque'}

        for name, setpoint in self.trajectories.items():
            if isinstance(setpoint, Setpoint):
                command = commands[type(setpoint)]
                cvra_rpc.message.send(self.target, command, [name, float(setpoint.value)])

            elif isinstance(setpoint, Trajectory):
                # Convert the trajectory to chunks, then select the first one
                # still in the future.
                chunks = trajectory_to_chunks(setpoint, 10)
                chunk = next(chunks)
                while chunk.start < date:
                    chunk = next(chunks)

                points = [[p.position, p.speed, p.acceleration, p.torque] for p in chunk.points]

                start_s = int(chunk.start)
                start_us = int((chunk.start - start_s) * 1e6)

                cvra_rpc.message.send(self.target, 'actuator_trajectory',
                                      [name, start_s, start_us, points])

            elif isinstance(setpoint, WheelbaseTrajectory):
                chunks = trajectory_to_chunks(setpoint, 10)
                chunk = next(chunks)
                while chunk.start < date:
                    chunk = next(chunks)

                start_s = int(chunk.start)
                start_us = int((chunk.start - start_s) * 1e6)

                cvra_rpc.message.send(self.target,
                                      'wheelbase_trajectory',
                                      [start_s, start_us, chunk.points])
