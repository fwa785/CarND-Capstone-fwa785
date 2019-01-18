from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # Create Yaw Controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Setup the PID controller to control the throttle
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # setup Low pass filter to filter out the noise in the velocity message
        tau = 0.5
        ts = .02
        self.velocity_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brack_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):

        # If DBW is not enabled, reset the PID controller
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # filter velocity noise
        current_velocity = self.velocity_lpf.filt(current_velocity)

        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Use PID controller to control the throttle
        velocity_err = linear_velocity - current_velocity
        self.last_velocity = current_velocity

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(velocity_err, sample_time)
        brake = 0

        if (linear_velocity == 0) and (current_velocity < 0.1):
            throttle = 0
            brake = 400
        elif (throttle < 0.1) and (velocity_err < 0):
            throttle = 0
            decel = max(velocity_err, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, steer
