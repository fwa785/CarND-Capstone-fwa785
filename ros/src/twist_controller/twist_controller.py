
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        pass

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # Return throttle, brake, steer
        return 1., 0., 0.
