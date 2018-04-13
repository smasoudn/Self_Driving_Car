import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
STOP_VELOCITY = 0.277778 # 1 km/h


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity,
                 brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel,
                 max_steer_angle,
                 sample_period):
        # init members
        self.sample_time = sample_period # 0.03 based on observation the interval is always around 0.03 seconds
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        # TODO: Implement
        self.velocity_controller = PID(0.65, 0.0, 0.0, mn=-1.0, mx=1.0)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1,
                                            max_lat_accel, max_steer_angle)
        self.lowpass_filter = LowPassFilter(tau=0.5, ts=0.1)



    def control(self, twist, current_velocity, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        velocity_cte = abs(twist.linear.x) - current_velocity.linear.x
        acceleration = self.velocity_controller.step(velocity_cte, dt)

        steer_delta = self.yaw_controller.get_steering(
            twist.linear.x,
            twist.angular.z,  # - current_velocity.twist.angular.z,
            current_velocity.linear.x)
        steer = self.lowpass_filter.filt(steer_delta)

        throttle = 0.0
        brake = 0.0

        # Note that throttle values passed to publish should be in the range 0 to 1.
        if twist.linear.x < STOP_VELOCITY:
            brake = self.calc_torque(abs(self.decel_limit))
        else:
            if acceleration < 0.0:
                deceleration = -acceleration
                if deceleration < self.brake_deadband:
                    brake = 0.0
                else:
                    brake = self.calc_torque(deceleration)
            else:
                throttle = acceleration

        return throttle, brake, steer

    def calc_torque(self, acceleration):

        return acceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius