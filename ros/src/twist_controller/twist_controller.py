from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base 	     = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        min_velocity     = kwargs['min_velocity']
        max_lat_accel    = kwargs['max_lat_accel']
        max_steer_angle  = kwargs['max_steer_angle']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.deadband    = kwargs['deadband']

        self.yawController = YawController(wheel_base, self.steer_ratio, min_velocity, max_lat_accel, max_steer_angle)
        self.velocity_pid = PID(0.5, 0., 0., self.decel_limit, self.accel_limit)
        self.lowpassFilt = LowPassFilter(0.07, 0.02)
        self.topVelocity = 0.

    def control(self, linear_velocity_target, angular_velocity_target, linear_velocity_current):
        
        # Throttle xor brake
        brake = 0.
        throttle_correction = self.velocity_pid.step(linear_velocity_target-linear_velocity_current, 0.02)
        if throttle_correction > 0.:
            throttle = throttle_correction
            throttle = self.lowpassFilt.filt(throttle)
        elif throttle_correction < -self.deadband:
            throttle = 0.
            brake = -throttle_correction
        else:
            throttle = 0.
        # hold brake while stopped at red light, until light changes
        if (linear_velocity_target <= 0.01) and (brake < self.deadband):
            brake = self.deadband
        
        # Steering
        # Use yawController for simulator
        if self.deadband>0.1:
            if linear_velocity_target > self.topVelocity: # mitigate rapid turning
                self.topVelocity = linear_velocity_target
            if linear_velocity_current > 0.05:
                steering = self.yawController.get_steering(self.topVelocity, angular_velocity_target, linear_velocity_current)
            else:
                steering = 0.
        # ...and alternate approach (to match reference) on the test site:
        else:
            steering = angular_velocity_target * self.steer_ratio
        return throttle, brake, steering
    
    def reset(self):
        self.velocity_pid.reset()
