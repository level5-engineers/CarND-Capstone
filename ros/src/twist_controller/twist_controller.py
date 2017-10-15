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
        accel_limit      = kwargs['accel_limit']
        self.deadband    = kwargs['deadband']

        self.yawController = YawController(wheel_base, self.steer_ratio, min_velocity, max_lat_accel, max_steer_angle)
        #self.correcting_pid = PID(0.065, 0.000775, 0.775, -1.57, 1.57)
        #self.velocity_pid = PID(0.8, 0.001, 0.2, decel_limit, accel_limit)
        #self.velocity_pid = PID(1.6, 0.00001, 0.04, decel_limit, accel_limit)  Good: best so far
        #self.velocity_pid = PID(2.0, 0.4, 0.1, decel_limit, accel_limit) Too much bias
        #self.velocity_pid = PID(0.1, 0.001, 1.0, decel_limit, accel_limit) Not responsive enough
        #self.velocity_pid = PID(0.9, 0.0005, 0.07, decel_limit, accel_limit) fair
        #self.velocity_pid = PID(5.0, 0.00001, 0.2, decel_limit, accel_limit) ok
        #self.velocity_pid = PID(2.0, 0.0, 0.05, decel_limit, accel_limit) not as good as 1.6
        #self.velocity_pid = PID(2.0, 0.00001, 0.04, decel_limit, accel_limit) good
        #self.velocity_pid = PID(0.35, 0., 0., decel_limit, accel_limit)
        self.velocity_pidHI = PID(0.35, 0., 0., self.decel_limit, accel_limit)
        self.velocity_pidLO = PID(0.7, 0., 0., self.decel_limit/2., accel_limit)
        #self.lowpassFilt = LowPassFilter(accel_limit/2., 0.02) # good
        self.lowpassFilt = LowPassFilter(0.07, 0.02)

    def control(self, linear_velocity_target, angular_velocity_target, linear_velocity_current):
        # Throttle
        if linear_velocity_target > 2.:
            throttle_correction = self.velocity_pidHI.step(linear_velocity_target-linear_velocity_current, 0.02)
        else:
            throttle_correction = self.velocity_pidLO.step(linear_velocity_target-linear_velocity_current, 0.02)
        throttle_correction = self.lowpassFilt.filt(throttle_correction)
        if throttle_correction > 0.:
            brake = 0.0
            if linear_velocity_target <= 0.01:
                throttle = 0.
                self.velocity_pidLO.reset() # <-- Reset the velocity PID
                self.velocity_pidHI.reset()
            else:
                throttle = throttle_correction
        # Braking
        else:
            throttle = 0.
            brake = -throttle_correction
            if brake < self.deadband * self.decel_limit/2.:
                brake = 0
                
        # Steering
        if linear_velocity_current > 0.05:
            steering = self.yawController.get_steering(linear_velocity_target, angular_velocity_target, linear_velocity_current) # self.correcting_pid.step(angular_velocity_target, 0.1)
        else:
            steering = 0.
        # Alternate approach: steering = angular_velocity_target * self.steer_ratio
        return throttle, brake, steering
    def reset(self):
        self.velocity_pidLO.reset()
        self.velocity_pidHI.reset()
