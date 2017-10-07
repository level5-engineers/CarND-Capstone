from yaw_controller import YawController
from pid import PID
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base 	    = kwargs['wheel_base']
        steer_ratio     = kwargs['steer_ratio']
        min_velocity    = kwargs['min_velocity']
        max_lat_accel   = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']

        decel_limit     = kwargs['decel_limit']
        accel_limit     = kwargs['accel_limit']

        self.yawController = YawController(wheel_base, steer_ratio, min_velocity, max_lat_accel, max_steer_angle)
        #self.correcting_pid = PID(0.065, 0.000775, 0.775, -1.57, 1.57)
        self.velocity_pid = PID(0.8, 0.0, 0.05, decel_limit, accel_limit)

    #TODO: Rough implementation - needs refinement
    def control(self, linear_velocity_target, angular_velocity_target, linear_velocity_current):
        brake = 0.
        throttle_correction = self.velocity_pid.step(linear_velocity_target-linear_velocity_current, 0.02)  #0.0414
        if throttle_correction > 0.:
            throttle = throttle_correction
        else:
            throttle = 0.
            brake = -throttle_correction
            if (linear_velocity_target) == 0. and (brake < 0.1):
                brake = 0.1 # hold brake while stopped at red light, until light changes
                
        steering = self.yawController.get_steering(linear_velocity_target, angular_velocity_target, linear_velocity_current) # self.correcting_pid.step(angular_velocity_target, 0.1)
        return throttle, brake, steering
    def reset(self):
        self.velocity_pid.reset()
