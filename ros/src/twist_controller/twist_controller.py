from yaw_controller import YawController
from pid import PID
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base 		= kwargs['wheel_base']
        steer_ratio 	= kwargs['steer_ratio']
        min_velocity    = kwargs['min_velocity']
        max_lat_accel 	= kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        self.yawController = YawController(wheel_base, steer_ratio, min_velocity, max_lat_accel, max_steer_angle)

    #TODO: Rough implementation - needs refinement
    def control(self, linear_velocity_target, angular_velocity_target, linear_velocity_current):
        brake = 0.0
        throttle = 0.1
        steering = self.yawController.get_steering(linear_velocity_target, angular_velocity_target, linear_velocity_current)
        return throttle, brake, steering
