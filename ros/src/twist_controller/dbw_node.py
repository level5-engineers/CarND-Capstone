#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

#DEBUG JWD
#from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
#ENDEBUG

import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass    = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband  = rospy.get_param('~brake_deadband', .1)
        decel_limit     = rospy.get_param('~decel_limit', -5)
        accel_limit     = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius    = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base      = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        #steer_ratio     = 2.67 # <-- original value for simulator
        decel_limit     = -1.0
        accel_limit     = 0.5

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        parms = {
            'wheel_base'      : wheel_base,
            'steer_ratio'     : steer_ratio,
            'min_velocity'    : 0.,
            'max_lat_accel'   : max_lat_accel,
            'max_steer_angle' : max_steer_angle,
            'decel_limit'     : decel_limit,
            'accel_limit'     : accel_limit
        }
        self.controller = Controller(**parms)
 
        self.current_command = None
        self.current_velocity = None
        self.dbw_enabled = False
        self.oneshot = False  # ISSUE ONLY ONE THROTTLE COMMAND
        self.frameCount = 0
        
        rospy.Subscriber('/twist_cmd', TwistStamped, self.callback_twist_cmd)
        rospy.Subscriber('/current_velocity', TwistStamped, self.callback_current_velocity)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.callback_dbw_enabled)
        
        #DEBUG JWD
        #rospy.Subscriber('/current_pose', PoseStamped, self.callback_pose)
        self.x = 0.
        self.y = 0.
        rospy.Subscriber('/final_waypoints', Lane, self.callback_waypoints)

        self.loop()

    # this is called every 3 seconds
    def callback_waypoints(self, waypoints):
        #self.controller.reset() # <-- Reset the velocity PID
        pass
    
    def callback_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
    #ENDEBUG

    # Get predicted throttle, brake, and steering using `controller` object
    # Controller class is roughly defined, so we are simply moving forward 
    # 0.1 throttle, attempting to steer. Braking is OFF
    def loop(self):
        rate = rospy.Rate(50) # 20Hz
        while not rospy.is_shutdown():
            if (self.current_command is not None) and (self.current_velocity is not None):
                # get the current velocity, target velocity, and target angle and pass into control
                linear_target  = self.current_command.twist.linear.x;
                angular_target = self.current_command.twist.angular.z;
                linear_current = self.current_velocity.twist.linear.x;
                angular_current = self.current_velocity.twist.angular.z;
                #rospy.loginfo("linearc: %.2f, lineart: %.2f, angularc: %.2f, angulart: %.2f", linear_current, linear_target, angular_current, angular_target)
                throttle, brake, steering = self.controller.control(linear_target, angular_target, linear_current)
                
                # Publish the control commands if dbw is enabled
                if self.dbw_enabled:
                    #if self.frameCount < 1000:
                    rospy.loginfo("v: %.2f, vtarg: %.2f, thr: %.2f, b: %.2f, s: %.2f", linear_current, linear_target, throttle, brake, steering)
                    self.publish(throttle, brake, steering)
                    #else:
                    #    rospy.loginfo("v: %.2f, t: %.2f, b: %.2f, s: %.2f debug: x: %.2f, y: %.2f", linear_current, 0.0, 1.0, 0.0, self.x, self.y)
                    #    self.publish(0., 1.0, 0.)
                    #self.frameCount += 1
                else:
                    self.controller.reset()
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        if throttle > 0.0:
            tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        #TODO: test throttle
        #if self.oneshot == False:  # ISSUE ONLY ONE THROTTLE COMMAND
        self.throttle_pub.publish(tcmd)
        #    self.oneshot = True
        
        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_PERCENT
        bcmd.pedal_cmd = brake * 1000.
        
        #TODO: test braking
        #bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        #if brake > 0. and throttle == 0.:
            #bcmd.pedal_cmd = brake * self.vehicle_mass * self.wheel_radius
        #    bcmd.pedal_cmd = BrakeCmd.TORQUE_MAX * brake    # 3412.
        self.brake_pub.publish(bcmd)

    def callback_twist_cmd(self, msg):
        self.current_command = msg

    def callback_current_velocity(self, msg):
        self.current_velocity = msg

    def callback_dbw_enabled(self, msg):
        self.dbw_enabled = msg.data

if __name__ == '__main__':
    DBWNode()
