#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import assist
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

#JWD DEBUG
from geometry_msgs.msg import TwistStamped

LOOKAHEAD_WPS = 150 # Number of waypoints to publish. (Can be changed.)
MAX_DECEL = 2.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        rospy.Subscriber('/current_pose', PoseStamped, self.callback_pose)
        rospy.Subscriber('/base_waypoints', Lane, self.callback_waypoints)

#JWD DEBUG
        rospy.Subscriber('/current_velocity', TwistStamped, self.callback_current_velocity)

        # TODO: Add a subscriber for /traffic_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.callback_traffic)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # current coords of car
        self.x_current = None
        self.y_current = None
        
        # t = theta (yaw)
        self.theta_current = None
        
        # max target velocity
        self.max_velocity = 10.0 #mps
        
        # list of base waypoints
        self.base_waypoints = None
        
        # stop line for the nearest red/yellow light, and if green then None
        self.red_light_waypoint = None
        
        #JWD DEBUG
        self.current_velocity = None
        
        # do not exit, await shutdown
        #rospy.spin()
        self.loop()

    # Distance between two points
    # p is the position structure
    def distance2(self, p1, p2):
        x, y = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(x*x + y*y)
 
    # Main logic
    def loop(self):
        rate = rospy.Rate(1./3) # Update once every 3 seconds
        while not rospy.is_shutdown():
            # we have base waypoints and current position
            if (self.base_waypoints is not None) and (self.x_current is not None):
                # JWD: moved update loop into this callback
                #if self.wp is None:
                #    return
                
                # get the index of the closest waypoint
                nearest_waypoint = assist.nearest_waypoint(self.base_waypoints, self.x_current, self.y_current, self.theta_current)
                
                #rospy.loginfo("idx: %d, x: %.2f, y: %.2f, t: %.2f", idx, self.x, self.y, self.t)
                
                # make a lane object
                lane = Lane()
                
                # number of base waypoints
                numPts = len(self.base_waypoints.waypoints)
                
                # and add a list of waypoints
                for _ in range(LOOKAHEAD_WPS):
                    # nearest waypoint object
                    wp = self.base_waypoints.waypoints[nearest_waypoint]
                    
                    new_point = Waypoint()
                    new_point.pose = wp.pose
                    
                    # set the velocity at each waypoint
                    
                    # X direction is forward from the car at any position regardless of orientation or position
                    new_point.twist.twist.linear.x = self.target_velocity
                    
                    #**** Red light *****
                    if self.red_light_waypoint is not None:
                        # If we have info on current vel and any waypoint is before the red light
                        if (self.current_velocity is not None) and (nearest_waypoint <= self.red_light_waypoint):
                            sidx = self.red_light_waypoint
                            
                            stopdist = self.distance2(wp.pose.pose.position, self.base_waypoints.waypoints[sidx].pose.pose.position)
                            
                            #vel = math.sqrt(2 * MAX_DECEL * stopdist) * 3.6
                            #if vel < 1.:
                            #    vel = 0.
                            #new_point.twist.twist.linear.x = min(vel, self.v)
                            currVel = self.current_velocity.twist.linear.x
                            
                            if stopdist < 2.*currVel*currVel/(2.*MAX_DECEL):
                                new_point.twist.twist.linear.x = 0.
                            else:
                                if stopdist < 4.*currVel*currVel/(2.*MAX_DECEL):
                                    new_point.twist.twist.linear.x = self.target_velocity / 2.
                                else:
                                    new_point.twist.twist.linear.x = self.target_velocity
                            rospy.loginfo("idx: %d, vel: %.2f, stopdist: %.2f", nearest_waypoint, new_point.twist.twist.linear.x, stopdist)
                        else: # all points beyond the stop line
                            new_point.twist.twist.linear.x = 0
                    
                    # append the point
                    lane.waypoints.append(new_point)
                    # idx = index of next waypoint. If the track is a loop, the index will start again
                    nearest_waypoint = (nearest_waypoint + 1) % numPts
                
                # send
                # in latest version conditional on whether lane gets updated or not
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

    # TODO: refine this initial implementation
    def callback_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        o = msg.pose.orientation
        quat = [o.x, o.y, o.z, o.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.t = euler[2]

    # JWD: called once
    def callback_waypoints(self, waypoints):
        self.wp = waypoints

    def callback_traffic(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_waypoint = msg.data-2 if msg.data >= 0 else None
        #pass

    #JWD DEBUG
    def callback_current_velocity(self, msg):
        self.current_velocity = msg

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
