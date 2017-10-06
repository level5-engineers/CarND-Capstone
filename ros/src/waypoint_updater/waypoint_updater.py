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

LOOKAHEAD_WPS = 20 # Number of waypoints to publish. (Can be changed.)
MAX_DECEL = 2.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
	rospy.loginfo("Hajime!!!")

	    # current coords of car
        self.x_current = None
        self.y_current = None
        self.theta_current = None
        
        # max target velocity, constant
        self.target_velocity = 10.0 #mps
        
        # list of base waypoints, set only once
        self.base_waypoints = None
        
        # list of previous waypoints, gets updated after each loop
        self.previous_waypoints = None
        
        # stop line for the nearest red/yellow light, and if green then None
        self.red_light_waypoint = None
        
        # current velocity of car, set by subscriber
        self.current_velocity = None
        
        rospy.Subscriber('/current_pose', PoseStamped, self.get_current_position)
        rospy.Subscriber('/base_waypoints', Lane, self.get_base_waypoints)
        rospy.Subscriber('/current_velocity', TwistStamped, self.get_current_velocity)
        rospy.Subscriber('/traffic_waypoint', Int32, self.callback_traffic)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # do not exit, await shutdown
        #rospy.spin()
        self.loop()

    # Main logic
    def loop(self):
        rate = rospy.Rate(5) # Update 5 times per second
        
        while not rospy.is_shutdown():
            x_current = self.x_current
            y_current = self.y_current
            theta_current = self.theta_current
            current_velocity = self.current_velocity
            
            # we have base waypoints and current position
            if (self.base_waypoints is not None) and (self.x_current is not None):
                # get the index of the closest waypoint
                nearest_waypoint = assist.nearest_waypoint(self.base_waypoints, x_current, y_current, theta_current)
                rospy.loginfo("Got the nearest waypoint: x=%.2f, y=%.2f", self.base_waypoints.waypoints[nearest_waypoint].pose.pose.position.x, self.base_waypoints.waypoints[nearest_waypoint].pose.pose.position.y)
                
                # make a lane object
                lane = Lane()
                
                # number of base waypoints
                numPts = len(self.base_waypoints.waypoints)
                rospy.loginfo("Alright, lets set the waypoints for this loop")
                # and add a list of waypoints
                for _ in range(LOOKAHEAD_WPS):
                    # nearest waypoint object
                    wp = self.base_waypoints.waypoints[nearest_waypoint]
                    
                    new_point = Waypoint()
                    new_point.pose = wp.pose
                    
                    # X direction is forward from the car at any position regardless of orientation or position
                    delta_v = 1.0
                    if current_velocity >= self.target_velocity:
                        delta_v = 0.0
                    new_point.twist.twist.linear.x = current_velocity + delta_v
                    current_velocity += delta_v
                    
                    # append the point
                    lane.waypoints.append(new_point)
                    nearest_waypoint = (nearest_waypoint + 1) % numPts
                    rospy.loginfo("x=%.2f, y=%.2f, v=%.2f", new_point.pose.pose.position.x, new_point.pose.pose.position.y, new_point.twist.twist.linear.x)
                # --- for loop ends -----
                
                # Publish!
		rospy.loginfo("Publishing points\n")
                self.final_waypoints_pub.publish(lane)
                
            # --- if ends, still in while loop ----
            rate.sleep()
        rospy.loginfo("Shutdown.")

    def get_current_position(self, msg):
        self.x_current = msg.pose.position.x
        self.y_current = msg.pose.position.y
        o = msg.pose.orientation
        quat = [o.x, o.y, o.z, o.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.theta_current = euler[2]
	rospy.loginfo("Current position received")

    def get_base_waypoints(self, waypoints):
        self.base_waypoints = waypoints
	rospy.loginfo("Base waypoints set")

    def callback_traffic(self, msg):
        self.red_light_waypoint = msg.data-2 if msg.data >= 0 else None

    def get_current_velocity(self, msg):
        self.current_velocity = msg.twist.linear.x
        rospy.loginfo("Current velocity set")

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # Euclidean distance between two points
    # p is the position structure
    def euclidean_distance(self, p1, p2):
        x, y = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(x*x + y*y)
     
    # Total distance between any two waypoints in the list, along the arc
    def arc_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def red_light_visible():
        pass
    
    def destination_visible():
        pass
    
    def final_waypoint_details():
        return []
    
    def steps():
        return []
    

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
