#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 5 # Number of waypoints to publish. (Can be changed.)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        rospy.Subscriber('/current_pose', PoseStamped, self.callback_pose)
        rospy.Subscriber('/base_waypoints', Lane, self.callback_waypoints)
        
        # TODO: Add a subscriber for /traffic_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Lane, self.callback_traffic) ??
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # other member variables we need
        self.x = None
        self.y = None
        self.t = None
        self.v = 4.0
        
        # do not exit, await shutdown
        rospy.spin()

    # TODO: refine this initial implementation
    def callback_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        o = msg.pose.orientation
        quat = [o.x, o.y, o.z, o.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.t = euler[2]

    # TODO: refine this initial implementation
    def callback_waypoints(self, waypoints):
        if self.x is None:
            return
        
        # get the index of the closest waypoint
        idx = assist.nearest_waypoint(waypoints, self.x, self.y, self.t)
        
        # make a lane object
        lane = Lane()
        
        # and add a list of waypoints
        for _ in range(LOOKAHEAD_WPS):
            wp = waypoints.waypoints[idx]
            new_point = Waypoint()
            new_point.pose = wp.pose
            
            # set the velocity at each waypoint
            new_point.twist.twist.linear.x = self.v
            
            # append the point
            lane.waypoints.append(new_point)
            idx += 1
            
        # send
        self.final_waypoints_pub.publish(lane)


    def callback_traffic(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
