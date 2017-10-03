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
        
        # other member variables we need
        self.x = None
        self.y = None
        self.t = None
        self.v = 10.0 #mps
        self.wp = None
        self.red_light_waypoint = None
        
        #JWD DEBUG
        self.current_velocity = None
        
        # do not exit, await shutdown
        #rospy.spin()
        self.loop()

    def distance2(self, p1, p2):
        x, y = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(x*x + y*y)
 
    def loop(self):
        rate = rospy.Rate(1./3) # Update once every 3 seconds
        while not rospy.is_shutdown():
            if (self.wp is not None) and (self.x is not None):
                # JWD: moved update loop into this callback
                if self.wp is None:
                    return
                # get the index of the closest waypoint
                idx = assist.nearest_waypoint(self.wp, self.x, self.y, self.t)
                #rospy.loginfo("idx: %d, x: %.2f, y: %.2f, t: %.2f", idx, self.x, self.y, self.t)
                
                # make a lane object
                lane = Lane()
                
                numPts = len(self.wp.waypoints)
                # and add a list of waypoints
                for _ in range(LOOKAHEAD_WPS):
                    wp = self.wp.waypoints[idx]
                    new_point = Waypoint()
                    new_point.pose = wp.pose
                    
                    # set the velocity at each waypoint
                    new_point.twist.twist.linear.x = self.v
                    if self.red_light_waypoint is not None:
                        if (self.current_velocity is not None) and (idx <= self.red_light_waypoint):
                            sidx = self.red_light_waypoint
                            stopdist = self.distance2(wp.pose.pose.position, self.wp.waypoints[sidx].pose.pose.position)
                            #vel = math.sqrt(2 * MAX_DECEL * stopdist) * 3.6
                            #if vel < 1.:
                            #    vel = 0.
                            #new_point.twist.twist.linear.x = min(vel, self.v)
                            currVel = self.current_velocity.twist.linear.x
                            if stopdist < 2.*currVel*currVel/(2.*MAX_DECEL):
                                new_point.twist.twist.linear.x = 0.
                            else:
                                if stopdist < 4.*currVel*currVel/(2.*MAX_DECEL):
                                    new_point.twist.twist.linear.x = self.v / 2.
                                else:
                                    new_point.twist.twist.linear.x = self.v
                            rospy.loginfo("idx: %d, vel: %.2f, stopdist: %.2f", idx, new_point.twist.twist.linear.x, stopdist)
                        else: # all points beyond the stop line
                            new_point.twist.twist.linear.x = 0.
                    
                    # append the point
                    lane.waypoints.append(new_point)
                    idx = (idx + 1) % numPts
                
                # send
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
