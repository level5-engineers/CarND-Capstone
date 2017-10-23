#!/usr/bin/env python

import os
import csv
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

PUBLISHING_RATE = 2 # Publishing frequency (Hz)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Member variables
        self.current_pose   = None  # current coords of vehicle
        self.base_waypoints = None  # list of base waypoints
        self.queue_wp       = None  # waypoints to publish
        self.next_waypoint  = None  # index of next waypoint
        self.stop_waypoint  = None  # stop line index for the nearest light
        self.next_basewp    = None  # the next waypoint index to retrieve from base
        self.destination    = None  # the final waypoint in the list
        self.num_base_wp    = 0     # the number of points in the base list
        self.msg_seq_num    = 0     # sequence number of published message
        self.velocity_drop  = 62.   # distance to begin reducing velocity
        self.VELOCITY_MAX   = 2.777 # mps Carla max of 10 km/h (updated by waypoints_cb)
        self.LOOKAHEAD_WPS  = 20    # Number of waypoints we will publish.
        self.prev_state     = None  # previous traffic light state
        self.halt           = False # shut down
        self.replan         = True  # when a light changes, update velocity
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.traffic_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Operations loop and publishing of /final_waypoints
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            self.update()
            if self.queue_wp is not None:
                self.publish(self.queue_wp)
            rate.sleep()

    def update(self):
        if (self.base_waypoints is None) or (self.current_pose is None):
            return
        
        # find the closest next waypoint in the queue
        self.next_waypoint = self.get_next_waypoint(self.queue_wp)
                        
        # if the queue is empty or next waypoint is not found in the queue
        if self.next_waypoint is None:
            # find the closest waypoint in the base waypoints
            next_wp = self.get_next_waypoint(self.base_waypoints)
            # initialize queue
            wp_idx = [idx % self.num_base_wp for idx in range(next_wp, next_wp + self.LOOKAHEAD_WPS)]
            self.queue_wp = [self.base_waypoints[wp] for wp in wp_idx]
            self.next_waypoint = 0
            self.next_basewp = (next_wp + self.LOOKAHEAD_WPS) % self.num_base_wp
            rospy.loginfo("Queue initialized with %d waypoints from base_waypoints", self.LOOKAHEAD_WPS)
        
        # manage queue
        if self.next_waypoint is not 0:
            # dequeue until head=next_waypoint
            del self.queue_wp[:self.next_waypoint]
            '''rospy.loginfo("Dequeued %d waypoints from queue", self.next_waypoint)'''
            # enqueue until length is LOOKAHEAD_WPS
            for i in range(self.next_waypoint):
                self.queue_wp.append(self.base_waypoints[self.next_basewp])
                self.update_waypoint_velocity(self.LOOKAHEAD_WPS-self.next_waypoint+i)
                '''rospy.loginfo("Enqueing waypoint %d to queue", self.next_basewp)'''
                
                self.next_basewp += 1                             # TODO: modulo calc
                if self.next_basewp == len(self.base_waypoints):  # handle end of track
                    self.next_basewp = 0                          # wrap around to the beginning
            '''rospy.loginfo("Queue has %d items", len(self.queue_wp))'''
        if self.replan:
            self.update_velocities()
            self.replan = False

    def update_velocities(self):
        # update all velocities in the queue
        #   if red in range of 4m (site) or 62m (sim) decel
        
        # for all waypoints in the queue
        for idx in range(len(self.queue_wp)):
            self.update_waypoint_velocity(idx)

    def update_waypoint_velocity(self, index):
        # By default, set the velocity to maximum
        self.queue_wp[index].twist.twist.linear.x = self.VELOCITY_MAX
        # unless we are in a halt state
        if self.halt:
            self.queue_wp[index].twist.twist.linear.x = 0.
        # otherwise...
        elif self.stop_waypoint is not None:
            # get the distance to the red light
            sidx = self.stop_waypoint
            distance_to_stopline = self.distance2(self.queue_wp[index].pose.pose.position, self.base_waypoints[sidx].pose.pose.position)
            # and calculate the ratio to reduce the velocity
            if distance_to_stopline <= 0:  # set velocity at or beyond stop line to zero
                self.queue_wp[index].twist.twist.linear.x = 0.
            elif distance_to_stopline < self.velocity_drop:
                ratio = distance_to_stopline / self.velocity_drop
                if distance_to_stopline <= 2.0:
                    ratio = 0.
                self.queue_wp[index].twist.twist.linear.x = self.VELOCITY_MAX * ratio

    def publish(self, wp_list):
        msg = Lane()
        msg.waypoints = wp_list
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.msg_seq_num
        self.msg_seq_num += 1
        self.final_waypoints_pub.publish(msg)

    # Flexible to work with queue or base waypoint list
    def get_next_waypoint(self, waypoint_list):
        nearest_idx = None
        if waypoint_list is None:
            return nearest_idx
        num_points = len(waypoint_list)
        lowest_dist = 9999999.9
        iterations = 0
        for idx, wp in enumerate(waypoint_list):
            iterations += 1
            delta = self.distance2(wp.pose.pose.position, self.current_pose.position)
            if delta < lowest_dist:
                lowest_dist = delta
                nearest_idx = idx
            if lowest_dist < 1.:
                break
        nearest_wp = waypoint_list[nearest_idx]
        if not self.is_waypoint_positive(nearest_wp.pose.pose, self.current_pose): # Is it in front?
            #nearest_idx = (nearest_idx + 1) % num_points
            nearest_idx += 1
        return nearest_idx

    def is_waypoint_positive(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x # convert to vehicle coords
        dy = pose1.position.y - pose2.position.y
        o = pose2.orientation
        _,_,t = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        lx = math.cos(-t)*dx - math.sin(-t)*dy
        return lx > 0.0

    # Distance between two points: p1 and p2 are position data structures
    def distance2(self, p1, p2):
        x, y = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(x*x + y*y)

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        
        # Check for proximity to destination
        if self.destination is not None:
            sidx = self.destination
            distance_to_destination = self.distance2(self.current_pose.position, self.base_waypoints[sidx].pose.pose.position)
            # and if we are within stopping distance...
            if distance_to_destination < self.velocity_drop:
                self.traffic_sub.unregister()         # unsubscribe from traffic light messages
                self.stop_waypoint = sidx             # set up an imaginary red traffic light
                self.halt = True
                rospy.loginfo("Destination waypoint acquired...begin slowing")
                self.replan = True
                self.destination = None

    # The following callback is latched (called once)
    def waypoints_cb(self, waypoints):
        self.base_waypoints = self.filterWaypoints(waypoints)
        self.num_base_wp = len(self.base_waypoints)  # the number of points in the base list
        self.destination = self.num_base_wp - 1
        
        # Acquire the default velocity from the waypoint loader
        self.VELOCITY_MAX = self.base_waypoints[self.num_base_wp/2].twist.twist.linear.x
        rospy.loginfo("Velocity max is: %.2f", self.VELOCITY_MAX)
        
        # If simulator, use longer queue
        if self.VELOCITY_MAX > 3.0:
            self.LOOKAHEAD_WPS = 100

        # Compute a safe stopping distance
        self.velocity_drop = self.VELOCITY_MAX * self.VELOCITY_MAX / 2.
        rospy.loginfo("Stopping distance is: %.2f", self.velocity_drop)
        
        #debug: set a closer destination waypoint to test end-of-track-halt condition
        #self.destination = 577

    def filterWaypoints(self, wp):
        if wp.waypoints[0].pose.pose.position.x == 10.4062:
            waypoints = []
            path = rospy.get_param('~path')
            if not os.path.isfile(path):
                return wp.waypoints
            with open(path) as wfile:
                reader = csv.DictReader(wfile, ['x','y','z','yaw'])
                for wp in reader:
                    p = Waypoint()
                    p.pose.pose.position.x = float(wp['x'])
                    p.pose.pose.position.y = float(wp['y'])
                    p.pose.pose.position.z = float(wp['z'])
                    q = tf.transformations.quaternion_from_euler(0., 0., float(wp['yaw']))
                    p.pose.pose.orientation = Quaternion(*q)
                    p.twist.twist.linear.x = 2.7777778
                    waypoints.append(p)
            rospy.loginfo("Corrected waypoints loaded")
            return waypoints
        return wp.waypoints

    def traffic_cb(self, msg):
        #self.stop_waypoint = msg.data if msg.data >= 0 else None
        # Halt before the stop line, otherwise we enter the intersection
        self.stop_waypoint = msg.data-3 if msg.data >= 0 else None
        
        # traffic light transitions cause replanning in update loop
        if self.stop_waypoint != self.prev_state:
            self.replan = True
            self.prev_state = self.stop_waypoint

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
