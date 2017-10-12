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

MAX_SPEED = 10
LOOKAHEAD_WPS = int(MAX_SPEED * 2) # Number of waypoints to publish. (Can be changed.)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
	rospy.loginfo("Hajime!!!")

	# current coords of car
        self.x_current = None
        self.y_current = None
        self.theta_current = None
        
        # max target velocity, constant
        self.max_velocity = MAX_SPEED * 0.95 #mpss
        
        # list of base waypoints, set only once
        self.base_waypoints = None
        self.destination = None
        
        # list of previous waypoints, gets updated after each loop
        self.previous_waypoints = None
        # list of next waypoints, keeping it persisted to prevent too much memory usage
        self.next_waypoints = None
        
        # stop line for the nearest red/yellow light, and if green then None
        self.red_light_waypoint = None
        
        # current velocity of car, set by subscriber
        self.current_velocity = None
        
        # current state of the car: go/stop
        self.status = None
        
        # message sequence number
        self.msg_seq_num = 0
        
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

            print "Current position: (", x_current, " ", y_current, ")"
            
            # we have base waypoints and current position
            if (self.base_waypoints is not None) and (self.x_current is not None and (self.red_light_waypoint is not None)):
                
                # get the index of the closest waypoint
                nearest_waypoint_index = assist.nearest_waypoint(self.base_waypoints, x_current, y_current, theta_current)
                
                # get the closest waypoint object
                nearest_waypoint = self.base_waypoints.waypoints[nearest_waypoint_index]
                
                rospy.loginfo("Got the nearest waypoint: x=%.2f, y=%.2f", nearest_waypoint.pose.pose.position.x, nearest_waypoint.pose.pose.position.y)
                
                # Store next LOOKAHEAD waypoints into next_waypoints, to be modified later.
                self.next_waypoints = self.base_waypoints.waypoints[nearest_waypoint_index:(nearest_waypoint_index + LOOKAHEAD_WPS)]
                
                stop = self.stop_point(self.next_waypoints)
                
                current_velocity = self.current_velocity
                if current_velocity > self.max_velocity:
                    current_velocity = self.max_velocity
                
                # MUST STOP
                if (stop is not None):
                    rospy.loginfo("Red traffic light detected")
                    target_velocity = 0.0
                    
                    # Must use new waypoints
                    if (self.status == "go" or self.status is None):
                        [n1, n2, delta_v] = self.steps(current_velocity, target_velocity, stop)
                        n3 = LOOKAHEAD_WPS - n1 - n2
                        i = 0
                        velocity = current_velocity
                        while (i < n1):
                            velocity += delta_v
                            self.next_waypoints[i].twist.twist.linear.x = velocity
                            rospy.loginfo("Velocity set to %.2f", velocity)
                            i = i + 1
                        rospy.loginfo("Speeding up for %d iterations", n1)
                        while (i <= LOOKAHEAD_WPS - n2):
                            self.next_waypoints[i].twist.twist.linear.x = velocity
                            rospy.loginfo("Velocity set to %.2f", velocity)
                            i = i + 1
                        rospy.loginfo("Staying at same speed for %d iterations", n3)
                        while (i < LOOKAHEAD_WPS):
                            velocity -= delta_v
                            if velocity < 0.0:
                                velocity = 0.0
                            self.next_waypoints[i].twist.twist.linear.x = velocity
                            rospy.loginfo("Velocity set to %.2f", velocity)
                            i = i + 1
                        rospy.loginfo("Slowing down for %d iterations", n2)
                    
                    # Reuse previous waypoints
                    elif (self.status == "stop"):
                        previous_limit = self.find_prior(nearest_waypoint)
                        rospy.loginfo("Last waypoint in previous waypoint list is %d", previous_limit)
                        i = 0
                        while (i < LOOKAHEAD_WPS):
                            if i <= previous_limit:
                                self.next_waypoints[i].twist.twist.linear.x = self.previous_waypoints[i].twist.twist.linear.x
                            else:
                                self.next_waypoints[i].twist.twist.linear.x = 0.0
                                rospy.loginfo("Setting velocity to zero")
                            i = i + 1
                        rospy.loginfo("Reusing previous waypoints")
                        
                    self.status = "stop"

                # KEEP GOING
                elif (stop is None):
                    rospy.loginfo("Red traffic light NOT detected")
                    target_velocity = self.max_velocity
                    
                    # Reset and use new waypoints
                    if (self.status == "stop" or self.status is None):
                        [n1, n2, delta_v] = self.steps(current_velocity, target_velocity, LOOKAHEAD_WPS)
                        rospy.loginfo("n1: %d, n2: %d, delta_v: %.2f", n1, n2, delta_v)
                        n3 = LOOKAHEAD_WPS - n1 - n2
                        i = 0
                        velocity = current_velocity
                        while (i < n1):
                            velocity += delta_v
                            self.next_waypoints[i].twist.twist.linear.x = velocity
                            rospy.loginfo("Velocity set to %.2f", velocity)
                            i = i + 1
                        while (i < LOOKAHEAD_WPS - n2):  # had to change to < rather than <=
                            self.next_waypoints[i].twist.twist.linear.x = velocity
                            rospy.loginfo("Velocity set to %.2f", velocity)
                            i = i + 1
                        rospy.loginfo("Staying at same speed for %d iterations", n3)
                        while (i < LOOKAHEAD_WPS):
                            velocity -= delta_v
                            if velocity < 0.0:
                                velocity = 0.0
                            self.next_waypoints[i].twist.twist.linear.x = velocity
                            rospy.loginfo("Velocity set to %.2f", velocity)
                            i = i + 1
                        rospy.loginfo("Slowing down for %d iterations", n2)
                    
                    # Reuse previous waypoints
                    elif (self.status == "go"):
                        previous_limit = self.find_prior(nearest_waypoint)
                        rospy.loginfo("Last waypoint in previous waypoint list is %d", previous_limit)
                        i = 0
                        while (i < LOOKAHEAD_WPS):
                            if i <= previous_limit:
                                self.next_waypoints[i].twist.twist.linear.x = self.previous_waypoints[i].twist.twist.linear.x
                            else:
                                self.next_waypoints[i].twist.twist.linear.x = target_velocity
                            i = i + 1
                        rospy.loginfo("Reusing previous waypoints")
                    self.status = "go"
                
                # Velocities calculated, now let's log and publish
                self.previous_waypoints = self.next_waypoints # ---- Is this working?
                
                for i in range(len(self.previous_waypoints)):
                    rospy.loginfo("Prev velocity is %.2f", self.previous_waypoints[i].twist.twist.linear.x)
                
                # Publish!
                rospy.loginfo("Publishing points\n")
                for i in range(len(self.next_waypoints)):
                    rospy.loginfo("Velocity is %.2f", self.next_waypoints[i].twist.twist.linear.x)
                self.publish(self.next_waypoints)
                
            # --- if ends, still in while loop ----
            rate.sleep()
        rospy.loginfo("Shutdown.")

    def publish(self, wp_list):
        msg = Lane()
        msg.waypoints = wp_list
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.msg_seq_num
        self.msg_seq_num += 1
        self.final_waypoints_pub.publish(msg)

    def get_current_position(self, msg):
        self.x_current = msg.pose.position.x
        self.y_current = msg.pose.position.y
        o = msg.pose.orientation
        quat = [o.x, o.y, o.z, o.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.theta_current = euler[2]
	    #rospy.loginfo("Current position received")

    def get_base_waypoints(self, waypoints):
        self.base_waypoints = waypoints
	rospy.loginfo("Base waypoints set. First waypoint:")
        self.destination = waypoints.waypoints[-1]
        rospy.loginfo("x=%.2f, y=%.2f \n", waypoints.waypoints[0].pose.pose.position.x, waypoints.waypoints[0].pose.pose.position.y)

    def callback_traffic(self, msg):
        self.red_light_waypoint = msg.data-2 if msg.data >= 0 else None

    def get_current_velocity(self, msg):
        self.current_velocity = msg.twist.linear.x
        #rospy.loginfo("Current velocity set")

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
    
    # Checks if red light OR destination is visible
    # Gets index of the stop waypoint in current lookahead
    # Returns index or None
    # Arguments: direct list of LOOKAHEAD waypoints
    def stop_point(self, waypoints):
        found = False
        stop_index = None
        i = 0
        num_points = len(waypoints)
        while(i<num_points and not found):
            if ((waypoints[i].pose.pose.position ==
                 self.base_waypoints.waypoints[self.red_light_waypoint].pose.pose.position) 
                or (waypoints[i].pose.pose.position == self.destination.pose.pose.position)):
                found = True
                stop_index = i
            else:
                i = i + 1
        return stop_index

    def find_prior(self, nearest_waypoint):
        i = 0
        length = len(self.previous_waypoints)
        while (i < length):
            if (self.previous_waypoints[i].pose.pose.position == nearest_waypoint.pose.pose.position):
                return i
            i = i + 1
        return i
            
    # Returns how many steps it will take to safely accelerate/decelerate
    def steps(self, current_velocity, target_velocity, stop):
        delta_v = 0.8
        max_velocity = 10.0
        # Number of waypoints for acceleration earlier
        n1 = int(abs(max_velocity - current_velocity) / delta_v)
        # Number of waypoints for deceleration later
        n2 = stop
        delta = 0.1
        while ((n1 + n2 > stop) or (max_velocity - (n2 * delta_v) < 0.0)):
            max_velocity -= delta
            n1 = int(abs(max_velocity - current_velocity) / delta_v)
            n2 = int(abs(target_velocity - max_velocity) / delta_v)
        return [n1, n2, delta_v]


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
