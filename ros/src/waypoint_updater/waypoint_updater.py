#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

import matplotlib.pyplot as plt
import matplotlib.patches as patches

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.TrG2V = 0 #transformation globe 2 vehicle
        self.FirstTime = True

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        t1 = tf.transformations.translation_matrix((px, py, pz))
        t2 = tf.transformations.quaternion_matrix([qx,qy,qz,qw])
        Tg2v = tf.transformations.concatenate_matrices(t2,t1)
        self.TrG2V = tf.transformations.inverse_matrix(Tg2v)
        #rospy.loginfo(self.TrG2V)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #waypointsInVehicleFrame = Lane()
        #for waypoint in waypoints.waypoints:
        #    pass
        if self.FirstTime is True:
            self.FirstTime = False
            x, y, z, orients = [], [], [], []
            for waypoint in waypoints.waypoints:
                x.append(waypoint.pose.pose.position.x)
                y.append(waypoint.pose.pose.position.y)
                z.append(waypoint.pose.pose.position.z)
                ox = waypoint.pose.pose.orientation.x
                oy = waypoint.pose.pose.orientation.y
                oz = waypoint.pose.pose.orientation.z
                ow = waypoint.pose.pose.orientation.w
                #rospy.loginfo([ox,oy,oz,ow])
                T = tf.transformations.quaternion_matrix([ox,oy,oz,ow])
                orients.append(50.0*(T[0:2,0]))
                rospy.loginfo(tf.transformations.euler_from_quaternion([ox,oy,oz,ow]))
            plt.plot(x,y)
            ax = plt.gca()
            for i in range(0,len(x),100):
                ax.add_patch(patches.Arrow(x[i],y[i],orients[i][0],orients[i][1],width=10))
                #rospy.loginfo([x[i],y[i],orients[i][0],orients[i][1]])
            plt.axis('equal')
            plt.show()
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
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
