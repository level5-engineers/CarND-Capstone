#!/usr/bin/env python
import rospy
import os
import csv
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 1 # set back to 3
CLASSIFIER_ENABLED = True
PUBLISHING_RATE = 6 # Publishing frequency (Hz)    set back to 10

from PIL import Image as PIL_Image

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.simulator = True if rospy.get_param('~sim') == 1 else False

        self.pose             = None
        self.waypoints        = None
        self.camera_image     = None
        self.lights           = []
        self.bridge           = CvBridge()
        self.light_classifier = TLClassifier(self.simulator)
        self.listener         = tf.TransformListener()
        self.state            = TrafficLight.UNKNOWN
        self.last_state       = TrafficLight.UNKNOWN
        self.last_wp          = -1
        self.state_count      = 0
        self.camera_image     = None
        self.seq              = 0
        self.count            = 0
        self.misscount        = 0.
        self.totcount         = 0.
        self.sight            = 75.   # cautionary distance
        self.wpAcquired       = False
        self.lastCrop         = None
        self.imageAcquired    = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config   = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        #rospy.spin()
        # Operations loop to identify red lights in the incoming camera image and publishes
        # the index of the waypoint closest to the red light's stop line to /traffic_waypoint
        rate = rospy.Rate(PUBLISHING_RATE)
        
        while not rospy.is_shutdown():
            if self.imageAcquired:
                light_wp, state = self.process_traffic_lights()
                '''
                Publish upcoming red lights at PUBLISHING_RATE frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times until we start using it. Otherwise the previous stable
                state is used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
                    self.last_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                self.state_count += 1
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        if not self.wpAcquired:
            self.wpAcquired = True
            self.waypoints = self.filterWaypoints(msg)
            if not self.simulator:
                self.sight = 16.  # adjust cautionary distance for site test
            print "cautionary distance", self.sight
            print self.waypoints[20].twist.twist.linear.x

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Capture the inbound camera image
        Args:
            msg (Image): image from car-mounted camera
        """
        self.imageAcquired = True
        self.camera_image = msg

    def distance(self, pos1, pos2):
        return math.sqrt((pos1.position.x - pos2.position.x)**2 + (pos1.position.y - pos2.position.y)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        index = -1 #Return if waypoints are empty
        if self.waypoints is None:
            return index
        
        lowDistance = 0
        #rospy.loginfo("len(wp): %d", len(self.waypoints))
        for i in range(len(self.waypoints)):
            distance = self.distance(pose, self.waypoints[i].pose.pose)
            
            if index == -1 or distance < lowDistance:
                index = i
                lowDistance = distance
            
        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.imageAcquired):
            self.prev_light_loc = None
            return TrafficLight.RED

        # fix camera encoding
        if hasattr(self.camera_image, 'encoding'):
            if self.camera_image.encoding == '8UC3':
                self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # get classification (simulator classifies whole image)
        if self.simulator:
            return self.light_classifier.get_classification(cv_image)

        # get classification (test site detects boxes using SSD and classifies with OpenCV)
        # if previous location is not known, scan most of the image
        if self.lastCrop is None:
            gostop, found, location = self.light_classifier.get_classification( cv_image[0:500, 50:50+700] )
            if found:
                # check y extents
                if location[0] < 150:
                    top = 0
                    bot = 300
                else:
                    top = location[0]-150
                    if location[0] > 350:
                        top = 200
                        bot = 500
                    else:
                        bot = location[0]+150

                # check x extents (remember, offset by 50)
                if location[1]+50 < 150:
                    left = 50
                    right = 350
                else:
                    left = location[1]+50-150
                    if location[1]+50 > 600:
                        left = 450
                        right = 750
                    else:
                        right = location[1]+50+150
                self.lastCrop = (top,bot, left,right)
                #print "first", self.lastCrop
            # (no else) TL not found, next cycle will be a complete search again
        # otherwise, use last known location as crop starting point
        else:
            (top,bot, left,right) = self.lastCrop
            gostop, found, location = self.light_classifier.get_classification( cv_image[top:bot, left:right] )
            if found: # determine crop for next cycle
                # check y extents, offset by top
                otop, oleft = top, left
                if location[0]+otop < 150:
                    top = 0
                    bot = 300
                else:
                    top = location[0]+otop-150
                    if location[0]+otop > 350:
                        top = 200
                        bot = 500
                    else:
                        bot = location[0]+otop+150
                
                # check x extents, offset by left
                if location[1]+oleft+50 < 200:
                    left = 50
                    right = 350
                else:
                    left = location[1]+oleft+50-150
                    if location[1]+oleft+50 > 600:
                        left = 450
                        right = 750
                    else:
                        right = location[1]+oleft+50+150
                
                self.lastCrop = (top,bot, left,right)
                #print "next", self.lastCrop
            else:
                self.lastCrop = None
                #print "none"
        return gostop

    def get_nearest_stop_line(self, waypoint_start_index):
        stop_line = None
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        last_index = 99999
        
        for i in range(0, len(stop_line_positions)):
            stopline_pose = PoseStamped()
            stopline_pose.pose.position.x = float(stop_line_positions[i][0])
            stopline_pose.pose.position.y = float(stop_line_positions[i][1])
            index = self.get_closest_waypoint(stopline_pose.pose)
            if index > waypoint_start_index and index < last_index:
                last_index = index;
                stop_line = stopline_pose
        
        return stop_line, last_index

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #rospy.loginfo("car_position: %d", car_position)
            if car_position > 0:
                stop_pos, stop_waypoint = self.get_nearest_stop_line(car_position)
                
                if stop_pos is not None:
                    #rospy.loginfo("stop_pos x: %.2f, stop_waypoint: %d", stop_pos.pose.position.x, stop_waypoint)
                    state = TrafficLight.UNKNOWN
                    
                    # if the traffic light is within sight, then attempt to classify
                    if self.distance(self.pose.pose, stop_pos.pose) < self.sight:
                        
                        if CLASSIFIER_ENABLED:
                            state = self.get_light_state(None)
                            if self.lights is not None:
                                stateTruth = TrafficLight.UNKNOWN
                                for light in self.lights:
                                    # Get the ground truth from /vehicle/traffic_lights
                                    if self.distance(light.pose.pose, stop_pos.pose) < 30.:
                                        stateTruth = light.state
                                        break
                                if (((stateTruth is 4) or (stateTruth is 2)) and (state is 0)) or (((stateTruth is 0) or (stateTruth is 1)) and (state is 4)) :
                                    print "Classifier: ", state, " Truth: ", stateTruth
                                    state = stateTruth
                                    #self.saveImage(self.camera_image, stateTruth)
                                    self.misscount += 1.
                                self.totcount += 1.
                                #print "mismatch%: ", self.misscount / self.totcount
                                #self.saveImage(self.camera_image, stateTruth)
                        else:
                            for light in self.lights:
                                # This section uses only /vehicle/traffic_lights
                                if self.distance(light.pose.pose, stop_pos.pose) < 30.:
                                    state = light.state
                                    #rospy.loginfo("light state: %d, x: %.2f, y: %.2f", state, self.pose.pose.position.x, self.pose.pose.position.y)
                                    #rospy.loginfo("dist tl to stop: %.2f", self.distance(light.pose.pose, stop_pos.pose))
                                    break
                        return stop_waypoint, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def saveImage(self, img, state):
        #if self.count%2==0:
            # fix camera encoding
        if hasattr(img, 'encoding'):
            if img.encoding == '8UC3':
                img.encoding = "rgb8"
        else:
            img.encoding = 'rgb8'
        img = self.bridge.imgmsg_to_cv2(img, "rgb8")

        image_data = cv2.resize(img, (224,224))
        img= PIL_Image.fromarray(image_data, 'RGB')
        #if state == TrafficLight.RED:
        #    img.save('/home/student/data/red/out'+str(self.seq).zfill(5)+'.png', 'PNG')
        #    self.seq += 1
        if state == TrafficLight.YELLOW:
            img.save('/home/student/data/yellow/out'+str(self.seq).zfill(5)+'.png', 'PNG')
            self.seq += 1
        elif state == TrafficLight.GREEN:
            img.save('/home/student/data/green/out'+str(self.seq).zfill(5)+'.png', 'PNG')
            self.seq += 1
        #else:
        #    img.save('/home/student/data/unknown/out'+str(self.seq).zfill(5)+'.png', 'PNG')
        #    self.seq += 1

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
            return waypoints
        return wp.waypoints

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
