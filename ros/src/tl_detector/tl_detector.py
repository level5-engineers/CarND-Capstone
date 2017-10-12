#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 6
CLASSIFIER_ENABLED = True

from PIL import Image as PIL_Image

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

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
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.camera_image = None
        self.seq = 9174

        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
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

    def distance(self, pos1, pos2):
	    return math.sqrt((pos1.position.x - pos2.position.x)**2 + (pos1.position.y - pos2.position.y)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        
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


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.RED

        # fix camera encoding
        if hasattr(self.camera_image, 'encoding'):
            if self.camera_image.encoding == '8UC3':
                self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

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
                
                # if the traffic light is within 75 meters, then attempt to classify
                if (stop_pos is not None) and (self.distance(self.pose.pose, stop_pos.pose) < 75.):
                    #rospy.loginfo("stop_pos x: %.2f, stop_waypoint: %d", stop_pos.pose.position.x, stop_waypoint)
                    state = TrafficLight.UNKNOWN
                    if CLASSIFIER_ENABLED:
                        state = self.get_light_state(None)
                        if self.lights is not None:
                            stateTruth = TrafficLight.UNKNOWN
                            for light in self.lights:
                                # This section uses only /vehicle/traffic_lights
                                if self.distance(light.pose.pose, stop_pos.pose) < 30.:
                                    stateTruth = light.state
                            if state != stateTruth:
                                state = stateTruth
                                print "Classifier mismatch...using correct state: ", state
                                #self.saveImage(self.camera_image, state)
                    else:
                        for light in self.lights:
                            # This section uses only /vehicle/traffic_lights
                            if self.distance(light.pose.pose, stop_pos.pose) < 30.:
                                state = light.state
                    return stop_waypoint, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def saveImage(self, img, state):
        if self.seq%2==0:
            # fix camera encoding
            if hasattr(img, 'encoding'):
                if img.encoding == '8UC3':
                    img.encoding = "rgb8"
            else:
                img.encoding = 'rgb8'
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")

            image_data = cv2.resize(img, (224,224))
            #image_data = (image_data - 128.)/128.
            #image_data = np.reshape(image_data, (1,224,224,3))
            img= PIL_Image.fromarray(image_data, 'RGB')
            if state == TrafficLight.RED:
                img.save('/home/student/data/red/out'+str(self.seq).zfill(5)+'.png', 'PNG')
            elif state == TrafficLight.YELLOW:
                img.save('/home/student/data/yellow/out'+str(self.seq).zfill(5)+'.png', 'PNG')
            elif state == TrafficLight.GREEN:
                img.save('/home/student/data/green/out'+str(self.seq).zfill(5)+'.png', 'PNG')
            else:
                img.save('/home/student/data/unknown/out'+str(self.seq).zfill(5)+'.png', 'PNG')
        self.seq += 1


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
