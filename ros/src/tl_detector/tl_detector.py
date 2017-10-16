#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

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

        self.loop()
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

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

    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():

            light_wp, state = self.process_traffic_lights()

            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

            rate.sleep()


    def distance(self, p1, p2):
        """
        Distance between two map coordinates copied from WaypointLoader class.
        """
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_wpt_distance = float('inf')
        closest_waypoint_ind = -1

        for index, waypoint in enumerate(self.waypoints.waypoints):
            current_wpt_distance = self.distance(self.waypoints.waypoints[index].pose.pose.position,pose.pose.position)
            if current_wpt_distance < min_wpt_distance:
                min_wpt_distance = current_wpt_distance
                closest_waypoint_ind = index

        return closest_waypoint_ind

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose is None or self.waypoints is None):
            return -1, None

        min_distance = float("inf")
        light_wp = -1

        #transform fast avoiding wait cycles
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("base_link","world", now, rospy.Duration(0.1))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn("fail to convert in tl_detector")
            return -1, TrafficLight.UNKNOWN

        wtl=PointStamped()
        wtl.header.frame_id = "/world"
        wtl.header.stamp =rospy.Time(0)
        wtl.point.z = 0

        for wp in range(len(stop_line_positions)):
            wtl.point.x = stop_line_positions[wp][0]
            wtl.point.y = stop_line_positions[wp][1]
            # Transform first waypoint to car coordinates
            ctl = self.listener.transformPoint("base_link",wtl)
            pose = PoseStamped()
            pose.pose.position.x = stop_line_positions[wp][0]
            pose.pose.position.y = stop_line_positions[wp][1]
            pose.pose.position.z = 0

            #only points ahead
            if ctl.point.x > 0 and ctl.point.x < min_distance and abs(ctl.point.y) < 10:
                min_distance = ctl.point.x
                light = pose

        #nothing ahead
        if light is None:
            return -1, TrafficLight.UNKNOWN
#        rospy.loginfo('stop line distance: %s pose %s', min_distance,(pose.pose.position.x,pose.pose.position.y))

        if min_distance < 50 and min_distance >=0:
            rospy.logwarn("getting close to a light")
            light_wp = self.get_closest_waypoint(light)
            state = self.get_light_state(light)
            return light_wp, state
        else:
            #self.waypoints = None
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
