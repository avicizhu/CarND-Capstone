#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import tf
import math
import time

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.tf_listener = tf.TransformListener()

        self.pose = None

        self.waypoints = None

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        if self.waypoints is None:
            return

        num_waypoints_in_list = len(self.waypoints.waypoints)

        # Gererate an empty lane to store the final_waypoints
        lane = Lane()
        lane.header.frame_id = self.waypoints.header.frame_id
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = []

        # Iterate through the complete set of waypoints until we found the closest
        #rospy.loginfo('Started at waypoint index: %s', self.prev_first_wpt_index)
        #start_time = time.time()
        first_wpt_index = -1
        min_wpt_distance = float('inf')

        for index, waypoint in enumerate(self.waypoints.waypoints):
            current_wpt_distance = self.distance(self.pose.pose.position, waypoint.pose.pose.position)
            if current_wpt_distance > 0 and current_wpt_distance < min_wpt_distance:
                min_wpt_distance = current_wpt_distance
                first_wpt_index = index

        self.waypoints.waypoints[first_wpt_index].pose.header.frame_id = self.waypoints.header.frame_id
        try:
            self.tf_listener.waitForTransform("base_link", "world", rospy.Time(0), rospy.Duration(0.05))
            transformed_waypoint = self.tf_listener.transformPose("base_link", self.waypoints.waypoints[first_wpt_index].pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
	    rospy.logwarn("fail %s", time.time())
            return

        if transformed_waypoint.pose.position.x <= 0.0:
            first_wpt_index += 1
            first_wpt_index %= num_waypoints_in_list

        planned_velocity = 10.0

        for num_wp in range(LOOKAHEAD_WPS):
                wp = Waypoint()
                wp.pose = self.waypoints.waypoints[(first_wpt_index + num_wp) % num_waypoints_in_list].pose
                wp.twist = self.waypoints.waypoints[(first_wpt_index + num_wp) % num_waypoints_in_list].twist

                wp.twist.twist.linear.x = planned_velocity
                wp.twist.twist.linear.y = 0.0
                wp.twist.twist.linear.z = 0.0

                wp.twist.twist.angular.x = 0.0
                wp.twist.twist.angular.y = 0.0
                wp.twist.twist.angular.z = 0.0
                lane.waypoints.append(wp)

        # finally, publish waypoints as modified on /final_waypoints topic
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint_timestamp = time.time()
        self.light_waypoint_index = msg.data
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

    def distance(self, pose1, pose2):
        return math.sqrt((pose1.x-pose2.x)**2 + (pose1.y-pose2.y)**2  + (pose1.z-pose2.z)**2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
