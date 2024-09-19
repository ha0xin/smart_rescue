#!/usr/bin/env python
import rospy
import tf
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

class TaskPlannerNode:
    def __init__(self):
        rospy.init_node("task_planner_node")
        rospy.loginfo("Task planner node started")
        self.listener = tf.TransformListener()

        # Allow the listener some time to buffer data
        rospy.sleep(0.2)

        self.rate = rospy.Rate(5.0)
        self.robot_num = rospy.get_param("~robot_num", 2)
        rospy.loginfo("Calculating distances for %d robots", self.robot_num)

        self.marker_sub = rospy.Subscriber("/centroid_markers", MarkerArray, self.marker_callback)
        self.goal_pub = rospy.Publisher("robot_0/move_base_simple/goal", PoseStamped, queue_size=10)
        self.nearest_marker = None

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.calculate_all_frame_distances()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Task planner node shutting down")
        except Exception as e:
            rospy.logerr("Unexpected error: %s", str(e))

    def calculate_all_frame_distances(self):
        for i in range(1, self.robot_num):
            frame_id_2 = "robot_%d/base_link" % i
            self.calculate_frame_origin_distance("robot_0/base_link", frame_id_2)

    def calculate_frame_origin_distance(self, frame_id_1: str, frame_id_2: str):
        try:
            (trans, rot) = self.listener.lookupTransform(frame_id_1, frame_id_2, rospy.Time(0))
            distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
            # rospy.loginfo("Distance between %s and %s: %.2f meters" % (frame_id_1, frame_id_2, distance))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Transform lookup failed between %s and %s: %s", frame_id_1, frame_id_2, str(e))

    def marker_callback(self, marker_array: MarkerArray):
        try:
            (robot_trans, robot_rot) = self.listener.lookupTransform("map", "robot_0/base_link", rospy.Time(0))
            min_distance = float('inf')
            nearest_marker = None

            for marker in marker_array.markers:
                marker: Marker
                distance = math.sqrt(
                    (marker.pose.position.x - robot_trans[0]) ** 2 +
                    (marker.pose.position.y - robot_trans[1]) ** 2 +
                    (marker.pose.position.z - robot_trans[2]) ** 2
                )
                if distance < min_distance:
                    min_distance = distance
                    nearest_marker = marker

            if nearest_marker:
                rospy.loginfo("Nearest marker ID: %d, Distance: %.2f meters", nearest_marker.id, min_distance)
                self.nearest_marker = nearest_marker

                self.explore()
                self.marker_sub.unregister() # explore once

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Transform lookup failed for robot_0/base_link: %s", str(e))

    def explore(self):
        if self.nearest_marker:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose = self.nearest_marker.pose
            goal.pose.position.z = 0
            self.goal_pub.publish(goal)
            rospy.loginfo("Exploring marker ID: %d", self.nearest_marker.id)
        else:
            rospy.logwarn("No nearest marker found")

if __name__ == "__main__":
    task_planner = TaskPlannerNode()
    task_planner.spin()