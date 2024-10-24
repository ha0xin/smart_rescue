#!/usr/bin/env python
import rospy
import networkx as nx


class GlobalMotionPlanner:
    def __init__(self):
        rospy.init_node("global_motion_planner_node")

        self.roadmap_topic = rospy.get_param("~roadmap_topic", "/roadmap")
        rospy.Subscriber(self.roadmap, RoadMap, self.roadmap_callback, queue_size=1)


        while not rospy.is_shutdown():
            self.roadmap = self.gen_roadmap()

            path = self.get_path()