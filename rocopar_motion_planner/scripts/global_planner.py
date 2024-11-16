#!/usr/bin/env python
import actionlib
import matplotlib.pyplot as plt
import networkx as nx
import rospy
import tf
import tf2_ros
import mbf_msgs.msg as mbf_msgs
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from roadmap_builder.msg import Edge, Node, RoadMap


class GlobalMotionPlanner:
    def __init__(self):
        rospy.init_node("global_motion_planner_node")

        self.roadmap_topic = rospy.get_param("~roadmap_topic", "/roadmap")
        self.roadmap = nx.Graph()

        # subscribers
        rospy.Subscriber(
            self.roadmap_topic, RoadMap, self.roadmap_callback, queue_size=1
        )

        rospy.Subscriber(
            "motion_planner/goal", PoseStamped, self.goal_callback, queue_size=1
        )
        # publishers
        self.path_pub = rospy.Publisher(
            "motion_planner/global/path_vis", Path, queue_size=1
        )
        # tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # action client
        self.exe_path_client = actionlib.SimpleActionClient(
            "move_base_flex/exe_path", mbf_msgs.ExePathAction
        )
        self.exe_path_client.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected to Move Base Flex ExePath server!")

        while not rospy.is_shutdown():
            rospy.spin()
        #     self.roadmap_graph = self.gen_roadmap()

        #     path = self.get_path()

    def roadmap_callback(self, msg: RoadMap):
        self.roadmap = msg
        # rospy.loginfo("Roadmap received")

    def goal_callback(self, goal: PoseStamped):
        rospy.loginfo("Goal received: %s", goal)
        self.goal = goal
        roadmap_graph = self.gen_roadmap()
        # output_path = "/home/haoxin/ws/mwe_ws/roadmap.png"
        # self.save_roadmap_image(roadmap_graph, output_path)

        start_node = self.find_nearest_node(self.my_position())
        end_node = self.find_nearest_node(self.goal.pose.position)
        rospy.loginfo("Start node: %s", "\n" + str(start_node))
        rospy.loginfo("End node: %s", "\n" + str(end_node))

        point_list = self.shortest_path(roadmap_graph, start_node, end_node)
        rospy.loginfo("Path: %s", point_list)

        self.draw_path(start_node, end_node, point_list)

    def save_roadmap_image(self, roadmap_graph, output_path):
        plt.figure(figsize=(8, 6))
        pos = {node: node for node in roadmap_graph.nodes()}  # 使用节点的坐标作为位置
        nx.draw(
            roadmap_graph,
            pos,
            with_labels=True,
            node_size=300,
            node_color="skyblue",
            font_size=10,
            font_weight="bold",
            edge_color="gray",
        )
        plt.savefig(output_path)
        plt.close()
        rospy.loginfo(f"Roadmap image saved to {output_path}")

    def my_position(self):
        trans: TransformStamped = self.tfBuffer.lookup_transform(
            "map", "robot_0/base_link", rospy.Time(0)
        )
        point = Point()
        point.x = trans.transform.translation.x
        point.y = trans.transform.translation.y
        point.z = trans.transform.translation.z
        return point

    def gen_roadmap(self) -> nx.Graph:
        G = nx.Graph()
        for node in self.roadmap.nodes:
            node: Node
            G.add_node((node.point.x, node.point.y))
        for edge in self.roadmap.edges:
            edge: Edge
            G.add_edge(
                (edge.source.x, edge.source.y),
                (edge.target.x, edge.target.y),
                weight=edge.cost,
            )

        return G

    def find_nearest_node(self, pos: Point) -> Node:
        min_dist = float("inf")
        nearest_node = None
        for node in self.roadmap.nodes:
            node: Node
            dist = (pos.x - node.point.x) ** 2 + (pos.y - node.point.y) ** 2
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def shortest_path(
        self, roadmap_graph: nx.Graph, start_node: Node, goal_node: Node
    ) -> list:
        return nx.shortest_path(
            roadmap_graph,
            (start_node.point.x, start_node.point.y),
            (goal_node.point.x, goal_node.point.y),
        )

    def draw_path(self, start_node: Node, end_node: Node, point_list: list):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = rospy.Time.now()
        start_pose.pose.position.x = start_node.point.x
        start_pose.pose.position.y = start_node.point.y
        start_pose.pose.orientation.w = 1
        path.poses.append(start_pose)

        for point in point_list:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1
            path.poses.append(pose)

        end_pose = PoseStamped()
        end_pose.header.frame_id = "map"
        end_pose.header.stamp = rospy.Time.now()
        end_pose.pose.position.x = end_node.point.x
        end_pose.pose.position.y = end_node.point.y
        end_pose.pose.orientation.w = 1
        path.poses.append(end_pose)

        self.path_pub.publish(path)
        goal = mbf_msgs.ExePathGoal()
        goal.path = path
        goal.tolerance_from_action = True
        goal.dist_tolerance = 0.5
        goal.angle_tolerance = 3.14/18
        self.exe_path_client.send_goal(goal)


    def interpolate(self, path: list) -> list:
        pass


if __name__ == "__main__":
    GlobalMotionPlanner()
