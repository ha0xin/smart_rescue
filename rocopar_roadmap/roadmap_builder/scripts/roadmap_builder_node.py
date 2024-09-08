import rospy

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from tuw_multi_robot_msgs.msg import Graph

from roadmap_builder.srv import GenVoronoi, GenVoronoiRequest
from roadmap_builder.srv import GetFrontiers, GetFrontiersRequest

import math
import networkx as nx
import matplotlib.pyplot as plt


def create_point_marker(ns, id, point, color, scale):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.05
    marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.text = 'yes'
    return marker

def create_line_marker(ns, id, edge, color, scale):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    point_f = Point(edge[0][0], edge[0][1], 0.05)
    point_t = Point(edge[1][0], edge[1][1], 0.05)
    marker.points = [point_f, point_t]
    return marker


class RoadmapBuilderNode():
    def __init__(self):
        rospy.init_node("roadmap_builder_node")

        # Variables
        self.oc_map = OccupancyGrid()
        self.map_topic = rospy.get_param("~map_topic", "/map")

        # ROS interfaces
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)

        self.voronoi_gen_client = rospy.ServiceProxy("/gen_voronoi", GenVoronoi)
        self.frontier_get_client = rospy.ServiceProxy("/get_frontiers", GetFrontiers)

        self.voronoi_pub = rospy.Publisher("voronoi_map", Graph, queue_size=1)
        self.roadmap_pub = rospy.Publisher('roadmap_vis', Marker, queue_size=100)

        # Wait for Ready
        rospy.loginfo("Waiting for publishers and servers available.")
        self.oc_map = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        rospy.wait_for_service("/gen_voronoi")
        rospy.wait_for_service("/get_frontiers")
        rospy.loginfo("Publishers and servers ready.")

        while not rospy.is_shutdown():
            # Pull a request for voronoi map
            rospy.loginfo("Pull a request for voronoi map.")
            voronoi_res = self.voronoi_gen_client.call(GenVoronoiRequest(self.oc_map))

            # Process of voronoi map
            roadmap, _ = self.build_roadmap(voronoi_res.voronoi_map)
            
            # Pull a request for frontier points
            rospy.loginfo("Pull a request for frontier points.")
            frontier_res = self.frontier_get_client.call(GetFrontiersRequest(self.oc_map))
            self.voronoi_pub.publish(voronoi_res.voronoi_map)

            # Add frontiers into roadmap
            for frontier_ in frontier_res.frontiers:
                centroid_ = frontier_.centroid
                near_node, _ = self.get_near_node(roadmap, centroid_)
                roadmap.add_edge((centroid_.x, centroid_.y), 
                                (near_node[0], near_node[1]))
                roadmap.nodes[(centroid_.x, centroid_.y)]["node_type"] = "frontier"
                print(near_node)

            # Display roadmap
            # self.display_roadmap(roadmap)
            ## publish the visualization of the roadmap
            ## publish nodes
            for i, node in enumerate(roadmap.nodes()):
                if roadmap.nodes[node]["node_type"] == "point":
                    blue_index = [1.0, 0.0, 0.0, 1.0]
                    marker = create_point_marker("nodes", i, node, color=blue_index, scale=0.1)
                elif roadmap.nodes[node]["node_type"] == "frontier":
                    red_index = [1.0, 1.0, 0.0, 0.0]
                    marker = create_point_marker("frontiers", i, node, color=red_index, scale=0.1)
                self.roadmap_pub.publish(marker)
            
            ## publish edges
            for i, edge in enumerate(roadmap.edges()):
                blue_index = [1.0, 0.0, 0.0, 1.0]
                marker = create_line_marker("frontiers", i, edge, color=blue_index, scale=0.05)
                self.roadmap_pub.publish(marker)

            rospy.sleep(1)

    def map_callback(self, msg):
        self.oc_map = msg

    def build_roadmap(self, v_map):
        origin_ = v_map.origin.position  # Type: geometery_msgs/Point

        roadmap_ = nx.Graph()

        # Add edges and nodes
        for seg in v_map.vertices:
            point_f = (round(seg.path[0].x + origin_.x, 2), round(seg.path[0].y + origin_.y, 2))
            point_t = (round(seg.path[-1].x + origin_.x, 2), round(seg.path[-1].y + origin_.y, 2))
            roadmap_.add_edge(point_f, point_t)
            roadmap_.nodes[point_f]["node_type"] = "point"
            roadmap_.nodes[point_t]["node_type"] = "point"
        
        # Cut broken branch
        # before_cut_ = roadmap_.number_of_nodes()
        # self.display_roadmap(roadmap_)
        roadmap_ = roadmap_.subgraph(max(nx.connected_components(roadmap_), key=len))
        roadmap_ = nx.Graph(roadmap_)
        # after_cut_ = roadmap_.number_of_nodes()
        # self.display_roadmap(roadmap_)
        # print("before cut: %d, after cut: %d"%(before_cut_,after_cut_))
        return roadmap_, origin_

    def get_near_node(self, roadmap, point):
        """get the nearest node in roadmap to given point

        Args:
            roadmap (networkx.Graph): roadmap in networkx.Graph form
            point (geometery/Point): the given point
        
        Return:
            ((near_point_x, near_point_y), min_dis)
        """
        nodes_ = list(roadmap.nodes)
        min_index_ = 0
        min_dis_ = math.inf
        for i in range(len(nodes_)):
            dis_ = math.sqrt((point.x - nodes_[i][0])**2 + (point.y - nodes_[i][1])**2)
            if dis_ < min_dis_:
                min_dis_ = dis_
                min_index_ = i
        
        return nodes_[min_index_], min_dis_

    def display_roadmap(self, roadmap):
        option = {
            "with_labels": True, 
            "font_weight": 'bold', 
            "node_color": ["blue" if roadmap.nodes[n]["node_type"]=="raw" else "red" for n in roadmap.nodes],
            "pos": {n: n for n in list(roadmap.nodes)}
            }
        nx.draw(roadmap, **option)
        plt.show()



if __name__ == "__main__":
    rbn = RoadmapBuilderNode()