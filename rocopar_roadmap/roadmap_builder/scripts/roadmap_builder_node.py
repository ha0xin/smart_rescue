import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from tuw_multi_robot_msgs.msg import Graph

from roadmap_builder.msg import Frontier, FrontierArray

from roadmap_builder.srv import gen_voronoi, gen_voronoiRequest
from roadmap_builder.srv import GetFrontiers, GetFrontiersRequest, GetFrontiersResponse

class RoadmapBuilder():
    def __init__(self):
        rospy.init_node("roadmap_builder_node")

        # Variables
        self.oc_map = OccupancyGrid()

        # ROS interfaces
        rospy.Subscriber("/merged_map", OccupancyGrid, self.map_callback, queue_size=1)

        # self.voronoi_gen_client = rospy.ServiceProxy("/gen_voronoi", gen_voronoi)
        self.frontiers_get_client = rospy.ServiceProxy("/get_frontiers", GetFrontiers)

        # self.voronoi_pub = rospy.Publisher("/voronoi_map", Graph, queue_size=1)
        self.frontiers_pub = rospy.Publisher('/frontiers', FrontierArray, queue_size=1)
        self.centroid_markers_pub = rospy.Publisher('/centroid_markers', MarkerArray, queue_size=10)

        self.oc_map = rospy.wait_for_message("/merged_map", OccupancyGrid)

        rate = rospy.Rate(3)

        while not rospy.is_shutdown():

            # rospy.loginfo("Pull a request to generate voronoi graph.")
            # req_voronoi = gen_voronoiRequest(self.oc_map)
            # res_voronoi = self.voronoi_gen_client.call(req_voronoi)
            # self.voronoi_pub.publish(res_voronoi.voronoi_map)

            rospy.loginfo("Pull a request to get frontiers.")
            req_fronteirs = GetFrontiersRequest(self.oc_map)
            self.frontiers_get_client.call(req_fronteirs)

            rate.sleep()

    def map_callback(self, msg):
        self.oc_map = msg




if __name__ == "__main__":
    rb = RoadmapBuilder()