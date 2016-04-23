# provides helper functions for running high speed and quality visualizations

import rospy
import time
from std_msgs.msg import Header, ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
import math
from helpers import param

class VisualizationDriver(object):
    """ The class responsible for visualizing the cars algorithms"""
    def __init__(self):
        self.viz_params = param("viz")
        self.channels = {}
        self.last_pubs = {}

        self.add_publisher("path_search.best_path", Marker)
        self.add_publisher("path_search.viable_paths", MarkerArray)
        self.add_publisher("path_search.speed", Float32)
        self.add_publisher("path_search.steering", Float32)

        self.add_publisher("goals.next_goal", Marker)
        self.add_publisher("goals.walls", Marker)
        self.add_publisher("goals.corridors", Marker)

        for k in self.channels.keys():
            self.last_pubs[k] = time.time()

    def add_publisher(self, channel_name, channel_type, queue_size=10):
        self.channels[channel_name] = rospy.Publisher(channel_name.replace(".", "/"), channel_type, queue_size=queue_size)

    def get_publisher(self, channel_name):
        return self.channels[channel_name]

    def has_subscribers(self, channel_name):
        return self.get_publisher(channel_name).get_num_connections() > 0

    def get_info(self, channel_name):
        ns = channel_name.split(".")
        info = self.viz_params["topics"]
        while ns:
            info = info[ns[0]]
            ns.pop(0)
        return info

    def ratelimit_observed(self, channel_name):
        rl = float(self.get_info(channel_name)["rate_limit"])
        if rl == 0:
            return False
        return time.time() - self.last_pubs[channel_name] > 1.0 / rl

    # returns true if there are subscribers and the topic is not above the given ratelimit
    def should_visualize(self, channel_name):
        return self.has_subscribers(channel_name) and self.ratelimit_observed(channel_name)

    def publish(self, name, msg):
        self.last_pubs[name] = time.time()
        self.channels[name].publish(msg)

    def publish_best_path(self, best_path):
        marker = self.marker_from_path(best_path.states, z=0.1, linewidth=0.06, \
            lifetime=1.0/float(self.get_info("path_search.best_path")["rate_limit"]))
        self.publish("path_search.best_path", marker)

    # publishes the whole path tree, making sure not to duplicate path segments
    def publish_viable_paths(self, path_tree):
        markers = [self.marker_clear_all()]

        def non_overlapping_paths(node):
            # given a tree data structure, this function will return list of lists of the
            # node "state" attributes
            # if visualized as a tree, these states will not contain overlapping segments
            if not node.children:
                return [node.state]
            else:
                paths = []

                for child in node.children:
                    child_paths = non_overlapping_paths(child)

                    if type(child_paths[0]) == list:
                        # child is not a leaf node, add self to first path and store
                        child_paths[0].insert(0, node.state)
                        paths = paths + child_paths
                    else:
                        # this is a leaf node, add self to path and store
                        child_paths.insert(0, node.state)
                        paths.append(child_paths)

                return paths

        candidate_paths = non_overlapping_paths(path_tree)
        markers += [self.marker_from_path(path, index=i, linewidth=0.03, color=ColorRGBA(0, 1, 0, 1), \
                    lifetime=1.0/float(self.get_info("path_search.best_path")["rate_limit"]))
                    for i, path in enumerate(candidate_paths)]
        marker_array = MarkerArray(markers=markers)

        self.publish("path_search.viable_paths", marker_array)

    def marker_from_path(self, states, index=0, linewidth=0.1, color=ColorRGBA(1, 0, 0, 1), z=0., lifetime=10.0):
        marker = Marker()
        marker.header = Header(
            stamp=rospy.Time.now(),
            frame_id="base_link")

        marker.ns = "Markers_NS"
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = 0 # action=0 add/modify object
        marker.color = color
        marker.lifetime = rospy.Duration.from_sec(lifetime)

        marker.pose = Pose()
        marker.pose.position.z = z
        marker.scale = Vector3(linewidth, 1, 1)

        # Fill the marker from the path.
        points = []
        for waypoint in states:
            points.append(Point(float(waypoint.x), float(waypoint.y), 0))
        marker.points = points
        marker.colors = []

        return marker

    def marker_clear_all(self):
        # Create a marker which clears all.
        marker = Marker()
        marker.header.frame_id = "base_link";
        marker.action = 3 # DELETEALL action.
        return marker
