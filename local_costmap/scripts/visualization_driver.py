## To use this driver:
# 1. import this function in heading:
#     from racecar_visualization_driver import VisualizationDriver
# 2. add the following line in Class "IcarusCar" Initiation:
#        self.visualization_driver = VisualizationDriver()
# 3. add the following lines after "candidate_paths" and "path" variables are defined in the "scan_callback" function
#    self.visualization_driver.publish_candidate_waypoints(candidate_paths)
#    self.visualization_driver.publish_best_waypoints(path)


## To use the desired_heading direction (added 01/20/2016, 1:20am):
# 4. add the following line in Class "DirectionController" Initiation:
#        self.visualization_driver = VisualizationDriver()
# 5. add the following line after "desired_heading" is defined in the "update_scan" function
#        self.visualization_driver.publish_desired_heading( -1 * self.desired_heading)

import rospy
import random
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
import math

class VisualizationDriver():

    def __init__(self):
        # # Publisher initialization
        self.candidate_waypoints_pub = rospy.Publisher('candidate_waypoints', Marker, queue_size=10)
        self.best_waypoints_pub = rospy.Publisher('best_waypoints', Marker, queue_size=10)
        self.desired_heading_pub = rospy.Publisher('desired_heading', Marker, queue_size=10)
        self.desired_steering_pub = rospy.Publisher('desired_steering', Marker, queue_size=10)

        self.best_path_pub = rospy.Publisher('best_path', Marker, queue_size=10)
        self.candidate_paths_pub = rospy.Publisher('candidate_paths', MarkerArray, queue_size=10)

    # Specific Functions
    def publish_candidate_waypoints(self, candidate_paths, costmap=None):
        self.candidate_waypoints_pub(self.marker_clear_all())
        c = 0
        for path in candidate_paths:
            c = c + 1;
            self.publish_waypoints(path, c, 0, 1 , 0, 0.10, self.candidate_waypoints_pub, costmap=costmap);

    def publish_best_waypoints(self, best_path, costmap=None):
        self.best_waypoints_pub(self.marker_clear_all())
        self.publish_waypoints(best_path, 100, 1, 0 , 0, 0.25, self.best_waypoints_pub, costmap=costmap);

    def publish_best_path(self, best_path, costmap=None):
        marker = self.marker_from_path(best_path, z=0.1, linewidth=0.06, costmap=costmap)
        self.best_path_pub.publish(marker)

    def publish_candidate_paths(self, candidate_paths, costmap=None):
        markers = [self.marker_clear_all()]
        markers += [self.marker_from_path(path,
                                          index=i,
                                          linewidth=0.03,
                                          color=ColorRGBA(0, 1, 0, 1),
                                          costmap=costmap)
                    for i, path in enumerate(candidate_paths)]
        marker_array = MarkerArray(markers=markers)
        self.candidate_paths_pub.publish(marker_array)

    def marker_from_path(self, path, index=0, linewidth=0.1, color=ColorRGBA(1, 0, 0, 1), z=0., costmap=None):
        marker = Marker()
        marker.header = Header(
            stamp=rospy.Time.now(),
            frame_id="base_link")

        marker.ns = "Markers_NS"
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = 0 # action=0 add/modify object
        marker.color = color
        marker.lifetime = rospy.Duration.from_sec(10.)

        marker.pose = Pose()
        marker.pose.position.z = z
        marker.scale = Vector3(linewidth, 1, 1)

        # Fill the marker from the path.
        points = []
        for waypoint in path.waypoints:
            x, y = waypoint[0], waypoint[1]
            points.append(Point(x, y, 0))
        marker.points = points
        marker.colors = []

        return marker

    def publish_desired_heading(self, heading):
        marker = Marker();
        marker.header.frame_id = "base_link";
        marker.ns = "Markers_NS";
        marker.id = 1000000;
        marker.type = 0;
        marker.action = 0;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
    
        points = [];
        length = 3.0;
        points.append(Point(0, 0, 0));
        if heading:
            points.append(Point(length * math.cos(heading), length * math.sin(heading), 0));
    
        marker.points = points;
    
        marker.scale.x = 0.1;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
        self.desired_heading_pub.publish(marker)
        
    def publish_desired_steering(self, steering):
        marker = Marker();
        marker.header.frame_id = "base_link";
        marker.ns = "Markers_NS";
        marker.id = 1000000;
        marker.type = 0;
        marker.action = 0;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
    
        points = [];
        length = 1.5;
        points.append(Point(0, 0, 0));
        if steering:
            points.append(Point(length * math.cos(steering), length * math.sin(steering), 0));
    
        marker.points = points;
    
        marker.scale.x = 0.1;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        self.desired_steering_pub.publish(marker)

    def publish_waypoints(self, path, pathID, colorr, colorg, colorb, markerSize, publisher, costmap=None):
        # # Assigning unique ID is key for displaying the right amount of points on rviz
        w = 0;
        for waypoint in path.waypoints:
            w = w + 1;
            marker = Marker();
            marker.header.frame_id = "base_link";
            marker.ns = "Markers_NS";
            marker.id = pathID * 100 + w;
            marker.type = 2;
            marker.action = 0;
            marker.pose.position.x = waypoint[0];
            marker.pose.position.y = waypoint[1];
            marker.pose.position.z = 0;
            if costmap == None:
                size = markerSize
            else:
                x, y = waypoint[0], waypoint[1]
                factor = costmap.cost_at(waypoint[0], waypoint[1])
                size = markerSize * (.3 + factor)
            marker.scale.x = size;
            marker.scale.y = size;
            marker.scale.z = size;
            marker.color.a = 1.0;
            marker.color.r = colorr;
            marker.color.g = colorg;
            marker.color.b = colorb;
            publisher.publish(marker)

    def marker_clear_all(self):
        # Create a marker which clears all.
        marker = Marker()
        marker.header.frame_id = "base_link";
        marker.action = 3 # DELETEALL action.
        return marker
        
    def publish_costmap(self):
        pass
