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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class VisualizationDriver():

    def __init__(self):
        # # Publisher initialization
        self.candidate_waypoints_pub = rospy.Publisher('candidate_waypoints', Marker, queue_size=10)
        self.best_waypoints_pub = rospy.Publisher('best_waypoints', Marker, queue_size=10)
        self.desired_heading_pub = rospy.Publisher('desired_heading', Marker, queue_size=10)
        self.desired_steering_pub = rospy.Publisher('desired_steering', Marker, queue_size=10)

    # Specific Functions
    def publish_candidate_waypoints(self, candidate_paths):
        c = 0
        for path in candidate_paths:
            c = c + 1;
            self.publish_waypoints(path, c, 0, 1 , 0, 0.10, self.candidate_waypoints_pub);

    def publish_best_waypoints(self, best_path):
        self.publish_waypoints(best_path, 100, 1, 0 , 0, 0.25, self.best_waypoints_pub);

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

    def publish_waypoints(self, path, pathID, colorr, colorg, colorb, markerSize, publisher):
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
            marker.pose.position.x = waypoint[1];
            marker.pose.position.y = waypoint[0];
            marker.pose.position.z = 0;
            marker.scale.x = markerSize;
            marker.scale.y = markerSize;
            marker.scale.z = markerSize;
            marker.color.a = 1.0;
            marker.color.r = colorr;
            marker.color.g = colorg;
            marker.color.b = colorb;
            publisher.publish(marker)
        
    def publish_costmap(self):
        pass
