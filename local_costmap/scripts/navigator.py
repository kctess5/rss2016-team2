""" Provides a goal pose based on sensor data, trying to follow the challenge route """
import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Vector3
import whoami

class Navigator(object):

    STRAIGHT_AHEAD = [8, 0, 0]

    def __init__(self, visualize=False):
        self.should_visualize = visualize
        self.goal = self.STRAIGHT_AHEAD

        self.laser_topic = "/scan" if whoami.is_racecar() else "/racecar/laser/scan"
        self.laser_sub = rospy.Subscriber(self.laser_topic, \
                LaserScan, self.laser_update, queue_size=1)

        if self.should_visualize:
            self.marker_id = 0
            self.viz_prefix = "local_costmap/navigator/viz/"
            self.goal_viz_topic = self.viz_prefix + "goal"
            self.corridor_viz_topic = self.viz_prefix + "corridors"
            self.goal_viz_pub = rospy.Publisher(self.goal_viz_topic, Marker, queue_size=10)
            self.corridor_viz_pub = rospy.Publisher(self.corridor_viz_topic, Marker, queue_size=10)

    def laser_update(self, laser_data):
        """
        Incorporates the laser data into its goal output
        ( This method doesn't return a value )
        """
        pass

    def camera_update(self, camera_data):
        """
        Incorporates the camera data into its goal output
        ( This method doesn't return a value )
        """
        pass

    def goalpoint(self):
        """
        Returns the current goal point as a list [x, y, heading] in the base frame.
        """
        return self.goal

    def visualize(self):
        """
        Output a marker for the goal point
        """
        if not self.should_visualize:
            return

        goal = Marker()
        goal.header = Header(
                stamp=rospy.Time.now(),
                frame_id="hokuyo_link") #TODO this should be base someday
        goal.ns = "navigator"
        goal.id = self.marker_id
        self.marker_id += 1
        goal.type = Marker.ARROW
        goal.action = 0
        goal.color = ColorRGBA(0, 1, 1, 1)
        goal.lifetime = rospy.Duration.from_sec(10.)
        goal.pose = Pose()
        goal.pose.position.z = 0.
        goal.pose.position.x = self.goal[0]
        goal.pose.position.y = self.goal[1]
        goal.scale = Vector3(.5, .1, .1)
        self.goal_viz_pub.publish(goal)
