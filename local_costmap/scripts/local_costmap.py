#!/usr/bin/env python
from __future__ import print_function
import rospy
from scipy import ndimage

from visualization_driver import VisualizationDriver

class FrameBuffer:
    def __init__(self, resolution=5, x_range=(-8,8), y_range=(-5,8)):
        self.discretization = resolution # bins per meter
        self.max_x = x_range[1]
        self.max_y = y_range[1]
        self.min_x = x_range[0]
        self.min_y = y_range[0]

        self.x0 = 0
        self.y0 = 0
        self.distmap = None

        self.find_center()
        self.clear()

    def find_center(self):
        self.x0 = abs(int(self.min_x * self.discretization))
        self.y0 = abs(int(self.min_y * self.discretization))

    def clear(self):
        width = (self.max_x - self.min_x) * self.discretization
        height = (self.max_y - self.min_y) * self.discretization

        # self.buffer = np.ones((width, height))
        self.buffer = np.ones((height, width))

    def add_sample(self, x, y):
        if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y):
            xind = int(x * self.discretization)
            yind = int(y * self.discretization)
            # add obstacle to the buffer
            self.buffer[yind + self.y0][xind + self.x0] = False

    def dist_transform(self):
        self.distmap = ndimage.distance_transform_edt(self.buffer)
        return self.distmap

    def dist_at(self, x, y):
        if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y) and not self.distmap == None:
            xind = int(x * self.discretization)
            yind = int(y * self.discretization)
            # add obstacle to the buffer
            return self.distmap[yind + self.y0][xind + self.x0] / self.discretization
        return 1000000


class LocalCostmap(object):
    """  Generates a costmap based off of sensor data from the car, without any global sense of map """
    def __init__(self, arg):

        # the class which manages the local costmap buffer
        self.buffer = FrameBuffer(5, (-8,8), (-5,8))
        self.first_laser_recieved = False

        self.node = rospy.init_node('icarus_main', anonymous=True)
        self.scan_subscriber = rospy.Subscriber(self.LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)

        self.pub_costmap = rospy.Publisher('~costmap', PoseArray, queue_size=1)

    def filter_lasers(self, angles, ranges):
        # do nothing
        return (angles, ranges)
        # remove the data on the edges, since they are noisy
        l = round(angles.shape[0] / 10)
        
        return (angles[l:-l], ranges[l:-l])

    def scan_callback(self, data):
        start = time.clock()

        if not self.first_laser_recieved:
            print("first laser received: angle_min: %f angle_max %f angle_incr: %f ranges_len: %d range_min: %.2f range_max_ %2.2f" % (data.angle_min, data.angle_max, data.angle_increment, data.ranges.shape[0], data.range_min, data.range_max))

        laser_angles = np.linspace(data.angle_min, data.angle_max, math.ceil((data.angle_max - data.angle_min) / data.angle_increment))
        laser_ranges = data.ranges
        

        laser_angles, laser_ranges  = self.filter_lasers(laser_angles, laser_ranges, data.range_min, data.range_max)
        laser_x, laser_y =  self.polar_to_euclid(laser_angles, laser_ranges)

        # compute the distance transform from the laser scanner data
        self.buffer.clear()
        for x, y in zip(laser_x, laser_y):
            self.buffer.add_sample(x,y)
        self.buffer.dist_transform()

        print ("Done computing distance map in:", time.clock() - start, "seconds")

        # if self.visualize == True:
        #     if self.im == None:
        #         self.im = plt.imshow(self.buffer.distmap)
        #         plt.colorbar(self.im)
        #     else:
        #         self.im.set_array(self.buffer.distmap)
        #     plt.draw()

        self.first_laser_recieved = True  


'''
    def generate_candidate_paths(self):
        curvatures = np.linspace(0, self.max_curve, num=self.path_candidates)

        paths = []

        ys = np.linspace(0, self.path_length, num=self.path_discretization)
        xs = np.zeros(self.path_discretization)

        paths.append(Path(0, self.max_speed, zip(xs,ys)))

        for curvature in curvatures[1:]:
            # set the speed profile
            if self.FAST_MODE:
                xp = [0,  .13, .2, .3,  self.max_curve]
                fp = [1,   1,  .6, .4,  .2]
            else:
                xp = [0,  .15, .2,  self.max_curve]
                fp = [1,   1,  .75,   .4]
            
            multiplier = np.interp(curvature, xp, fp)

            r = self.radius(curvature)
            # theta = self.path_length * math.sqrt(multiplier) / r
            theta = self.path_length / r
            thetas = np.linspace(0, theta, num=self.path_discretization)

            ys = r * np.sin(thetas)
            xs = r * np.cos(thetas) - r

            paths.append(Path(curvature, self.max_speed * multiplier, zip(xs, ys)))
            paths.append(Path(-1*curvature, self.max_speed * multiplier, zip(-1*xs, ys)))

        return paths
'''
class PathGenerator(object):
    def __init__(self):
        pass
    def generate_paths(self):
        ''' Return a list of Path namedtuples for later evaluation
        '''
        pass

'''
def pick_path(self, path_candidates):

        # desired_heading = self.navigation_controller.desired_heading

        # if desired_heading == None:
        #     return None
        # else:
        #     return Path(desired_heading, self.max_speed)

        best_path = None
        best_cost = 2500

        for path in path_candidates:
            cost = self.local_controller.evaluate_path(path)
            cost += 220 * self.navigation_controller.evaluate_path(path)

            if cost < best_cost:
                # print("better path found:", path, cost)
                best_path = path
                best_cost = cost

        return best_path
'''

class PathEvaluator(object):
    def __init__(self):
        pass
    def evaluate_paths(self, paths, costmap):
        ''' Return cost metrics for each path. 
            The lowest cost path will be chosen for execution.
        '''
        pass

class LocalExplorer(object):
    def __init__(self):
        # TODO inherit from controller thing
        self.PLANNING_FREQ = 40

        self.costmap = LocalCostmap()
        self.path_gen = PathGenerator()
        self.path_eval = PathEvaluator()
        self.visualization_driver = VisualizationDriver()

        rospy.Timer(rospy.Duration(1.0 / self.PLANNING_FREQ), self.timer_callback)

    def timer_callback(self, event):
        rospy.logdebug("Computing control in LocalExplorer")

        paths = self.path_gen.generate_paths()
        costs = self.path_eval.evaluate_paths(paths, self.costmap)

        assert len(paths) == len(costs)

        # TODO a different path evaluator might return the picked path directly
        best_path = paths[min(range(len(costs)), key=lambda i: costs[i])]

        # Visualizations
        # TODO untested
        self.visualization_driver.publish_candidate_waypoints(paths)
        self.visualization_driver.publish_best_waypoints(best_path)

        # TODO steer towards best_path.steering_angle

Path = collections.namedtuple("Path", ["steering_angle", "waypoints", "speed"])

if __name__ == '__main__':
    try:
        LocalExplorer()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
