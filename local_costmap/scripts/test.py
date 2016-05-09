#!/usr/bin/env python
from __future__ import print_function
import rospy

import numpy as np
from helpers import *
from scipy import signal
import matplotlib.pyplot as plt
import threading
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped
import random
from navigator3 import ScanMaximaNavigator
from visualization_driver import VisualizationDriver


from time import gmtime, strftime
import yaml, itertools

def dump_records(fn, records):
    with open(fn, 'w') as f:
        f.write(yaml.dump({'records': records}))

def append_records(fn, record):
    with open(fn, 'a') as f:
        f.write(yaml.dump(record))

def fetch_records(fn):
    d = []
    with open(fn, 'r') as stream:
        try:
            return(yaml.load(stream)['records'])
        except yaml.YAMLError as exc:
            print(exc)


Scan = collections.namedtuple("Scan", ["angle_increment", "angle_min", "angle_max", "ranges"])

class CaptureData(object):
    """ Capture data from RVIZ about selected headings and save yaml"""
    def __init__(self):
        rospy.init_node('capture_data')

        self.records = []
        self.last_scan = None

        self.scan_subscriber = rospy.Subscriber("/scan",
            numpy_msg(LaserScan), self.scan_callback, queue_size=1)

        self.point_subscriber = rospy.Subscriber("/move_base_simple/goal",
            PoseStamped, self.click_callback, queue_size=1)

        rospy.on_shutdown(lambda: self.on_shutdown())

    def click_callback(self, pose):
        if self.last_scan:
            print("Added scan...")
            self.add_record(pose.pose.position, self.last_scan)

    def scan_callback(self, scan):
        self.last_scan = scan

    def add_record(self, picked_goal, laser_msg):
        record = {
            "laser": {
                "ranges": map(lambda x: float(x), laser_msg.ranges),
                "angle_increment": float(laser_msg.angle_increment),
                "angle_min": float(laser_msg.angle_min),
                "angle_max": float(laser_msg.angle_max)
            },
            "picked_goal": {
                "x": float(picked_goal.x),
                "y": float(picked_goal.y),
                "theta": float(euclid_to_polar(picked_goal.x, picked_goal.y)[1])
            }
        }
        self.records.append(record)

    def save_records(self):
        if len(self.records) > 0:
            dump_records("parameter_data_" + strftime("%Y_%m_%d_%H_%M", gmtime()), self.records)
        
    def on_shutdown(self):
        self.save_records()

class OptimizeNavigationParameters(object):
    """docstring for OptimizeParameters"""
    def __init__(self):
        rospy.init_node('optimize_params')

        # holds all of the parameter permutations
        self.particles = []
        self.scores = []
        self.training_data = []
        self.not_started = True
        self.best_err = 1000

        self.bail_cutoff = 1.6

        self.fps_counter = FPSCounter2(size=100)

        viz = VisualizationDriver()
        self.navigator = ScanMaximaNavigator(viz, True)

        # fills up the particle array
        self.load_training_data()
        self.generate_particles()
        self.score_particles()

        self.save()

        def on_shutdown(self):
            self.save()

    def load_training_data(self):
        print("Loading training data...")
        self.training_data = fetch_records("parameter_data.txt")
        print("Loaded", len(self.training_data), "training records")

    def generate_particles(self):
        data = param("navigator")

        bounds = map(lambda x: x[1]["bounds"], data["optimization"].iteritems())
        types = map(lambda x: x[1]["type"], data["optimization"].iteritems())
        steps = map(lambda x: x[1]["step"], data["optimization"].iteritems())
        names = map(lambda x: x[0], data["optimization"].iteritems())
        
        discretizations = []

        for i in xrange(len(names)):
            rmin = bounds[i][0]
            rmax = bounds[i][1]
            step = steps[i]

            if types[i] == "int":
                options = list(np.arange(int(rmin), int(rmax), int(step)))
            elif types[i] == "float":
                options = list(np.arange(float(rmin), float(rmax), float(step)))
            elif types[i] == "bool":
                options = [0,1]

            discretizations.append(options)

        num_options = map(lambda x: len(x), discretizations)
        print("Generating", np.product(num_options), "particles")

        particles = list(itertools.product(*discretizations))
        random.shuffle(particles)
        self.particles = particles
        self.names = names

        print("Done generating", len(self.particles), "particles")

    def score_particles(self):
        print("Scoring particles...")
        
        count = 0
        for i in self.particles:
            if self.fps_counter.index % 50 == 1:
                print()
                print("finished processing", count, "of", len(self.particles), "particles")
                print ("recent particles per second: ", self.fps_counter.fps())
            
            self.fps_counter.step()
            count += 1
            
            try:
                err = self.score(i)
            except:
                err = 1000
            
            self.scores.append(err)
            
            if err < self.best_err:
                self.best_err = err
                best_particle = i

                print()
                print("Found better option with err:", err)
                print("   particle: ", self.annotate_particle(i))
            
            # log all somewhat decent options
            if err < self.bail_cutoff * self.best_err:
                self.log(i, err)

        print("Done scoring particles")

    def annotate_particle(self, particle):
        param_obj = {}
        for i in xrange(len(self.names)):
            param_obj[self.names[i]] = particle[i]
        return param_obj

    def log(self, particle, err):

        record = {
            "particle": map(lambda y: float(y), particle),
            "names": self.names,
            "err": float(err)
        }
        append_records("parameter_log.txt", record)

    def score(self, particle):
        param_obj = self.annotate_particle(particle)
        self.navigator.set_params(param_obj)

        running_score = 0
        for i in self.training_data:
            ld = i["laser"]
            laser_obj = Scan(ranges=ld["ranges"], angle_increment=ld["angle_increment"], 
                angle_min=ld["angle_min"], angle_max=ld["angle_max"])

            self.navigator.scan_callback(laser_obj)

            if self.navigator.goalpoint():
                angle = self.navigator.goalpoint()[2]
                ground_truth_angle = i["picked_goal"]["theta"]
                angular_err = abs(angle - ground_truth_angle)

                running_score += angular_err*angular_err
            else:
                # penalty for failing to find a goal point in a given trial
                running_score += 10

            if running_score > self.best_err * self.bail_cutoff:
                # break early if this one is sure to not work
                break

        return running_score


    def save(self):
        print("Saving, do not cancel!")
        data = {
            "particles": map(lambda x: map(lambda y: float(y), x), self.particles),
            "scores": map(lambda x: float(x), self.scores),
            "names": self.names
        }

        dump_records("optimization_results.txt", data)

import matplotlib.pyplot as plt

class LoadParameterLog(object):
    """docstring for LoadParameterLog

        notes: 
            contiguous_min: 20 - 70
            laser max: 
    """
    def __init__(self):

        rospy.init_node('load_params')

        self.raw_log = None
        self.log = []

        # self.load_raw_log("parameter_log4.txt")
        
        self.load_log("cleaned_log_pretty_good.yaml")
        self.filter_log()
        # self.print_log()
        # self.save_log("cleaned_log3_good_filtered.yaml")
        self.analyze(["gaussian_width", "laser_max", "distance_threshold", "goal_angle_max"])

    def load_raw_log(self, log_file="parameter_log.txt"):
        print("Loading raw log")
        with open(log_file, 'r') as stream:
            data = stream.readlines()

            chunks = []
            for i in data:
                if i[0] == "e":
                    chunks.append([i])
                elif i[0] == " ":
                    chunks[-1][-1] = chunks[-1][-1] + i.strip()
                else:
                    chunks[-1].append(i.strip())

            for chunk in chunks:
                err = yaml.load(chunk[0])["err"]
                names = yaml.load(chunk[1])["names"]
                raw_particle = yaml.load(chunk[2])["particle"]

                particle = {
                    "params": {},
                    "err": err
                }

                for i in xrange(len(names)):
                    # raw_particle[i]
                    particle["params"][names[i]] = raw_particle[i]

                self.log.append(particle)
        print("Done loading log")

    def load_log(self, log_file="cleaned_log.yaml"):
        print("Loading log")
        self.log = fetch_records(log_file)
        print("Done loading log")
    
    def save_log(self, log_file="cleaned_log.yaml"):
        dump_records(log_file, self.log)
    
    def filter_log(self):
        # print(len(self.log))
        # self.log = filter(lambda x: x["err"] < 1.8, self.log)
        # self.log = filter(lambda x: x["params"]["laser_max"] >= 7.5, self.log)

        self.log = filter(lambda x: x["params"]["gaussian_width"] > 0.11, self.log)
        self.log = filter(lambda x: x["params"]["gaussian_width"] < 0.17, self.log)
        
        self.log = filter(lambda x: x["params"]["contiguous_min"] > 45, self.log)
        self.log = filter(lambda x: x["params"]["contiguous_min"] < 50, self.log)

        self.log = filter(lambda x: x["params"]["goal_angle_max"] > 1.41, self.log)
        self.log = filter(lambda x: x["params"]["goal_angle_max"] <= 1.6, self.log)
        # self.log = filter(lambda x: x["params"]["goal_angle_max"] >= 7.5, self.log)

        # uniq = []

        # for i in self.log:
        #     def found(particle):
        #         for i in particle

        # print(len(self.log))
        # self.log = list(set(self.log))
        # print(len(self.log))

    def analyze(self, which):

        print(len(self.log))
        # gw = np.array(map(lambda x: x["params"]["gaussian_width"], self.log))
        # mw = np.array(map(lambda x: x["params"]["median_width"], self.log))
        # plt.scatter(gw, mw)

        fig = plt.figure()

        for i in xrange(len(which)):
            ax = plt.subplot(len(which),1,i+1) 

            errs = np.array(map(lambda x: x["params"][which[i]], self.log))
            hist, bins = np.histogram(errs, bins=20)
            width = 0.7 * (bins[1] - bins[0])
            center = (bins[:-1] + bins[1:]) / 2

            ax.bar(center, hist, align='center', width=width)

        # errs = np.array(map(lambda x: x["params"][which], self.log))
        # hist, bins = np.histogram(errs, bins=20)
        # width = 0.7 * (bins[1] - bins[0])
        # center = (bins[:-1] + bins[1:]) / 2

        # fig = plt.figure()
        # ax1 = fig.add_subplot(211)
        # ax1.bar(center, hist, align='center', width=width)

        # ax2 = fig.add_subplot(212)
        
        # plt.bar(center, hist, align='center', width=width)
        plt.show()
        
    def print_log(self):
        for i in self.log:
            print()
            print(i)
        
if __name__ == '__main__':
    try:
        # CaptureData()
        # OptimizeNavigationParameters()
        LoadParameterLog()
        print("Done.")
    except rospy.ROSInterruptException:
        pass
    rospy.spin()