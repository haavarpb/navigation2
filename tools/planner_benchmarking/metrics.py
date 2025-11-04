#! /usr/bin/env python3
# Copyright 2022 Joshua Wallace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import sys
import argparse
from functools import partial

import math
import os
import pickle
import numpy as np

from random import seed
from random import randint
from random import uniform

from transforms3d.euler import euler2quat


def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        path = navigator._getPathImpl(initial_pose, goal_pose, planner, use_start=True)
        if path is not None:
            results.append(path)
        else:
            return results
    return results


def getRandomStart(costmap, max_cost, side_buffer, time_stamp, res, origin):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0]-side_buffer)
        col = randint(side_buffer, costmap.shape[1]-side_buffer)

        if costmap[row, col] < max_cost:
            # Transform the starting vector given relative to the grid frame to the map frame
            start.pose.position.x = col*res + origin[0]
            start.pose.position.y = row*res + origin[1]

            yaw = uniform(0, 1) * 2*math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            start.pose.orientation.w = quad[0]
            start.pose.orientation.x = quad[1]
            start.pose.orientation.y = quad[2]
            start.pose.orientation.z = quad[3]
            break
    return start


def getRandomGoal(costmap, max_cost, side_buffer, time_stamp, res, origin, start):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0]-side_buffer)
        col = randint(side_buffer, costmap.shape[1]-side_buffer)

        start_x = start.pose.position.x
        start_y = start.pose.position.y
        goal_x = col*res + origin[0]
        goal_y = row*res + origin[1]
        x_diff = goal_x - start_x
        y_diff = goal_y - start_y
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

        if costmap[row, col] < max_cost and dist > 1.0: # Make it a variable based on the size of the map
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y

            yaw = uniform(0, 1) * 2*math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            goal.pose.orientation.w = quad[0]
            goal.pose.orientation.x = quad[1]
            goal.pose.orientation.y = quad[2]
            goal.pose.orientation.z = quad[3]
            break
    return goal

def poseStamped(frame_id, pose2d, ts, start):
    retval = PoseStamped()
    retval.header.frame_id = frame_id
    retval.header.stamp = ts
    retval.pose.position.x = pose2d[0]
    retval.pose.position.y = pose2d[1]
    return retval


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--start_pose', nargs=3, default=None, metavar=tuple("xyz"))
    parser.add_argument('-g', '--goal_pose', nargs=3, default=None, metavar=tuple("xyz"))
    parser.add_argument('-it', '--iterations', default=100, metavar='n')
    parser.add_argument('-p', '--planners', nargs="+", default=['GridBased'])
    parser.add_argument('-r', '--random_seed', type=int, default=33)
    args = parser.parse_args(sys.argv[1:])

    rclpy.init()

    navigator = BasicNavigator()

    # Get the costmap for start/goal validation
    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    ox = costmap_msg.metadata.origin.position.x # Position of pgm-file origin relative to map
    oy = costmap_msg.metadata.origin.position.y # Position of pgm-file origin relative to map
    origin = np.array([ox, oy])
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)
    planners = args.p
    max_cost = 210
    side_buffer = round(np.min(costmap.shape)*0.1)
    time_stamp = navigator.get_clock().now().to_msg()
    results = []
    seed(args.r)
    random_pairs = int(args.it)
    res = costmap_msg.metadata.resolution

    if args.s is not None or args.g is not None:
        # Use random start/goals
        getStart = partial(poseStamped, 'map', np.array(args.s).astype(float), time_stamp, None)
        getGoal = partial(poseStamped, 'map', np.array(args.g).astype(float), time_stamp)
    else:
        getStart = partial(getRandomStart, costmap, max_cost, side_buffer, time_stamp, res, origin)
        getGoal = partial(getRandomGoal, costmap, max_cost, side_buffer, time_stamp, res, origin)

    for i in range(random_pairs):
        print("Cycle: ", i, "out of: ", random_pairs)
        start = getStart()
        goal = getGoal(start)
        print("Start", start)
        print("Goal", goal)
        result = getPlannerResults(navigator, start, goal, planners)
        if len(result) == len(planners):
            results.append(result)
        else:
            print("One of the planners was invalid")

    print("Write Results...")
    with open(os.getcwd() + '/results.pickle', 'wb+') as f:
        pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/costmap.pickle', 'wb+') as f:
        pickle.dump(costmap_msg, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/planners.pickle', 'wb+') as f:
        pickle.dump(planners, f, pickle.HIGHEST_PROTOCOL)
    print("Write Complete")
    exit(0)


if __name__ == '__main__':
    main()
