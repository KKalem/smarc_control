#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

import numpy as np

import config
import rospy
import time

from csv_line_publisher import LinePublisher
from line_controller import LineController

if __name__=='__main__':
    rospy.init_node('example_line_follower', anonymous=True)

    rate = rospy.Rate(config.UPDATE_FREQ)
    waypoint_file = config.WAYPOINTS_FILE
    pose_topic = config.POSE_TOPIC
    line_topic = config.LINE_TOPIC


    # parse the csv file to a list of tuples
    with open(waypoint_file, 'r') as fin:
        wps = [tuple(map(lambda a: float(a.strip()), line.split(','))) for line in fin.readlines()]

    # line planner acts like an external planning node and publishes a line
    # for the control to follow
    lp = LinePublisher(waypoints = wps,
                       pose_topic = pose_topic,
                       line_topic = line_topic)

    # controller subs to a line topic and follows that line using PID for pitch/yaw
    # LoLo rolls when yaw'ing and we can do nothing about it now. (16/02)
    lc = LineController(line_topic = line_topic,
                        pose_topic = pose_topic,
                        no_pitch=True)

    t1 = time.time()
    while not rospy.is_shutdown():
        dt = time.time()-t1
        lp.update()
        lc.update(dt)
        t1 = time.time()
        rate.sleep()
