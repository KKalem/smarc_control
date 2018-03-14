#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-02-09

import numpy as np

import time
import sys
import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from utils import geometry as G
from utils import Pid

import config


class LoloPublisher:
    def __init__(self, frame_id='odom'):
        """
        a simple class to keep information about lolos fins.
        publishes coordinated fin movements when move_xxx methods are called
        # fin 0 -> vertical top right, + = right
        # fin 1 -> vertback top left,  + = right
        # fin 2 -> vertback bottom, right, + = left
        # fin 3 -> vertback bottom, left, + = left
        # fin 4,5 -> dont do anything
        # back_fins/0 -> horizontal, + = down
        """
        self.fin0pub = rospy.Publisher(config.LOLO_FIN0_INPUT, FloatStamped, queue_size=1)
        self.fin1pub = rospy.Publisher(config.LOLO_FIN1_INPUT, FloatStamped, queue_size=1)
        self.fin2pub = rospy.Publisher(config.LOLO_FIN2_INPUT, FloatStamped, queue_size=1)
        self.fin3pub = rospy.Publisher(config.LOLO_FIN3_INPUT, FloatStamped, queue_size=1)
        self.backfinspub = rospy.Publisher(config.LOLO_BACKFIN_INPUT, FloatStamped, queue_size=1)

        self.frame_id = frame_id



    def yaw(self, direction, frame_id='odom'):
        """
        + = move right
        """
        #  direction = np.sign(direction)

        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = direction

        self.fin0pub.publish(out)
        self.fin1pub.publish(out)

        # the control for these fins are inverted for some reason
        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = direction * -1

        self.fin2pub.publish(out)
        self.fin3pub.publish(out)

        if np.sign(out.data)==1:
            config.pprint('>>>',out.data)
        elif np.sign(out.data)==-1:
            config.pprint('<<<',out.data)
        else:
            config.pprint('---',out.data)

    def pitch(self,direction, frame_id='odom'):
        """
        + = move up
        """
        #  direction = np.sign(direction)
        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = direction

        self.backfinspub.publish(out)
        if np.sign(out.data)==-1:
            config.pprint('^^^',out.data)
        elif np.sign(out.data)==1:
            config.pprint('vvv',out.data)
        else:
            config.pprint('---',out.data)


class LineController:
    def __init__(self, line_topic, pose_topic, no_pitch = False):

        # receive the line requests from ros
        rospy.Subscriber(line_topic, Path, self.update_line)

        self.pos = [0,0,0]
        self.ori = [0,0,0,0]

        # get the pose
        rospy.Subscriber(pose_topic, Odometry, self.update_pose)

        self._current_line = None
        self._frame_id = 'odom'

        self._yaw_pid = Pid.PID(*config.LOLO_YAW_PID)
        self._pitch_pid = Pid.PID(*config.LOLO_PITCH_PID)

        # create a publisher for LOLO
        self._lolopub = LoloPublisher()

        # we need to change from yz to xz if the two points have the same 
        # y,z values, this leads to nans in distance calculations!
        self._using_yz_for_pitch=True

        # disable pitch control
        self._no_pitch = no_pitch

    def update_pose(self, data):
        datapos = data.pose.pose.position
        x = datapos.x
        y = datapos.y
        z = datapos.z

        self.pos = [x,y,z]

        dataori = data.pose.pose.orientation
        x = dataori.x
        y = dataori.y
        z = dataori.z
        w = dataori.w

        self.ori = [x,y,z,w]

        self._frame_id = data.header.frame_id


    def update_line(self, data):
        # data should contain Path. which has a 'poses' field
        # which contains a list of "PoseStamped"
        p1 = data.poses[0].pose.position
        p2 = data.poses[1].pose.position

        # extract the  x,y,z values from the PoseStamped things
        l1 = (p1.x, p1.y, p1.z)
        l2 = (p2.x, p2.y, p2.z)

        # we finally have a line
        self._current_line = (l1, l2)

        # also extract the frame id, since we will need it when publishing control signals
        self._frame_id = data.header.frame_id


    def update(self, dt):
        if self._current_line is None:
            return
        # use a pid for yaw and another for pitch.
        # bang-bang control for the fins

        # first the yaw, find the yaw error
        # just project the 3D positions to z=0 plane for the yaw control

        # cant do the same checks we did for pitch since yaw HAS to use xy plane.
        # pitch can use either xz OR yz.
        yaw_pos = np.array(self.pos[:2])
        yaw_line_p1 = np.array(self._current_line[0][:2])
        yaw_line_p2 = np.array(self._current_line[1][:2])

        yaw_error = G.ptToLineSegment(yaw_line_p1, yaw_line_p2, yaw_pos)
        if not np.isnan(yaw_error):
            # this only gives the magnitude of the error, not the 'side' of it
            x,y = yaw_pos
            x1,y1 = yaw_line_p1
            x2,y2 = yaw_line_p2
            s = (x-x1)*(y2-y1)-(y-y1)*(x2-x1)
            # negative s = line is to the right
            yaw_correction = np.sign(s)*self._yaw_pid.update(yaw_error, dt)
            self._lolopub.yaw(yaw_correction)

        if not self._no_pitch:
            x0,y0,z0 = self.pos
            x1,y1,z1 = self._current_line[0]
            x2,y2,z2 = self._current_line[1]

            # create a plane from the current line.
            pa = np.array([x1,y1,z1])
            pb = np.array([x2,y2,z2])
            # put a point near the middle somewhere
            pc = np.array([x1+10,y1+10,(z1+z2)/2])
            # make a plane out of these 3 points
            ab = pa-pb
            ac = pa-pc
            xx = np.cross(ab,ac)
            d = xx[0]*pa[0] + xx[1]*pa[1] + xx[2]*pa[2]
            # this function returns +1 if the point is above the plane
            above = lambda pp: -np.sign(xx[0]*pp[0]+xx[1]*pp[1]+xx[2]*pp[2]-d)

            # this gives the magnitude of the error
            pitch_error = np.abs((xx[0]*x0+xx[1]*y0+xx[2]*z0+d)/np.sqrt(xx[0]**2+xx[1]**2+xx[2]**2))

            # this only gives the magnitude of the error, not the 'side' of it
            pitch_correction = self._pitch_pid.update(pitch_error, dt)

            # combine side with magnitude for control
            control = above(self.pos)*pitch_correction
            self._lolopub.pitch(control)


if __name__=='__main__':
    import config
    import rospy
    import time
    import sys

    rospy.init_node('line_controller', anonymous=True)

    args = sys.argv
    if args[1] == 'nopitch':
        no_pitch = True
    else:
        no_pitch = False

    pose_topic = config.POSE_TOPIC
    line_topic = config.LINE_TOPIC

    # controller subs to a line topic and follows that line using PID for pitch/yaw
    # LoLo rolls when yaw'ing and we can do nothing about it now. (16/02)
    lc = LineController(line_topic = line_topic,
                        pose_topic = pose_topic,
                        no_pitch = no_pitch)

    t1 = time.time()
    rate = rospy.Rate(config.UPDATE_FREQ)
    while not rospy.is_shutdown():
        dt = time.time()-t1
        lc.update(dt)
        t1 = time.time()
        rate.sleep()
