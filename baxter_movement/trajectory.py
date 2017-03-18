#!/usr/bin/env python

import rospy
import sys
import struct
import argparse
import copy
import numpy
import math
import actionlib
import roslib
import string
import time
import tf

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from random import randint

from baxter_interface import CHECK_VERSION
import baxter_interface
from baxter_core_msgs.msg import EndpointState

import control



def Ik_solution(p,q):

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    rs.enable()

    limb = 'right'
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
    'left': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q)),
    'right': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q))}

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limb_joint = list(resp.joints[0].position)
        # limb = baxter_interface.Limb('right')
        # limb.set_joint_positions(limb_joints)
        # limb.move_to_joint_positions(limb_joints)
        #print "\nIK Joint Solution:\n", limb_joints
        return limb_joint

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return

def BaxterMovement():
    limb.set_joint_positions(limb_joints)

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)
        # print point

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

def draw_face(keyboard_str, offset_x, offset_y, offset_r):

    # draw a circle
    r = 10
    q = Quaternion(x=-0.4177807149532, y=4.099274143222, z=-0.15996636325311, w=-0.1321823626761)
    z_high = -0.0051726872428+0.01
    z_low = -0.0301726872428+0.01
    x0 = 0.678147548375
    y0 = -0.2908471534
    p0 = Point(x=x0,y=y0,z=z_high)

    limb = "right"
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    limb_interface = baxter_interface.limb.Limb(limb)
    # draw_circle(0, 0, 1, 0)
    time = 16

    # print "finish"

    control.BaxterMovement(p0,q)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)
    start_point = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_high)
    p1 = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_low)
    angles_start0 = Ik_solution(start_point,q)
    traj.add_point(angles_start0, 1)
    angles_start1 = Ik_solution(p1,q)
    traj.add_point(angles_start1, 2)
    r_m = offset_r*r
    x0_m = x0+offset_x*r/100
    y0_m = y0+offset_y*r/100
    for t in range(1, int(offset_r*100+2)):
        theta = t*2*3.1415926/(offset_r*100)
        yp = r_m*math.sin(theta)
        xp = r_m*math.cos(theta)
        p2 = Point(x=x0_m+xp/100.0,y=y0_m+yp/100.0,z=z_low)
        angles = Ik_solution(p2,q)
        traj.add_point(angles, 2+0.1*t)
    angles_end0 = Ik_solution(start_point,q)
    traj.add_point(angles_end0, 14)
    angles_end1 = Ik_solution(p0,q)
    traj.add_point(angles_end1, 15)


    # happy
    if keyboard_str == 1:
        #draw a circle
        # draw_circle(0, 0, 1)
        eye_left_st = Point(x=x0-0.2*r/100.0,y=y0-0.6*r/100.0,z=z_high)
        eye_left1 = Point(x=x0-0.2*r/100.0,y=y0-0.6*r/100.0,z=z_low)
        eye_left2 = Point(x=x0-0.4*r/100.0,y=y0-0.4*r/100.0,z=z_low)
        eye_left3 = Point(x=x0-0.2*r/100.0,y=y0-0.2*r/100.0,z=z_low)
        eye_left_end = Point(x=x0-0.2*r/100.0,y=y0-0.2*r/100.0,z=z_high)

        eye_right_st = Point(x=x0-0.2*r/100.0,y=y0+0.2*r/100.0,z=z_high)
        eye_right1 = Point(x=x0-0.2*r/100.0,y=y0+0.2*r/100.0,z=z_low)
        eye_right2 = Point(x=x0-0.4*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        eye_right3 = Point(x=x0-0.2*r/100.0,y=y0+0.6*r/100.0,z=z_low)
        eye_right_end = Point(x=x0-0.2*r/100.0,y=y0+0.6*r/100.0,z=z_high)

        mouth_st = Point(x=x0+0.4*r/100.0,y=y0+0.4*r/100.0,z=z_high)
        mouth1 = Point(x=x0+0.4*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        mouth2 = Point(x=x0+0.6*r/100.0,y=y0,z=z_low)
        mouth3 = Point(x=x0+0.4*r/100.0,y=y0-0.4*r/100.0,z=z_low)
        mouth_end = Point(x=x0+0.4*r/100.0,y=y0-0.4*r/100.0,z=z_high)

        angles_1 = Ik_solution(eye_left_st,q)
        traj.add_point(angles_1, time+1)
        angles_1 = Ik_solution(eye_left1,q)
        traj.add_point(angles_1, time+2)
        angles_1 = Ik_solution(eye_left2,q)
        traj.add_point(angles_1, time+3)
        angles_1 = Ik_solution(eye_left3,q)
        traj.add_point(angles_1, time+4)
        angles_1 = Ik_solution(eye_left_end,q)
        traj.add_point(angles_1, time+5)

        angles_1 = Ik_solution(eye_right_st,q)
        traj.add_point(angles_1, time+6)
        angles_1 = Ik_solution(eye_right1,q)
        traj.add_point(angles_1, time+7)
        angles_1 = Ik_solution(eye_right2,q)
        traj.add_point(angles_1, time+8)
        angles_1 = Ik_solution(eye_right3,q)
        traj.add_point(angles_1, time+9)
        angles_1 = Ik_solution(eye_right_end,q)
        traj.add_point(angles_1, time+10)

        angles_1 = Ik_solution(mouth_st,q)
        traj.add_point(angles_1, time+11)
        angles_1 = Ik_solution(mouth1,q)
        traj.add_point(angles_1, time+12)
        angles_1 = Ik_solution(mouth2,q)
        traj.add_point(angles_1, time+13)
        angles_1 = Ik_solution(mouth3,q)
        traj.add_point(angles_1, time+14)
        angles_1 = Ik_solution(mouth_end,q)
        traj.add_point(angles_1, time+15)

        angles_end1 = Ik_solution(p0,q)
        traj.add_point(angles_end1, time + 16)

    elif keyboard_str == 2:
        #draw a circle
        # draw_circle(0, 0, 1)

        eye_left_st = Point(x=x0-0.4*r/100.0,y=y0-0.6*r/100.0,z=z_high)
        eye_left1 = Point(x=x0-0.4*r/100.0,y=y0-0.6*r/100.0,z=z_low)
        eye_left2 = Point(x=x0-0.2*r/100.0,y=y0-0.4*r/100.0,z=z_low)
        eye_left3 = Point(x=x0-0.4*r/100.0,y=y0-0.2*r/100.0,z=z_low)
        eye_left_end = Point(x=x0-0.4*r/100.0,y=y0-0.2*r/100.0,z=z_high)

        eye_right_st = Point(x=x0-0.4*r/100.0,y=y0+0.2*r/100.0,z=z_high)
        eye_right1 = Point(x=x0-0.4*r/100.0,y=y0+0.2*r/100.0,z=z_low)
        eye_right2 = Point(x=x0-0.2*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        eye_right3 = Point(x=x0-0.4*r/100.0,y=y0+0.6*r/100.0,z=z_low)
        eye_right_end = Point(x=x0-0.4*r/100.0,y=y0+0.6*r/100.0,z=z_high)

        mouth_st = Point(x=x0+0.6*r/100.0,y=y0+0.4*r/100.0,z=z_high)
        mouth1 = Point(x=x0+0.6*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        mouth2 = Point(x=x0+0.4*r/100.0,y=y0,z=z_low)
        mouth3 = Point(x=x0+0.6*r/100.0,y=y0-0.4*r/100.0,z=z_low)
        mouth_end = Point(x=x0+0.6*r/100.0,y=y0-0.4*r/100.0,z=z_high)

        angles_1 = Ik_solution(eye_left_st,q)
        traj.add_point(angles_1, time+1)
        angles_1 = Ik_solution(eye_left1,q)
        traj.add_point(angles_1, time+2)
        angles_1 = Ik_solution(eye_left2,q)
        traj.add_point(angles_1, time+3)
        angles_1 = Ik_solution(eye_left3,q)
        traj.add_point(angles_1, time+4)
        angles_1 = Ik_solution(eye_left_end,q)
        traj.add_point(angles_1, time+5)

        angles_1 = Ik_solution(eye_right_st,q)
        traj.add_point(angles_1, time+6)
        angles_1 = Ik_solution(eye_right1,q)
        traj.add_point(angles_1, time+7)
        angles_1 = Ik_solution(eye_right2,q)
        traj.add_point(angles_1, time+8)
        angles_1 = Ik_solution(eye_right3,q)
        traj.add_point(angles_1, time+9)
        angles_1 = Ik_solution(eye_right_end,q)
        traj.add_point(angles_1, time+10)

        angles_1 = Ik_solution(mouth_st,q)
        traj.add_point(angles_1, time+11)
        angles_1 = Ik_solution(mouth1,q)
        traj.add_point(angles_1, time+12)
        angles_1 = Ik_solution(mouth2,q)
        traj.add_point(angles_1, time+13)
        angles_1 = Ik_solution(mouth3,q)
        traj.add_point(angles_1, time+14)
        angles_1 = Ik_solution(mouth_end,q)
        traj.add_point(angles_1, time+15)

        angles_end1 = Ik_solution(p0,q)
        traj.add_point(angles_end1, time + 16)

        #disgust
    elif keyboard_str == 3:

        eyebrow_left_st = Point(x=x0-0.7*r/100.0,y=y0-0.6*r/100.0,z=z_high)
        eyebrow_left1 = Point(x=x0-0.7*r/100.0,y=y0-0.6*r/100.0,z=z_low)
        eyebrow_left2 = Point(x=x0-0.6*r/100.0,y=y0-0.4*r/100.0,z=z_low)
        eyebrow_left3 = Point(x=x0-0.7*r/100.0,y=y0-0.2*r/100.0,z=z_low)
        eyebrow_left_end = Point(x=x0-0.7*r/100.0,y=y0-0.2*r/100.0,z=z_high)

        eyebrow_right_st = Point(x=x0-0.7*r/100.0,y=y0+0.2*r/100.0,z=z_high)
        eyebrow_right1 = Point(x=x0-0.7*r/100.0,y=y0+0.2*r/100.0,z=z_low)
        eyebrow_right2 = Point(x=x0-0.6*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        eyebrow_right3 = Point(x=x0-0.7*r/100.0,y=y0+0.6*r/100.0,z=z_low)
        eyebrow_right_end = Point(x=x0-0.7*r/100.0,y=y0+0.6*r/100.0,z=z_high)

        mouth_st = Point(x=x0+0.2*r/100.0,y=y0-0.4*r/100.0,z=z_high)
        mouth1 = Point(x=x0+0.2*r/100.0,y=y0-0.4*r/100.0,z=z_low)
        mouth3 = Point(x=x0+0.6*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        mouth_end = Point(x=x0+0.6*r/100.0,y=y0+0.4*r/100.0,z=z_high)

        angles_1 = Ik_solution(eyebrow_left_st,q)
        traj.add_point(angles_1, time+1)
        angles_1 = Ik_solution(eyebrow_left1,q)
        traj.add_point(angles_1, time+2)
        angles_1 = Ik_solution(eyebrow_left2,q)
        traj.add_point(angles_1, time+3)
        angles_1 = Ik_solution(eyebrow_left3,q)
        traj.add_point(angles_1, time+4)
        angles_1 = Ik_solution(eyebrow_left_end,q)
        traj.add_point(angles_1, time+5)

        angles_1 = Ik_solution(eyebrow_right_st,q)
        traj.add_point(angles_1, time+6)
        angles_1 = Ik_solution(eyebrow_right1,q)
        traj.add_point(angles_1, time+7)
        angles_1 = Ik_solution(eyebrow_right2,q)
        traj.add_point(angles_1, time+8)
        angles_1 = Ik_solution(eyebrow_right3,q)
        traj.add_point(angles_1, time+9)
        angles_1 = Ik_solution(eyebrow_right_end,q)
        traj.add_point(angles_1, time+10)

        angles_1 = Ik_solution(mouth_st,q)
        traj.add_point(angles_1, time+11)
        angles_1 = Ik_solution(mouth1,q)
        traj.add_point(angles_1, time+12)
        angles_1 = Ik_solution(mouth3,q)
        traj.add_point(angles_1, time+13)
        angles_1 = Ik_solution(mouth_end,q)
        traj.add_point(angles_1, time+14)

        # draw_circle(-0.4, 0.4, 0.15, time+15)
        offset_x = -0.4
        offset_y = 0.4
        offset_r = 0.15
        # x0 = x0 + 0.05
        # y0 = y0 +0.05
        time = time + 15
        r_m = offset_r*r
        x0_m = x0+offset_x*r/100
        y0_m = y0+offset_y*r/100
        start_point = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_high)
        p1 = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_low)
        angles_start0 = Ik_solution(start_point,q)
        traj.add_point(angles_start0, time + 1)
        angles_start1 = Ik_solution(p1,q)
        traj.add_point(angles_start1, time + 2)
        for t in range(1, int(offset_r*100)+1):
            theta = t*2*3.1415926/(offset_r*100)
            yp = r_m*math.sin(theta)
            xp = r_m*math.cos(theta)
            p2 = Point(x=x0_m+xp/100.0,y=y0_m+yp/100.0,z=z_low)
            angles = Ik_solution(p2,q)
            traj.add_point(angles, time +2+0.1*t)

        angles_end0 = Ik_solution(start_point,q)
        traj.add_point(angles_end0, time + 5)
        angles_end1 = Ik_solution(p0,q)
        traj.add_point(angles_end1, time + 6)

        offset_x = -0.4
        offset_y = -0.4
        offset_r = 0.15
        time = time + 15
        r_m = offset_r*r
        x0_m = x0+offset_x*r/100
        y0_m = y0+offset_y*r/100
        start_point = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_high)
        p1 = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_low)
        angles_start0 = Ik_solution(start_point,q)
        traj.add_point(angles_start0, time + 1)
        angles_start1 = Ik_solution(p1,q)
        traj.add_point(angles_start1, time + 2)
        for t in range(1, int(offset_r*100+2)):
            theta = t*2*3.1415926/(offset_r*100)
            yp = r_m*math.sin(theta)
            xp = r_m*math.cos(theta)
            p2 = Point(x=x0_m+xp/100.0,y=y0_m+yp/100.0,z=z_low)
            angles = Ik_solution(p2,q)
            traj.add_point(angles, time + 2+0.1*t)

        angles_end0 = Ik_solution(start_point,q)
        traj.add_point(angles_end0, time + 5)
        angles_end1 = Ik_solution(p0,q)
        traj.add_point(angles_end1, time + 6)
        # draw_circle(-0.4, -0.4, 0.15, time+20)
        # BaxterMovement(p0,q)

    #suprise
    elif keyboard_str == 4:
        #draw a circle
        # draw_circle(0, 0, 1)

        eye_left_st = Point(x=x0-0.6*r/100.0,y=y0-0.6*r/100.0,z=z_high)
        eye_left1 = Point(x=x0-0.6*r/100.0,y=y0-0.6*r/100.0,z=z_low)
        eye_left2 = Point(x=x0-0.4*r/100.0,y=y0-0.2*r/100.0,z=z_low)
        eye_left3 = Point(x=x0-0.2*r/100.0,y=y0-0.6*r/100.0,z=z_low)
        eye_left_end = Point(x=x0-0.2*r/100.0,y=y0-0.6*r/100.0,z=z_high)

        eye_right_st = Point(x=x0-0.6*r/100.0,y=y0+0.6*r/100.0,z=z_high)
        eye_right1 = Point(x=x0-0.6*r/100.0,y=y0+0.6*r/100.0,z=z_low)
        eye_right2 = Point(x=x0-0.4*r/100.0,y=y0+0.2*r/100.0,z=z_low)
        eye_right3 = Point(x=x0-0.2*r/100.0,y=y0+0.6*r/100.0,z=z_low)
        eye_right_end = Point(x=x0-0.2*r/100.0,y=y0+0.6*r/100.0,z=z_high)

        angles_1 = Ik_solution(eye_left_st,q)
        traj.add_point(angles_1, time+1)
        angles_1 = Ik_solution(eye_left1,q)
        traj.add_point(angles_1, time+2)
        angles_1 = Ik_solution(eye_left2,q)
        traj.add_point(angles_1, time+3)
        angles_1 = Ik_solution(eye_left3,q)
        traj.add_point(angles_1, time+4)
        angles_1 = Ik_solution(eye_left_end,q)
        traj.add_point(angles_1, time+5)

        angles_1 = Ik_solution(eye_right_st,q)
        traj.add_point(angles_1, time+6)
        angles_1 = Ik_solution(eye_right1,q)
        traj.add_point(angles_1, time+7)
        angles_1 = Ik_solution(eye_right2,q)
        traj.add_point(angles_1, time+8)
        angles_1 = Ik_solution(eye_right3,q)
        traj.add_point(angles_1, time+9)
        angles_1 = Ik_solution(eye_right_end,q)
        traj.add_point(angles_1, time+10)

        # draw_circle(0.6, 0, 0.2)
        offset_x = 0.6
        offset_y = 0
        offset_r = 0.2
        time = time + 10
        r_m = offset_r*r
        x0_m = x0+offset_x*r/100
        y0_m = y0+offset_y*r/100
        start_point = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_high)
        p1 = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_low)
        angles_start0 = Ik_solution(start_point,q)
        traj.add_point(angles_start0, time + 1)
        angles_start1 = Ik_solution(p1,q)
        traj.add_point(angles_start1, time + 2)
        for t in range(1, int(offset_r*100+2)):
            theta = t*2*3.1415926/(offset_r*100)
            yp = r_m*math.sin(theta)
            xp = r_m*math.cos(theta)
            p2 = Point(x=x0_m+xp/100.0,y=y0_m+yp/100.0,z=z_low)
            angles = Ik_solution(p2,q)
            traj.add_point(angles, time + 2+0.1*t)

        angles_end0 = Ik_solution(start_point,q)
        traj.add_point(angles_end0, time + 5)
        angles_end1 = Ik_solution(p0,q)
        traj.add_point(angles_end1, time + 6)

    traj.start()
    traj.wait(100)


def draw_circle(offset_x, offset_y, offset_r, time):

    r = 10
    q = Quaternion(x=-0.4177807149532, y=4.099274143222, z=-0.15996636325311, w=-0.1321823626761)
    q1 = Quaternion(x=-0.5177807149532, y=4.299274143222, z=-0.35996636325311, w=-0.2321823626761)
    z_high = -0.0051726872428+0.01
    z_low = -0.0301726872428+0.01
    x0 = 0.678147548375
    y0 = -0.2908471534
    p0 = Point(x=x0,y=y0,z=z_high)
    p1 = Point(x=x0-0.1,y=y0+0.1,z=z_high)
    control.BaxterMovement(p0,q)

    limb = "right"
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, time)

    start_point = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_high)
    p1 = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_low)
    angles_start0 = Ik_solution(start_point,q)
    traj.add_point(angles_start0, time + 1)
    angles_start1 = Ik_solution(p1,q)
    traj.add_point(angles_start1, time + 2)

    r_m = offset_r*r
    x0_m = x0+offset_x*r/100
    y0_m = y0+offset_y*r/100
    for t in range(1, int(offset_r*100+2)):
        theta = t*2*3.1415926/(offset_r*100)
        yp = r_m*math.sin(theta)
        xp = r_m*math.cos(theta)
        p2 = Point(x=x0_m+xp/100.0,y=y0_m+yp/100.0,z=z_low)
        angles = Ik_solution(p2,q)
        traj.add_point(angles, time + 2+0.1*t)

    angles_end0 = Ik_solution(start_point,q)
    traj.add_point(angles_end0, time + 14)
    angles_end1 = Ik_solution(p0,q)
    traj.add_point(angles_end1, time + 15)
    traj.start()
    traj.wait(17)
    # print "finish circle"
    # control.BaxterMovement(p1,q1)



def main():
    rospy.init_node("baxter_movement", anonymous = True)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    keyboard_str = int(raw_input('Input:'))
    # draw_circle(0, 0, 1, 0)
    # limb = "right"
    # traj = Trajectory(limb)
    draw_face(keyboard_str, 0, 0, 1)
    # control.BaxterMovement()

    # control.draw_face(keyboard_str)
    # traj.start()
    # traj.wait(100)

if __name__ =='__main__':
    main()
