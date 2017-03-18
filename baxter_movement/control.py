#!/usr/bin/env python

import rospy
import sys
import struct

import copy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Image

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from ar_track_alvar_msgs.msg import AlvarMarkers

from random import randint

from baxter_interface import CHECK_VERSION
import baxter_interface
from baxter_core_msgs.msg import EndpointState

import numpy
import math

def BaxterMovement(p,q):

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
        limb = baxter_interface.Limb('right')
        # limb.set_joint_positions(limb_joints)
        limb.move_to_joint_positions(limb_joints)
        #print "\nIK Joint Solution:\n", limb_joints

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return


def draw_circle(offset_x, offset_y, offset_r):

    r = 10
    q = Quaternion(x=-0.4177807149532, y=4.099274143222, z=-0.15996636325311, w=-0.1321823626761)
    z_high = -0.0051726872428+0.01
    z_low = -0.0301726872428+0.01
    x0 = 0.678147548375
    y0 = -0.2908471534
    p0 = Point(x=x0,y=y0,z=z_high)

    start_point = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_high)
    p1 = Point(x=x0+(offset_x+offset_r)*r/100.0,y=y0+offset_y*r/100.0,z=z_low)
    BaxterMovement(start_point,q)
    BaxterMovement(p1,q)
    r_m = offset_r*r
    x0_m = x0+offset_x*r/100
    y0_m = y0+offset_y*r/100
    for t in range(1, int(offset_r*100+1)):
        theta = t*2*3.1415926/(offset_r*100)
        yp = r_m*math.sin(theta)
        xp = r_m*math.cos(theta)
        p2 = Point(x=x0_m+xp/100.0,y=y0_m+yp/100.0,z=z_low)
        # BaxterMovement(p1,q)
        BaxterMovement(p2,q)
        # p1 = p2
    BaxterMovement(start_point,q)
    BaxterMovement(p0,q)

def draw_face(keyboard_str):

    r = 10
    q = Quaternion(x=-0.4177807149532, y=4.099274143222, z=-0.15996636325311, w=-0.1321823626761)
    z_high = -0.0051726872428+0.01
    z_low = -0.0301726872428+0.01
    x0 = 0.678147548375
    y0 = -0.2908471534
    p0 = Point(x=x0,y=y0,z=z_high)
    BaxterMovement(p0,q)

    # happy
    if keyboard_str == 1:
        #draw a circle
        draw_circle(0, 0, 1)
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

        BaxterMovement(eye_left_st,q)
        BaxterMovement(eye_left1,q)
        BaxterMovement(eye_left2,q)
        BaxterMovement(eye_left3,q)
        BaxterMovement(eye_left_end,q)

        BaxterMovement(eye_right_st,q)
        BaxterMovement(eye_right1,q)
        BaxterMovement(eye_right2,q)
        BaxterMovement(eye_right3,q)
        BaxterMovement(eye_right_end,q)

        BaxterMovement(mouth_st,q)
        BaxterMovement(mouth1,q)
        BaxterMovement(mouth2,q)
        BaxterMovement(mouth3,q)
        BaxterMovement(mouth_end,q)
        BaxterMovement(p0,q)

    #sad
    elif keyboard_str == 2:
        #draw a circle
        draw_circle(0, 0, 1)

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

        BaxterMovement(eye_left_st,q)
        BaxterMovement(eye_left1,q)
        BaxterMovement(eye_left2,q)
        BaxterMovement(eye_left3,q)
        BaxterMovement(eye_left_end,q)

        BaxterMovement(eye_right_st,q)
        BaxterMovement(eye_right1,q)
        BaxterMovement(eye_right2,q)
        BaxterMovement(eye_right3,q)
        BaxterMovement(eye_right_end,q)

        BaxterMovement(mouth_st,q)
        BaxterMovement(mouth1,q)
        BaxterMovement(mouth2,q)
        BaxterMovement(mouth3,q)
        BaxterMovement(mouth_end,q)
        BaxterMovement(p0,q)

    #disgust
    elif keyboard_str == 3:
        #draw a circle
        draw_circle(0, 0, 1)

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
        mouth2 = Point(x=x0+0.5*r/100.0,y=y0-0.2*r/100.0,z=z_low)
        mouth3 = Point(x=x0+0.6*r/100.0,y=y0+0.4*r/100.0,z=z_low)
        mouth_end = Point(x=x0+0.6*r/100.0,y=y0+0.4*r/100.0,z=z_high)

        BaxterMovement(eyebrow_left_st,q)
        BaxterMovement(eyebrow_left1,q)
        BaxterMovement(eyebrow_left2,q)
        BaxterMovement(eyebrow_left3,q)
        BaxterMovement(eyebrow_left_end,q)

        BaxterMovement(eyebrow_right_st,q)
        BaxterMovement(eyebrow_right1,q)
        BaxterMovement(eyebrow_right2,q)
        BaxterMovement(eyebrow_right3,q)
        BaxterMovement(eyebrow_right_end,q)

        draw_circle(-0.4, 0.4, 0.15)
        draw_circle(-0.4, -0.4, 0.15)

        BaxterMovement(mouth_st,q)
        BaxterMovement(mouth1,q)
        BaxterMovement(mouth2,q)
        BaxterMovement(mouth3,q)
        BaxterMovement(mouth_end,q)
        BaxterMovement(p0,q)

    #suprise
    elif keyboard_str == 4:
        #draw a circle
        draw_circle(0, 0, 1)

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

        BaxterMovement(eye_left_st,q)
        BaxterMovement(eye_left1,q)
        BaxterMovement(eye_left2,q)
        BaxterMovement(eye_left3,q)
        BaxterMovement(eye_left_end,q)

        BaxterMovement(eye_right_st,q)
        BaxterMovement(eye_right1,q)
        BaxterMovement(eye_right2,q)
        BaxterMovement(eye_right3,q)
        BaxterMovement(eye_right_end,q)

        draw_circle(0.6, 0, 0.2)

def main():

    rospy.init_node("baxter_movement", anonymous = True)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # rospy.Subscriber('rcampub',Point,camcb)# change the node name

    right = baxter_interface.Limb('right')
    right.set_joint_position_speed(1)

    keyboard_str = int(raw_input('Input:'))
    draw_face(keyboard_str)

# if __name__=='__main__':
#     main()
