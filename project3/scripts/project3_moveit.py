#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import tf
from baxter_interface import Gripper
import numpy as np
import random

# set up robot class
class BaxterMoveit:

    def __init__(self, limb):

        # init moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)


        # instantiate things
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander(limb)

        # set up gripper things
        # self.right_gripper = Gripper('right')
        self.left_gripper = Gripper('left')  # Assuming only using left gripper right now

        # create pub to pub trajs for RVIZ
        display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory)
        print "============ Waiting for RVIZ..."
        rospy.sleep(5)
        print "============ Starting project "


    def robot_shutdown(self):
        print "============ STOPPING"
        moveit_commander.roscpp_shutdown()



def move_to_square(robot, label):

    # compute offset from label
    offset = compute_label_offset(robot,label)
    print(offset)

    now = rospy.Time(0)
    t = tf.TransformListener(True, rospy.Duration(10.0))
    # transform = tf.Transformer(True, rospy.Duration(10.0))
    t.waitForTransform("/base", "/chessboard", now,rospy.Duration(3.0));
    pose = t.lookupTransform("/base", "/chessboard", now)

    print "The pose of the chessboard is: {0}".format(pose)

    pick_place(robot, pose, offset)    

    # # set up pose goal
    # pose_target = geometry_msgs.msg.Pose()
    # # THIS POSE HAS THE SAME ORIENTATION AS THE CHESSBOARD BUT THE CHESSBOARD IS FACING UP SO I NEED TO GET THE REVERSE?!?!
    # # ALSO NEED TO ADD OFFSETS TO POSITIONS OF PARTS OF CHESSBOARD TO FOUND X,Y
    # # pose_target.orientation.x = pose[1][0]
    # # pose_target.orientation.y = pose[1][1]
    # # pose_target.orientation.z = pose[1][2]
    # # pose_target.orientation.w = pose[1][3]
    # # pose_target.position.x = pose[0][0] + offset[0]
    # # pose_target.position.y = pose[0][1] + offset[1]
    # # pose_target.position.z = pose[0][2]
    # pose_target.orientation.x = 1
    # pose_target.orientation.y = 0
    # pose_target.orientation.z = 0
    # pose_target.orientation.w = 0
    # pose_target.position.x = 0.75 + offset[0]
    # pose_target.position.y = 0 + offset[1]
    # pose_target.position.z = 0

    # robot.arm.set_pose_target(pose_target)
    # robot.arm.set_num_planning_attempts(3);
    # robot.arm.set_planning_time(5.0);
    # robot.arm.set_goal_position_tolerance(0.01)
    # robot.arm.set_goal_orientation_tolerance(0.01)


    # plan1 = robot.arm.plan()
    # print "============ Waiting while RVIZ displays plan1..."
    # rospy.sleep(10)
    # print "============ Waiting while RVIZ executes plan1..."
    # robot.arm.go(wait=True)
    # rospy.sleep(10)


def pick_place(robot, pose, offset):

    #--- Set pose goal for just above position of peice 
    pose_temp = ( (pose[0][0],pose[0][1],pose[0][2]+0.1), pose[1])
    print(pose_temp)

    print("Moving to just above piece")
    move_to_pose(robot, pose_temp, offset)

    #--- open gripper
    print("Opening gripper")
    robot.left_gripper.open()
    rospy.sleep(1)

    #--- Set pose goal for exact piece
    print("Moving to just exact loc of piece")
    move_to_pose(robot, pose, offset)

    #--- close gripper
    print("Closing gripper")
    robot.left_gripper.close()
    rospy.sleep(1)


    #--- Set pose goal for just above position of peice 
    print("Moving to just above piece")
    move_to_pose(robot, pose_temp, offset)


def move_to_pose(robot, pose, offset):
    pose_target = geometry_msgs.msg.Pose()

    # pose_target.orientation.x = 1
    # pose_target.orientation.y = 0
    # pose_target.orientation.z = 0
    # pose_target.orientation.w = 0
    # pose_target.position.x = 0.75 + offset[0]
    # pose_target.position.y = 0 + offset[1]
    # pose_target.position.z = 0

    # want from to be opposite of gripper along x so mult pose quat w/ [1,0,0,0]
    # ori = quaternion_multiply(pose[1], [1,0,0,0])
    pose_target.orientation.x = pose[1][0]
    pose_target.orientation.y = pose[1][1]
    pose_target.orientation.z = pose[1][2]
    pose_target.orientation.w = pose[1][3]
    pose_target.position.x = pose[0][0] + offset[0]
    pose_target.position.y = pose[0][1] + offset[1]
    pose_target.position.z = pose[0][2]

    robot.arm.set_pose_target(pose_target)
    robot.arm.set_num_planning_attempts(3);
    robot.arm.set_planning_time(5.0);
    robot.arm.set_goal_position_tolerance(0.01)
    robot.arm.set_goal_orientation_tolerance(0.01)

    print("\tPlanning...")
    plan1 = robot.arm.plan()
    rospy.sleep(5)
    print("\tExecuting...")
    robot.arm.go(wait=True)
    rospy.sleep(5)

def compute_label_offset(robot,label):
    # x_axis = ['a','b','c','d','e','f','g','h',]
    # y_axis = [1,2,3,4,5,6,7,8]

    if label[0] == 'a':
        x_pos = 0;
    if label[0] == 'b':
        x_pos = 0.1;
    if label[0] == 'c':
        x_pos = 0.2;
    if label[0] == 'd':
        x_pos = 0.3;
    if label[0] == 'e':
        x_pos = 0.4;
    if label[0] == 'f':
        x_pos = 0.5;
    if label[0] == 'g':
        x_pos = 0.6;
    if label[0] == 'h':
        x_pos = 0.7;

    if label[1] == '1':
        y_pos = 0;
    if label[1] == '2':
        y_pos = 0.1;
    if label[1] == '3':
        y_pos = 0.2;
    if label[1] == '4':
        y_pos = 0.3;
    if label[1] == '5':
        y_pos = 0.4;
    if label[1] == '6':
        y_pos = 0.5;
    if label[1] == '7':
        y_pos = 0.6;
    if label[1] == '8':
        y_pos = 0.7;

    offset = [x_pos, y_pos]

    return offset

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

# -----------------------------------------------------
# MAIN!!!
def main():
    rospy.init_node('project3_moveit',
            anonymous=True)
    baxter = BaxterMoveit('left_arm')   
    try:
        move_to_square(baxter,'a1')
    except rospy.ROSInterruptException:
        pass
    baxter.robot_shutdown()


if __name__=='__main__':
    main()
    rospy.spin()

