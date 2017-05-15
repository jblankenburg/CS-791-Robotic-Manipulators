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
from traj_msgs.msg import PieceMovement

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

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

        move_from = data.move_from
        move_to = data.move_to
        
        print "Moving piece from: {0}".format(move_from)
        print "Moving pieve to:   {0}".format(move_to)
        
        move_to_square(self, move_from, move_to)

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.Subscriber("pieceMovement", PieceMovement, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def move_to_square(robot, label_from, label_to):

    # compute offset from label
    offset_f = compute_label_offset(robot,label_from)
    # compute offset from label
    offset_t = compute_label_offset(robot,label_to)

    now = rospy.Time(0)
    t = tf.TransformListener(True, rospy.Duration(10.0))
    # transform = tf.Transformer(True, rospy.Duration(10.0))
    t.waitForTransform("/base", "/checkerboard", now,rospy.Duration(3.0));
    pose = t.lookupTransform("/base", "/checkerboard", now)

    print "The pose of the chessboard is: {0}".format(pose)
    quat_t = [1, 0, 0, 0]
    pose_t = [pose[0],quat_t]
    print "reversed pose is {0}".format(quat_t)

    pick_place(robot, pose_t, offset_t, offset_f)    


def pick_place(robot, pose, offset_t, offset_f):


    #--- Set pose goal for just above position of peice 
    pose_temp = ( (pose[0][0],pose[0][1],pose[0][2]+0.1), pose[1])
    print(pose_temp)

    #------- PICK --------------------

    print("Moving to just above piece to pick")
    move_to_pose(robot, pose_temp, offset_f)

    #--- open gripper
    print("Opening gripper")
    robot.left_gripper.open()
    rospy.sleep(1)

    #--- Set pose goal for exact piece
    print("Moving to just exact loc of piece to pick")
    move_to_pose(robot, pose, offset_f)

    #--- close gripper
    print("Closing gripper")
    robot.left_gripper.close()
    rospy.sleep(1)

    #--- Set pose goal for just above position of peice 
    print("Moving to just above piece to pick")
    move_to_pose(robot, pose_temp, offset_f)


    #------- PLACE --------------------

    print("Moving to just above loc to place")
    move_to_pose(robot, pose_temp, offset_t)

    #--- Set pose goal for exact piece
    print("Moving to just exact loc to place piece")
    move_to_pose(robot, pose, offset_t)

    #--- open gripper
    print("Opening gripper")
    robot.left_gripper.open()
    rospy.sleep(1)

    #--- Set pose goal for just above position of peice 
    print("Moving to just above loc to place")
    move_to_pose(robot, pose_temp, offset_t)



def move_to_pose(robot, pose, offset):
    pose_target = geometry_msgs.msg.Pose()

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

    print("pose {0}".format(pose))
    print("offset {0}".format(offset))
    print("target pose {0}".format(pose_target))

    print("\tPlanning...")
    plan1 = robot.arm.plan()
    rospy.sleep(5)
    print("\tExecuting...")
    robot.arm.go(wait=True)
    rospy.sleep(5)

def compute_label_offset(robot,label):
    # x_axis = ['a','b','c','d','e','f','g','h',]
    # y_axis = [1,2,3,4,5,6,7,8]

    # .0625 is cell size
    k = 0.0625/2
    delta_k = 0.0625

    if label[0] == 'a':
        x_pos = 3*delta_k + k;
    if label[0] == 'b':
        x_pos = 2*delta_k + k;
    if label[0] == 'c':
        x_pos = delta_k + k;
    if label[0] == 'd':
        x_pos = k;
    if label[0] == 'e':
        x_pos = -k;
    if label[0] == 'f':
        x_pos = -delta_k - k;
    if label[0] == 'g':
        x_pos = -2*delta_k - k;
    if label[0] == 'h':
        x_pos = -3*delta_k - k;

    if label[1] == '1':
        y_pos = -3*delta_k - k;
    if label[1] == '2':
        y_pos = -2*delta_k - k;
    if label[1] == '3':
        y_pos = -delta_k - k;
    if label[1] == '4':
        y_pos = -k;
    if label[1] == '5':
        y_pos = k;
    if label[1] == '6':
        y_pos = delta_k + k;
    if label[1] == '7':
        y_pos = 2*delta_k + k;
    if label[1] == '8':
        y_pos = 3*delta_k + k;

    offset = [y_pos, x_pos]

    return offset



# -----------------------------------------------------
# MAIN!!!
def main():
    rospy.init_node('project3_moveit',
            anonymous=True)
    baxter = BaxterMoveit('left_arm')   
    try:
        # move_to_square(baxter,'a1')
        baxter.listener()
    except rospy.ROSInterruptException:
        pass
    baxter.robot_shutdown()


if __name__=='__main__':
    main()
    # rospy.spin()

