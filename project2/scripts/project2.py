#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# create a ROS node that publishes a baxter_core_msgs/JointCommand msg at 1000hz
import sys, time
import numpy as np
import random
import copy
import baxter_interface

import rospy
from math import sin, cos, pi
from baxter_core_msgs.msg import JointCommand
from geometry_msgs.msg import Point
from std_msgs.msg import String

# def vel_test():
#     jointCmdPub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)
#     rospy.init_node('vel_test')
#     rate = rospy.Rate(1000)
#     i = 0
#     while not rospy.is_shutdown():
#       jointCmd = JointCommand()
#       jointCmd.mode = JointCommand.TORQUE_MODE
#       jointCmd.names.append('right_s1')
#       if i < 4000:
#         vel = i / 1000.0
#         jointCmd.command.append( vel )
#       else:
#         jointCmd.command.append(0.0)
#       i = i+1
#       jointCmdPub.publish(jointCmd)
#       rospy.loginfo("%s",jointCmd)
#       rate.sleep()

global_var = 0

# write a listener for the given command message
def callback(data):
    global global_var
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    t = rospy.get_param("time")
    # print(data.command[0])
    # computeConstants(data.command[0], int(t))
    if global_var == 0:
      computeConstants(data.command[0], 4000) # based on joint pub dave gave us, it stops after 4000
      global_var = 1;

    # output joint angles
    setJointAngles(data);

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/robot/limb/right/joint_command", JointCommand, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# correctly determine the constants for a cubic trajectory
def computeConstants( q_s, t):
  # for our case we are assuming we are given an initial velocity, 
  # but it has no acceleration, and we want to end with no velocity or acceleration.

  # want to solve:
  # q_s = a_0
  # q_s_dot = a_1
  # q_f = a_0 + a_1*t + a_2*t^2 + a_3*t^3
  # q_f_dot = a_1 + 2*a_2*t + 3*a_3*t^2

  # we have q_s => we have a_0
  # we assume q_s_dot is 0 => a_1 is 0 
  # then we have two equations and two unknowns (since we assume q_f and q_f_dor are 0)
  # so we can solve this using numpy, forming this as Ax = b:

  #        b                =        A              x
  #  [q_f - a_0 - a_1*t     = [ t^2    t^3     * [ a_2
  #   q_f_dot - a_1     ]       2*t    3*t^2]      a_3]
  
  a_0 = q_s
  a_1 = 0.000001
  q_f = 0
  q_f_dot = 0

  a = np.array([[np.power(t,2), np.power(t,3)], [2*t, 3*np.power(t,2)]])
  b = np.array([q_f - a_0 - a_1*t, q_f_dot - a_1])
  x = np.linalg.solve(a,b)
  # print(a)
  # print(b)
  # print(x)
  # print(np.allclose(np.dot(a,x), b))

  rospy.set_param("a_0", a_0)
  rospy.set_param("a_1", a_1)
  rospy.set_param("a_2", float(x[0]))
  rospy.set_param("a_3", float(x[1]))

# correctly output the joint angles at each time step
def setJointAngles(data):

  # get params
  a_0 = rospy.get_param("a_0")
  a_1 = rospy.get_param("a_1")
  a_2 = rospy.get_param("a_2")
  a_3 = rospy.get_param("a_3")
  t = rospy.get_param("time")

  # compute q_f and q_f_dot?
  q_f = a_0 + a_1*t + a_2*t*t + a_3*t*t*t
  q_f_dot = a_1 + 2*a_2*t + 3*a_3*t*t

  print(q_f)
  print(q_f_dot)



# part 2
# choose your own adventure part 2:
#   publish the instantaneous Jacobian matrix for the baxter at 1000Hz




if __name__ == '__main__':
  try:
    # vel_test()
    listener()
  except rospy.ROSInterruptException:
    pass

