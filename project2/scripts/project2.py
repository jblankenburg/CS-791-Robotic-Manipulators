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
from traj_msgs.msg import TrajectoryCommand
from sensor_msgs.msg import JointState

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

CONST_FLAG = 0
TRAJ_COM = None
JOINT_COM = None

# write a listener for the given command message
def traj_callback(data):
    global TRAJ_COM
    rospy.loginfo("traj callback!!!!")
    TRAJ_COM = data 

# write a listener for the given command message
def joint_callback(data):
    global CONST_FLAG
    global TRAJ_COM
    JOINT_COM = data
    rospy.loginfo("Joint callback!!!!")

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    # print(data.command[0])
    # computeConstants(data.command[0], int(t))
    q_s = []
    q_s_dot = []
    for name in TRAJ_COM.names:
      for ind in range(len(data.name)):
        if data.name[ind] == name:
          q_s.append(data.position[ind]) 
          q_s_dot.append(data.velocity[ind])
    # print(q_s)
    # print(q_s_dot)

    if CONST_FLAG == 0:
      computeConstants(q_s, q_s_dot, TRAJ_COM.q_final, TRAJ_COM.qdot_final, TRAJ_COM.t_k) # based on joint pub dave gave us, it stops after 4000
      CONST_FLAG = 1;

    # output joint angles
    setJointAngles(TRAJ_COM.t_k);

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # subscribe to the trajectory command topic for time and joints and q's
    rospy.loginfo("THIS!!!!")
    rospy.Subscriber("/trajectory_command", TrajectoryCommand, traj_callback)
    rospy.Subscriber("/robot/joint_states", JointState, joint_callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# correctly determine the constants for a cubic trajectory
def computeConstants( q_s, q_s_dot, q_f, q_f_dot, t_dur):
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


  t = t_dur.data.secs



  a_0 = np.zeros([len(q_s),1])  
  a_1 = np.zeros([len(q_s),1])  
  a_2 = np.zeros([len(q_s),1])  
  a_3 = np.zeros([len(q_s),1])  



  for i in range(len(q_s)):
    a_0[i] = q_s[i]
    a_1[i] = q_s_dot[i]
    # q_f = 0
    # q_f_dot = 0

    a = np.array([[np.power(t,2), np.power(t,3)], [2*t, 3*np.power(t,2)]])
    b = np.array([q_f[i] - a_0[i] - a_1[i]*t, q_f_dot[i] - a_1[i]])
    x = np.linalg.solve(a,b)
    a_2[i] = float(x[0])
    a_3[i] = float(x[1])
    # print(a)
    # print(b)
    # print(x)
    # print(np.allclose(np.dot(a,x), b))

  # print('\n\n\n')
  # print(t)
  # print(len(q_s))
  # print(a_0.tolist())
  # print('\n\n\n')

  rospy.set_param("a_0", a_0.tolist())
  rospy.set_param("a_1", a_1.tolist())
  rospy.set_param("a_2", a_2.tolist())
  rospy.set_param("a_3", a_3.tolist())


# correctly output the joint angles at each time step
def setJointAngles(t_dur):

  t = t_dur.data.secs

  # get params
  a_0 = rospy.get_param("a_0")
  a_1 = rospy.get_param("a_1")
  a_2 = rospy.get_param("a_2")
  a_3 = rospy.get_param("a_3")

  print(len(a_0))

  q_f = np.zeros(len(a_0))
  q_f_dot = np.zeros(len(a_0))
  for i in range(len(a_0)):
    # compute q_f and q_f_dot?
    # print(a_0[i])
    # print(a_1[i])
    # print(t)
    # print(a_1[i][0]*t)
    # print(a_0[i] + a_1[i]*t + a_2[i]*t*t)
    # print(a_0[i] + a_1[i]*t + a_2[i]*t*t + a_3[i]*t*t*t)
    q_f[i] = a_0[i][0] + a_1[i][0]*t + a_2[i][0]*t*t + a_3[i][0]*t*t*t
    q_f_dot[i] = a_1[i][0] + 2*a_2[i][0]*t + 3*a_3[i][0]*t*t

    print('\n\n\n')
    rospy.loginfo(q_f)
    # print(q_f_dot)
    print('\n\n\n')
    vel_test(q_f)


def vel_test(q_f):
    jointCmdPub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)
    # rospy.init_node('vel_test')
    rate = rospy.Rate(1000)
    i = 0
    while not rospy.is_shutdown():
      jointCmd = JointCommand()
      jointCmd.mode = JointCommand.VELOCITY_MODE

      for i in range(len(q_f)):
        jointCmd.names.append(TRAJ_COM.names[i])
        jointCmd.command.append(q_f[i])

      # TrajectoryComamnd. = Duration
      jointCmdPub.publish(jointCmd)
      rospy.loginfo("%s",jointCmd)
      rospy.loginfo(q_f)
      rate.sleep()



# part 2
# choose your own adventure part 2:
#   publish the instantaneous Jacobian matrix for the baxter at 1000Hz




if __name__ == '__main__':
  try:
    # vel_test()
    listener()
  except rospy.ROSInterruptException:
    pass

