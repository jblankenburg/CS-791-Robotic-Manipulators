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

def vel_test():
    jointCmdPub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)
    rospy.init_node('vel_test')
    rate = rospy.Rate(1000)
    i = 0
    while not rospy.is_shutdown():
      jointCmd = JointCommand()
      jointCmd.mode = JointCommand.VELOCITY_MODE
      jointCmd.names.append('right_s0')
      jointCmd.names.append('right_s1')
      jointCmd.names.append('right_e0')
      jointCmd.names.append('right_e1')
      jointCmd.names.append('right_w0')
      jointCmd.names.append('right_w1')
      jointCmd.names.append('right_w2')
      if i < 4000:
      	vel = i / 1000.0
        jointCmd.command.append( vel )
      else:
        jointCmd.command.append(0.0)
      i = i+1
      jointCmdPub.publish(jointCmd)
      rospy.loginfo("%s",jointCmd)
      rospy.set_param('time', i)
      rate.sleep()

if __name__ == '__main__':
  try:
    vel_test()
  except rospy.ROSInterruptException:
    pass


# write a listener for the given command message

# correctly determine the constants for a cubic trajectory

# correctly output the joint angles at each time step

# part 2