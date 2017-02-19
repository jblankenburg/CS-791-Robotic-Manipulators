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

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from myRobot import MyRobot
import numpy as np

def joint_st():
    pub = rospy.Publisher("joint_state", JointState, queue_size=10)
    rospy.init_node('joint_st', anonymous=True)
    rate = rospy.Rate(1) # 1hz


    while not rospy.is_shutdown():

        joint = JointState()

        joint.header.frame_id = '/map'
        joint.header.stamp = rospy.Time.now()

        joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']
        joint.name = joint_names
        joint.position = [1.0*np.random.rand(1,1), 
                            1.0*np.random.rand(1,1),
                            1.0*np.random.rand(1,1),
                            1.0*np.random.rand(1,1),
                            1.0*np.random.rand(1,1),
                            1.0*np.random.rand(1,1),
                            1.0*np.random.rand(1,1)] 

        rospy.loginfo(joint)
        pub.publish(joint)

        rate.sleep()

if __name__ == '__main__':
    try:
        joint_st()
    except rospy.ROSInterruptException:
        pass


