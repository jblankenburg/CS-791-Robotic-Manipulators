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
import sys
import getopt
from std_msgs.msg import String
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from myRobot import MyRobot
import numpy as np

def joint_st(num_joints):
    pub = rospy.Publisher("joint_state", JointState, queue_size=10)
    rospy.init_node('joint_st', anonymous=True)
    rate = rospy.Rate(1) # 1hz


    while not rospy.is_shutdown():

        joint = JointState()

        joint.header.frame_id = '/map'
        joint.header.stamp = rospy.Time.now()

        names = set_joint_names(num_joints)
        positions = set_joint_positions(num_joints)
        joint.name = names
        joint.position = positions

        rospy.loginfo(joint)
        pub.publish(joint)

        rate.sleep()



# ------------------------------------------

def set_joint_names(num_joints):

    names = []
    for i in range(1,num_joints+1):
        name = 'joint_{}'.format(i)
        names.extend([name])
    print'\n\n'
    print names
    return names

def set_joint_positions(num_joints):
    positions = []
    for i in range(1,num_joints+1):
        position = 1.0*np.random.rand(1,1)
        positions.extend(position)
    print positions
    return positions

# ------------------------------------------


if __name__ == '__main__':

    argv = sys.argv[1:]

    num_joints = 7
    try:
        opts, args = getopt.getopt(argv,"hn:",["num_joints="])
    except getopt.GetoptError:
        print 'joint_state_pub.py -n <num_joints (int)>'
        print '\t Try -h for help'
        sys.exit(2)
    if opts:
        for opt, arg in opts:
            if opt == '-h':
                print 'joint_state_pub.py -n <num_joints (int)> \n------ Where n is num joints to publish states for\n\t should match num states in bot_file.json passed to robot_pub_sub.py'
                print '\t   IF NOT PROVIDED: Default = 7'
                sys.exit()
            elif opt in ("-n", "--num_joints"):
                num_joints = int(arg)
                print( 'Will publish joint states for {} joints!'.format(num_joints))
    else:
        print 'No number of joints provided. Default = 7.'


    try:
        joint_st(num_joints)
    except rospy.ROSInterruptException:
        pass


