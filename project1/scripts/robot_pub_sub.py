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
import numpy as np
from std_msgs.msg import String
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from myRobot import MyRobot

def talker(data = []):

    print(data)
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    # bot1 = MyRobot("/home/janelle/catkin_manip/src/project1/scripts/two_link_planar.json")
    # bot = MyRobot("/home/janelle/catkin_manip/src/CS-791-Robotic-Manipulators/project1/scripts/DLR_manipulator.json")
    bot_file = rospy.get_param("bot_file")
    bot = MyRobot(bot_file)
    pub_robot(bot, pub, data)

    # while not rospy.is_shutdown():

    #     pub_robot(bot, pub)
    #     rate.sleep()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    talker(data.position)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('joint_state', JointState, callback)

    # # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

# ---------------------------------------------------------------------------------

def pub_robot(bot, pub, data):
    # loop through all joints and publish them and their links
    points = []
    p = Point(); 
    p.x = 0; p.y = 0; p.z = 0;
    points.append(p)
    # publish remainders
    print('asdfasdfasdfasdfasdf')
    print(len(bot.dhParams))
    for k in range(0,len(bot.dhParams)+1):

        if data and k < len(bot.dhParams):
            # set the new orientation
            print('\n\n\n\n')
            # print(data)
            print(k) 
            theta = bot.dhParams[k][3]
            print(theta)
            bot.dhParams[k][3] = data[k]
            theta2 = bot.dhParams[k][3]
            print(theta2)

        # get translation and rotation
        print('--------------------')
        # print(k)
        pos = bot.getTranslation(0,k)
        p = Point(); 
        p.x = pos[0]; p.y = pos[1]; p.z = pos[2];
        R = bot.getRotationMatrix(0,k)
        # ori = decomposeRotMat(R)
        ori = rotMat2Quat(R)
        # print(pos)
        # print(R)
        # print(ori)
        
        # publish cube and add points to link
        pub_cube(pos, ori, pub, k)
        points.append(p)
        # print('points')
        # print(points)
        print('--------------------\n\n\n\n\n')
        print(type(bot.dhParams))

    # publish link
    pub_line_strip(points, pub, k+1)

# def decomposeRotMat(R):
#     ori = np.zeros((3,1))
#     ori[0] = np.arctan2(R[2,1],R[2,2])
#     ori[1] = np.arctan2(-R[2,0],np.sqrt((R[2,1]*R[2,1])+(R[2,2]*R[2,2])))
#     ori[2] = np.arctan2(R[1,0],R[0,0])
#     return ori

def rotMat2Quat(R):
    ori = np.zeros((4,1))
    ori[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1)
    ori[1] = 0.5*(np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1))
    ori[2] = 0.5*(np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1))
    ori[3] = 0.5*(np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1))
    return ori

def pub_cube(pos, ori, pub, idx):
    cube = Marker()
    cube.header.frame_id = "/map"
    cube.header.stamp = rospy.Time.now()
    cube.type = cube.CUBE
    cube.action = cube.ADD
    cube.pose.position.x = pos[0]
    cube.pose.position.y = pos[1]
    cube.pose.position.z = pos[2]
    cube.pose.orientation.w = ori[0]
    cube.pose.orientation.x = ori[1]
    cube.pose.orientation.y = ori[2]
    cube.pose.orientation.z = ori[3]
    cube.ns = 'cube'
    cube.scale.x = 0.01
    cube.scale.y = 0.01
    cube.scale.z = 0.01
    cube.color.b = 1.0
    cube.color.r = float(2*idx)/10
    print(float(2*idx)/10)
    cube.color.a = 1.0
    cube.id = idx
    # cube.lifetime = rospy.Duration()
    rospy.loginfo('\nCube Pos:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z))
    rospy.loginfo('\nCube Ori:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z))
    pub.publish(cube)
    return cube

def pub_line_strip(pnts, pub, idx):
    line_strip = Marker()
    line_strip.header.frame_id = "/map"
    line_strip.header.stamp = rospy.Time.now()
    line_strip.type = line_strip.LINE_STRIP
    line_strip.action = line_strip.ADD
    line_strip.points = pnts
    line_strip.ns = 'line_strip'
    line_strip.scale.x = 0.005
    line_strip.color.g = 1.0
    line_strip.color.a = 1.0
    line_strip.id = idx;
    # line_strip.lifetime = rospy.Duration()
    # rospy.loginfo('\n\nLine Strip:\n\tx1: {}\ty1: {}\tz1: {}\n\tx2: {}\ty2: {}\tz2: {}\n\tx3: {}\ty3: {}\tz3: {}',line_strip.points[0].x,line_strip.points[0].y,line_strip.points[0].z,line_strip.points[1].x,line_strip.points[1].y,line_strip.points[1].z,line_strip.points[2].x,line_strip.points[2].y,line_strip.points[2].z,)
    rospy.loginfo('\nLine Strip:\n{}'.format(line_strip.points))
    pub.publish(line_strip)
    return line_strip

# ---------------------------------------------------------------------------------

#  usage:    -b <bot_file.json>
if __name__ == '__main__':

    argv = sys.argv[1:]

    bot_file = ''
    try:
        opts, args = getopt.getopt(argv,"hb:",["bot_file="])
    except getopt.GetoptError:
        print 'robot_pub_sub.py -b <bot_file>'
        sys.exit(2)
    if opts:
        for opt, arg in opts:
            if opt == '-h':
                print 'robot_pub_sub.py -b <bot_file.json>'
                sys.exit()
            elif opt in ("-b", "--bot_file"):
                bot_file = arg
                print( 'Bot file is: {}'.format(bot_file))
    else:
        print 'No bot file provided.\n Try robot_pub_sub.py -h for help.\n  Exiting.'
        sys.exit(2)

    rospy.init_node('myRobot', anonymous=True)
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
    rospy.set_param('bot_file', bot_file)
    talker()
    listener()
    rospy.spin()



