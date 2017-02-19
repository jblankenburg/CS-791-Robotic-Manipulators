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
from myRobot import MyRobot

def talker():
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    bot = MyRobot("/home/janelle/catkin_manip/src/project1/scripts/robot_test.json")

    while not rospy.is_shutdown():

        # CUBE Marker
        cube = Marker()
        cube.header.frame_id = "/map"
        cube.header.stamp = rospy.Time.now()
        cube.type = cube.CUBE
        cube.action = cube.ADD
        cube.pose.position.x = 0.5
        cube.pose.position.y = 1
        cube.pose.position.z = 1.5
        cube.ns = 'cube'
        cube.scale.x = 0.1
        cube.scale.y = 0.1
        cube.scale.z = 0.1
        cube.color.b = 1.0
        cube.color.a = 1.0
        cube.id = 0
        cube.lifetime = rospy.Duration()
        rospy.loginfo('\nCube:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z))
        pub.publish(cube)

        # LINE_STRIP Marker
        line_strip = Marker()
        line_strip.header.frame_id = "/map"
        line_strip.header.stamp = rospy.Time.now()
        line_strip.type = line_strip.LINE_STRIP
        line_strip.action = line_strip.ADD
        p1 = Point(); p1.x = 0;   p1.y = 0; p1.z = 0;
        p2 = Point(); p2.x = 0.5; p2.y = 1; p2.z = 1.5;
        p3 = Point(); p3.x = 2;   p3.y = 2; p3.z = 2;
        line_strip.points = [p1, p2, p3]
        line_strip.ns = 'line_strip'
        line_strip.scale.x = 0.05
        line_strip.color.g = 1.0
        line_strip.color.a = 1.0
        line_strip.id = 1;
        line_strip.lifetime = rospy.Duration()
        # rospy.loginfo('\n\nLine Strip:\n\tx1: {}\ty1: {}\tz1: {}\n\tx2: {}\ty2: {}\tz2: {}\n\tx3: {}\ty3: {}\tz3: {}',line_strip.points[0].x,line_strip.points[0].y,line_strip.points[0].z,line_strip.points[1].x,line_strip.points[1].y,line_strip.points[1].z,line_strip.points[2].x,line_strip.points[2].y,line_strip.points[2].z,)
        rospy.loginfo('\nLine Strip:\n{}'.format(line_strip.points))
        pub.publish(line_strip)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


