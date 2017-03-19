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
import message_filters

import rospy
from math import sin, cos, pi
from baxter_core_msgs.msg import JointCommand
from geometry_msgs.msg import Point
from std_msgs.msg import String
from traj_msgs.msg import TrajectoryCommand
from traj_msgs.msg import TrajectoryMultiCommand
from sensor_msgs.msg import JointState


MULTI_TRAJ_CMD = None
TRAJ_CMD = None
JOINT_STATE = None
Q_S = None
Q_S_DOT = None

def controlLoop():

	global JOINT_STATE
	global MULTI_TRAJ_CMD
	global TRAJ_CMD
	global Q_S
	global Q_S_DOT

	# init node
	rospy.init_node('project2_part2', anonymous=True)

	# set up publishers and subscribers
	jointSub = rospy.Subscriber("/robot/joint_states", JointState, joint_callback, queue_size=1)
	trajSub = rospy.Subscriber("/multi_trajectory_command", TrajectoryMultiCommand, traj_callback, queue_size=1)
	jointCmdPub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)
	truePub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)

	# wait for messages
	while JOINT_STATE is None or MULTI_TRAJ_CMD is None:
		pass

	# set robot to first location in trajectory command:
	# verify the arm is in the right location? 
	pos_f = MULTI_TRAJ_CMD[0].q_final
	pubTruePos(pos_f, truePub)

	# loop through the list of trajectory commands and publish each section
	for i in range(len(MULTI_TRAJ_CMD)-1):

		print('performing trajectory i: %s ', i)
		TRAJ_CMD = MULTI_TRAJ_CMD[i]

		# set rates and get timing
		rate_val = 1000
		rate = rospy.Rate(rate_val)
		start = rospy.get_time()
		lin_dur = float(TRAJ_CMD.t_k.data.secs) + float(TRAJ_CMD.t_k.data.nsecs)/np.power(10,9)
		quad_dur = float(TRAJ_CMD.t_k_prime.data.secs) + float(TRAJ_CMD.t_k.data.nsecs)/np.power(10,9)

		# while not shutdown
		next_traj = 0
		while not rospy.is_shutdown() and not next_traj:

			# get timing
			now = rospy.get_time()
			delta_t = now - start

			# compute linear trajectory:
			if delta_t < lin_dur:
				q_k1 = MULTI_TRAJ_CMD[i].q_final
				q_k2 = MULTI_TRAJ_CMD[i+1].q_final
				t1 = float(MULTI_TRAJ_CMD[i].t_k.data.secs) + float(MULTI_TRAJ_CMD[i].t_k.data.nsecs)/np.power(10,9)

				q_f_dot = computeLinear(q_k1, q_k2, t1)
				pubVels(q_f_dot, jointCmdPub)
				rospy.loginfo('q_f_dot_linear: %s', q_f_dot)


			# compute the quadtratic transition trajectory:
			elif delta_t < lin_dur + quad_dur:
				q_f_dot_q = computeQuad(i, delta_t)
				pubVels(q_f_dot_q, jointCmdPub)
				rospy.loginfo('q_f_dot_transition: %s', q_f_dot)

			# if past quad transition time, exit loop to get next trajectory
			else:
				next_traj = 1

			# sleep
			rate.sleep()


def joint_callback(data):
	global JOINT_STATE
	global Q_S
	global Q_S_DOT
	q_s = []
	q_s_dot = []
	for i in range(len(MULTI_TRAJ_CMD)):
	    for name in MULTI_TRAJ_CMD[i].names:
	      for ind in range(len(data.name)):
	        if data.name[ind] == name:
	          q_s.append(data.position[ind]) 
	          q_s_dot.append(data.velocity[ind])
	JOINT_STATE = data
	Q_S = q_s
	Q_S_DOT = q_s_dot    
	# rospy.loginfo("Joint callback!!!!")

def traj_callback(data):
    global MULTI_TRAJ_CMD
    MULTI_TRAJ_CMD = data.points 
    # rospy.loginfo("traj callback!!!!")


def pubTruePos(pos_f, pub):
	jointCmd = JointCommand()
	jointCmd.mode = JointCommand.POSITION_MODE
	for i in range(len(pos_f)):
		jointCmd.names.append(MULTI_TRAJ_CMD[0].names[i])
		jointCmd.command.append(pos_f[i])

	# publish message
	pub.publish(jointCmd)


def computeLinear(q_k1, q_k2, t1):

	q_f_dot = np.zeros(len(q_k1))
	# compute linear vel for each joint
	for i in range(len(q_k1)):
		q_f_dot[i] = (q_k2[i] - q_k1[i])/t1
	return q_f_dot  


def computeQuad(i, t_cur):
	global MULTI_TRAJ_CMD


	# get given positions
	q_k1 = MULTI_TRAJ_CMD[i].q_final
	q_k2 = MULTI_TRAJ_CMD[i+1].q_final
	# if in range
	# rospy.loginfo('i:%s  i+1: %s i+2: %s len(MULTI_TRAJ_CMD)-2: %s', i, i+1, i+2, len(MULTI_TRAJ_CMD))
	if i <= len(MULTI_TRAJ_CMD)-3:
		q_k3 = MULTI_TRAJ_CMD[i+2].q_final
	#  no second traj to compare to, so set q_k3 to 0's
	else:
		q_k3 = np.zeros([len(q_k1),1])

	# get given timing 
	delta_t1 = float(MULTI_TRAJ_CMD[i].t_k.data.secs) + float(MULTI_TRAJ_CMD[i].t_k.data.nsecs)/np.power(10,9)
	delta_t2 = float(MULTI_TRAJ_CMD[i+1].t_k.data.secs) + float(MULTI_TRAJ_CMD[i+1].t_k.data.nsecs)/np.power(10,9)
	delta_t_prime = float(MULTI_TRAJ_CMD[i].t_k_prime.data.secs) + float(MULTI_TRAJ_CMD[i].t_k_prime.data.nsecs)/np.power(10,9)

	# for each joint:
	accel = np.zeros(len(q_k1))
	q_f_dot = np.zeros(len(q_k1))
	q_k12_dot = computeLinear(q_k1, q_k2, delta_t1)
	q_k23_dot = computeLinear(q_k2, q_k3, delta_t2)
	for i in range(len(q_k1)):
		# calculate acceleration
		accel[i] = (q_k23_dot[i] - q_k12_dot[i]) / delta_t_prime 

		# integrate acceleration to get velocity (i.e. a*t)
		q_f_dot[i] = accel[i]*t_cur
	return q_f_dot



def pubVels(q_f_dot, pub):
	global TRAJ_CMD

	# set up joint command msg
	jointCmd = JointCommand()
	jointCmd.mode = JointCommand.VELOCITY_MODE
	for i in range(len(q_f_dot)):
		jointCmd.names.append(MULTI_TRAJ_CMD[0].names[i])
		jointCmd.command.append(q_f_dot[i])

	# publish message
	print(jointCmd)
	pub.publish(jointCmd)



if __name__ == '__main__':
  try:
    # vel_test()
    controlLoop()
  except rospy.ROSInterruptException:
    pass


