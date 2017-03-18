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
from sensor_msgs.msg import JointState


TRAJ_CMD = None
JOINT_STATE = None
Q_S = None
Q_S_DOT = None

def controlLoop():

	global JOINT_STATE
	global TRAJ_CMD
	global Q_S
	global Q_S_DOT

	# init node
	rospy.init_node('project2', anonymous=True)

	# set up publishers and subscribers
	jointSub = rospy.Subscriber("/robot/joint_states", JointState, joint_callback, queue_size=1)
	trajSub = rospy.Subscriber("/trajectory_command", TrajectoryCommand, traj_callback, queue_size=1)
	jointCmdPub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)
	# truePub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)

	# wait for messages
	while JOINT_STATE == None or TRAJ_CMD == None:
		pass

	# compute coeffs
	[a0, a1, a2, a3] = computeConstants(Q_S, Q_S_DOT, TRAJ_CMD.q_final, TRAJ_CMD.qdot_final, TRAJ_CMD.t_k) 
	rospy.loginfo('a0: %s\n a1: %s\n a2: %s\n a3: %s\n', a0, a1, a2, a3)

	# set rates and get timing
	rate_val = 1000
	rate = rospy.Rate(rate_val)
	start = rospy.get_time()
	dur = float(TRAJ_CMD.t_k.data.secs) + float(TRAJ_CMD.t_k.data.nsecs)/np.power(10,9)

	# while not shutdown
	while not rospy.is_shutdown():
		# # get joint state
		# state = JOINT_STATE

		# # get trajectory
		# cmd = TRAJ_CMD

		# # compute coeffs
	    # [a0, a1, a2, a3] = computeConstants(Q_S, Q_S_DOT, TRAJ_CMD.q_final, TRAJ_CMD.qdot_final, TRAJ_CMD.t_k) 

		# for time now until duration
		# time = rospy.Time.now()
		now = rospy.get_time()
		delta_t = now - start
		if delta_t < dur:

			# compute velocities
			q_f_dot = computeVelocities(delta_t, a0, a1, a2, a3)

			# publish velocites
			pubVels(q_f_dot, jointCmdPub)
			rospy.loginfo('q_f_dot: %s', q_f_dot)

		else:

			# publish zeros
			q_f_dot = np.zeros([len(a0), 1]) 
			pubVels(q_f_dot, jointCmdPub)
			rospy.loginfo('q_f_dot: %s', q_f_dot)

			# # verify the arm is in the right location? 
			# NOTE: IF TOO FAR FROM LOC BECUASE NOT ENOUGH TIME, THEN WILL BREAK GAZEBO
			# pos_f = TRAJ_CMD.q_final
			# pubTruePos(pos_f, truePub)

		# sleep
		rate.sleep()


def joint_callback(data):
    global JOINT_STATE
    global Q_S
    global Q_S_DOT
    q_s = []
    q_s_dot = []
    for name in TRAJ_CMD.names:
      for ind in range(len(data.name)):
        if data.name[ind] == name:
          q_s.append(data.position[ind]) 
          q_s_dot.append(data.velocity[ind])
	JOINT_STATE = data
	Q_S = q_s
	Q_S_DOT = q_s_dot    
	# rospy.loginfo("Joint callback!!!!")

def traj_callback(data):
    global TRAJ_CMD
    TRAJ_CMD = data 
    # rospy.loginfo("traj callback!!!!")


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

		a = np.array([[np.power(t,2), np.power(t,3)], [2*t, 3*np.power(t,2)]])
		b = np.array([q_f[i] - a_0[i] - a_1[i]*t, q_f_dot[i] - a_1[i]])
		x = np.linalg.solve(a,b)
		a_2[i] = float(x[0])
		a_3[i] = float(x[1])

	return [a_0, a_1, a_2, a_3]



def computeVelocities(t, a_0, a_1, a_2, a_3):

	q_f = np.zeros(len(a_0))
	q_f_dot = np.zeros(len(a_0))
	for i in range(len(a_0)):
		q_f[i] = a_0[i][0] + a_1[i][0]*t + a_2[i][0]*t*t + a_3[i][0]*t*t*t
		q_f_dot[i] = a_1[i][0] + 2*a_2[i][0]*t + 3*a_3[i][0]*t*t

	return q_f_dot    


def pubVels(q_f_dot, pub):

	# set up joint command msg
	jointCmd = JointCommand()
	jointCmd.mode = JointCommand.VELOCITY_MODE
	for i in range(len(q_f_dot)):
		jointCmd.names.append(TRAJ_CMD.names[i])
		jointCmd.command.append(q_f_dot[i])

	# publish message
	pub.publish(jointCmd)


def pubTruePos(pos_f, pub):
	jointCmd = JointCommand()
	jointCmd.mode = JointCommand.POSITION_MODE
	for i in range(len(pos_f)):
		jointCmd.names.append(TRAJ_CMD.names[i])
		jointCmd.command.append(pos_f[i])

	# publish message
	pub.publish(jointCmd)



# part 2
# choose your own adventure part 2:
#   publish the instantaneous Jacobian matrix for the baxter at 1000Hz



if __name__ == '__main__':
  try:
    # vel_test()
    controlLoop()
  except rospy.ROSInterruptException:
    pass


