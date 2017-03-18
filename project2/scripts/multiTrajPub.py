#!/usr/bin/env python
# license removed for brevity
import rospy
from traj_msgs.msg import TrajectoryCommand
from traj_msgs.msg import TrajectoryMultiCommand
from std_msgs.msg import Duration, Time

def multiTrajPub():
    pub = rospy.Publisher('/multi_trajectory_command', TrajectoryMultiCommand, queue_size=1)
    rospy.init_node('multiTrajPub', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():

        # TrajectoryComamnd. = Duration
        trajCom1 = TrajectoryCommand()
        trajCom1.time = rospy.Time.now()
        trajCom1.t_k.data = rospy.Time(10)
        trajCom1.t_k_prime.data = rospy.Time(2)
        trajCom1.names.append('right_s0')
        trajCom1.names.append('right_s1')
        trajCom1.names.append('right_e0')
        trajCom1.names.append('right_e1')
        trajCom1.names.append('right_w0')
        trajCom1.names.append('right_w1')
        trajCom1.names.append('right_w2')
        trajCom1.q_final = [0, 0, 0, 0, 0, 0, 0]
        # trajCom.qdot_final = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]


        trajCom2 = TrajectoryCommand()
        trajCom2.time = rospy.Time.now()
        trajCom2.t_k.data = rospy.Time(10)
        trajCom2.t_k_prime.data = rospy.Time(2)
        trajCom2.names.append('right_s0')
        trajCom2.names.append('right_s1')
        trajCom2.names.append('right_e0')
        trajCom2.names.append('right_e1')
        trajCom2.names.append('right_w0')
        trajCom2.names.append('right_w1')
        trajCom2.names.append('right_w2')
        trajCom2.q_final = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        # trajCom2.qdot_final = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

        # trajCom.q_final = [1, .6, -0.2, 0.5, -0.2, -1, 0]
        # trajCom.qdot_final = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

        trajCom3 = TrajectoryCommand()
        trajCom3.names.append('right_e1')
        trajCom3.time = rospy.Time.now()
        trajCom3.t_k.data = rospy.Time(10)
        trajCom3.t_k_prime.data = rospy.Time(2)
        trajCom3.q_final = [ 2.0 ]
        # trajCom3.qdot_final = [ 0.01 ]

        multiTrajCom = TrajectoryMultiCommand()
        multiTrajCom.points = [trajCom1, trajCom2, trajCom3] 

        print(multiTrajCom)
        pub.publish(multiTrajCom)
        rate.sleep()

if __name__ == '__main__':
    try:
        multiTrajPub()
    except rospy.ROSInterruptException:
        pass