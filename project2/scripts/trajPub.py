#!/usr/bin/env python
# license removed for brevity
import rospy
from traj_msgs.msg import TrajectoryCommand
from std_msgs.msg import Duration, Time

def trajPub():
    pub = rospy.Publisher('/trajectory_command', TrajectoryCommand, queue_size=1)
    rospy.init_node('trajPub', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

# pos
# 0.9826666908439572, 0.5940597752786445, 
# -0.21321062066663643, 0.4699321833994867, 
# -0.20273988682244237, -0.9907680805171095, 
# -0.016386982361796143
# vel
# -1.0525348814958922e-15, 3.2961146915857897e-15, 
# .175910198513561e-15, 2.037441542149736e-15, 
# 9.9410408622839e-16, 2.1447872384344207e-16, 
# -1.0177663528736229e-15

        # TrajectoryComamnd. = Duration
        trajCom = TrajectoryCommand()
        trajCom.time = rospy.Time.now()
        trajCom.t_k.data = rospy.Time(10)
        trajCom.t_k_prime.data = rospy.Time(10)
        trajCom.names.append('right_s0')
        trajCom.names.append('right_s1')
        trajCom.names.append('right_e0')
        trajCom.names.append('right_e1')
        trajCom.names.append('right_w0')
        trajCom.names.append('right_w1')
        trajCom.names.append('right_w2')
        trajCom.q_final = [1, .6, -0.2, 0.5, -0.2, -1, 0]
        trajCom.qdot_final = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        # trajCom.names.append('right_e1')
        # trajCom.q_final = [ 2.0 ]
        # trajCom.qdot_final = [ 0.01 ]

        print(trajCom)
        pub.publish(trajCom)
        rate.sleep()

if __name__ == '__main__':
    try:
        trajPub()
    except rospy.ROSInterruptException:
        pass