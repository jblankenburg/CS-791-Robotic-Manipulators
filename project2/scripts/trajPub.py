#!/usr/bin/env python
# license removed for brevity
import rospy
from traj_msgs.msg import TrajectoryCommand
from std_msgs.msg import Duration, Time

def trajPub():
    pub = rospy.Publisher('/trajectory_command', TrajectoryCommand, queue_size=10)
    rospy.init_node('trajPub', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

        # TrajectoryComamnd. = Duration
        trajCom = TrajectoryCommand()
        trajCom.time = rospy.Time.now()
        trajCom.t_k.data = rospy.Time(1)
        trajCom.t_k_prime.data = rospy.Time(1)
        # trajCom.names.append('right_s0')
        # trajCom.names.append('right_s1')
        # trajCom.names.append('right_e0')
        # trajCom.names.append('right_e1')
        # trajCom.names.append('right_w0')
        # trajCom.names.append('right_w1')
        # trajCom.names.append('right_w2')
        # trajCom.q_final = [0, 0, 0, 0, 0, 2.0, 0]
        # trajCom.qdot_final = [0, 0, 0, 0, 0, 1.0, 0]
        trajCom.names.append('right_e1')
        trajCom.q_final = [ 2.0 ]
        trajCom.qdot_final = [ 0.01 ]

        print(trajCom)
        pub.publish(trajCom)
        rate.sleep()

if __name__ == '__main__':
    try:
        trajPub()
    except rospy.ROSInterruptException:
        pass