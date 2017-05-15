#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from traj_msgs.msg import PieceMovement

def talker():
    pub = rospy.Publisher('pieceMovement', PieceMovement, queue_size=10)
    rospy.init_node('pieceMovementPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        postionCmd = PieceMovement()
        postionCmd.move_from = 'c3'
        postionCmd.move_to = 'c4'
        rospy.loginfo(postionCmd)
        pub.publish(postionCmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass