#!/usr/bin/env python  
import roslib
import rospy
import tf

# class ChessBoard:

#     def __init__(self,robot):
  
#         # collision_object = moveit_msgs.msg.CollisionObject()
#         # collision_object.header.frame_id = robot.arm.get_planning_frame()
        
#         # just put a tf thing at a random position which corresponds to 0,0
#         #  on the chessboard?

#         cb = tf.TransformBroadcaster()
#         cb.sendTransform((msg.x, msg.y, 0),
#                      tf.transformations.quaternion_from_euler(0, 0, msg.theta),
#                      rospy.Time.now(),
#                      turtlename,
#                      "world")

#         print("MADE IT HERE!!!!!\n")


if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10000.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.75, 0.0, 0.0),
                         (1, 0.0, 0.0, 0.0),
                         rospy.Time.now(),
                         "chessboard",
                         "base")
        rate.sleep()