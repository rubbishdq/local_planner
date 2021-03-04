#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped

class UAV_tf_node:

    def __init__(self):
        rospy.init_node('uav_tf_node', anonymous=True)
        if len(sys.argv) < 3:
            rospy.logerr('Error: len(sys.argv) must not be less than 3! Now %d' % (len(sys.argv)))
        else:
            start_frame = sys.argv[1]
            end_frame = sys.argv[2]
            self.mavpos_ = (0., 0., 0.)
            self.mavrot_ = (0., 0., 0., 1.)
            self.is_pose_init_ = False
            self.mavposeSub_ = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavposeCallback)
            self.tfBroadcaster_ = tf.TransformBroadcaster()
            rate = rospy.Rate(20.0)
            while not rospy.is_shutdown():
                if self.is_pose_init_:
                    self.tfBroadcaster_.sendTransform(self.mavpos_, self.mavrot_, rospy.Time.now(), end_frame, start_frame)
                rate.sleep()
            pass

    def mavposeCallback(self, msg):
        self.mavpos_ = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.mavrot_ = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.is_pose_init_ = True


if __name__ == '__main__':
    utn = UAV_tf_node()

