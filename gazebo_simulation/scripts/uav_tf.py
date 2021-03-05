#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
class UAV_tf_node:

    def __init__(self, use_gazebo_ground_truth):
        rospy.init_node('uav_tf_node', anonymous=True)
        if len(sys.argv) < 3:
            rospy.logerr('Error: len(sys.argv) must not be less than 3! Now %d' % (len(sys.argv)))
        else:
            self.start_frame_ = sys.argv[1]
            self.end_frame_ = sys.argv[2]
            self.mavpos_ = (0., 0., 0.)
            self.mavrot_ = (0., 0., 0., 1.)
            self.is_pose_init_ = False
            if use_gazebo_ground_truth:
                self.mavposeSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazeboposeCallback)
            else:
                self.mavposeSub_ = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavposeCallback)
            self.mavposePub_ = rospy.Publisher('/'+self.start_frame_+'/pose', PoseStamped, queue_size=100)
            self.tfBroadcaster_ = tf.TransformBroadcaster()
            rate = rospy.Rate(15.0)
            while not rospy.is_shutdown():
                if self.is_pose_init_:
                    self.tfBroadcaster_.sendTransform(self.mavpos_, self.mavrot_, rospy.Time.now(), self.end_frame_, self.start_frame_)
                    self.publishMavpose()
                rate.sleep()
            pass

    def mavposeCallback(self, msg):
        self.mavpos_ = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.mavrot_ = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.is_pose_init_ = True

    def gazeboposeCallback(self, msg):
        if self.start_frame_ in msg.name:
            i = msg.name.index(self.start_frame_)
            pose = msg.pose[i]
            self.mavpos_ = (pose.position.x, pose.position.y, pose.position.z)
            self.mavrot_ = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            self.is_pose_init_ = True
    
    def publishMavpose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.mavpos_[0]
        msg.pose.position.y = self.mavpos_[1]
        msg.pose.position.z = self.mavpos_[2]
        msg.pose.orientation.x = self.mavrot_[0]
        msg.pose.orientation.y = self.mavrot_[1]
        msg.pose.orientation.z = self.mavrot_[2]
        msg.pose.orientation.w = self.mavrot_[3]
        self.mavposePub_.publish(msg)


if __name__ == '__main__':
    utn = UAV_tf_node(True)

