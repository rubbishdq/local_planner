#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from gazebo_msgs.msg import ModelStates
from snapstack_msgs.msg import State
class UAV_tf_node:

    def __init__(self, start_frame, end_frame, use_gazebo_ground_truth = False):
        rospy.init_node('uav_tf_node', anonymous=True)
        self.start_frame_ = start_frame
        self.end_frame_ = end_frame
        self.mavpos_ = (0., 0., 0.)
        self.mavrot_ = (0., 0., 0., 1.)
        self.mav_linear_vel_ = (0., 0., 0.)
        self.mav_angular_vel_ = (0., 0., 0.)
        self.is_pose_init_ = False
        self.is_vel_init_ = False
        if use_gazebo_ground_truth:
            self.mavposeSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazeboposeCallback)
        else:
            self.mavposeSub_ = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.mavposeCallback)
        # TODO: use a filter to obtain robot's velocity using ground truth pose
        self.mavvelSub_ = rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.mavvelCallback)  # or velocity_local
        self.mavposePub_ = rospy.Publisher('pose', PoseStamped, queue_size=100)
        self.mavstatePub_ = rospy.Publisher('state', State, queue_size=100)
        self.tfBroadcaster_ = tf.TransformBroadcaster()
        rate = rospy.Rate(8.0)
        while not rospy.is_shutdown():
            if self.is_pose_init_:
                self.tfBroadcaster_.sendTransform(self.mavpos_, self.mavrot_, rospy.Time.now(), self.end_frame_, self.start_frame_)
                self.publishMavpose()
                if self.is_vel_init_:
                    self.publishMavstate()
            rate.sleep()
        pass

    def mavposeCallback(self, msg):
        self.mavpos_ = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.mavrot_ = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.is_pose_init_ = True

    def mavvelCallback(self, msg):
        self.mav_linear_vel_ = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.mav_angular_vel_ = (msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)
        self.is_vel_init_ = True

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
    
    def publishMavstate(self):
        msg = State()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.state_stamp = rospy.Time.now()
        msg.pos.x = self.mavpos_[0]
        msg.pos.y = self.mavpos_[1]
        msg.pos.z = self.mavpos_[2]
        msg.quat.x = self.mavrot_[0]
        msg.quat.y = self.mavrot_[1]
        msg.quat.z = self.mavrot_[2]
        msg.quat.w = self.mavrot_[3]
        msg.vel.x = self.mav_linear_vel_[0]
        msg.vel.y = self.mav_linear_vel_[1]
        msg.vel.z = self.mav_linear_vel_[2]
        msg.w.x = self.mav_angular_vel_[0]
        msg.w.y = self.mav_angular_vel_[1]
        msg.w.z = self.mav_angular_vel_[2]
        self.mavstatePub_.publish(msg)


if __name__ == '__main__':
    use_gazebo_ground_truth = True
    if len(sys.argv) < 2:
        rospy.logerr('Error: len(sys.argv) must not be less than 2! Now %d' % (len(sys.argv)))
        exit()
    utn = UAV_tf_node(sys.argv[1], sys.argv[2], use_gazebo_ground_truth = use_gazebo_ground_truth)

