#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

class CoreControllerNode:
    def __init__(self):
        rospy.init_node('core_controller_node', anonymous=True)

        self.is_takeoff_ = False
        self.current_state_ = State()

        self.uav_target_pose_local_ = PoseStamped()
        self.uav_target_pose_local_.header.seq = 1
        self.uav_target_pose_local_.header.frame_id = 'map'
        self.uav_target_pose_local_.pose.orientation.w = 1

        self.last_request_ = rospy.Time.now()

        self.uav_target_pose_local_pub_ = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.mavstateSub_ = rospy.Subscriber('/mavros/state', State, self.mavstateCallback)

        self.arming_client_ = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.land_client_ = rospy.ServiceProxy('/mavros/cmd/land', CommandBool)
        self.set_mode_client_ = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.statusloop_timer_ = rospy.Timer(rospy.Duration(1), self.statusloopCallback)
        self.publishloop_timer_ = rospy.Timer(rospy.Duration(0.1), self.publishloopCallback)

        rate = rospy.Rate(20)
        for i in range(10):
            self.uav_target_pose_local_pub_.publish(self.uav_target_pose_local_)
            rate.sleep()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def mavstateCallback(self, msg):
        self.current_state_ = msg

    def statusloopCallback(self, event):
        if self.is_takeoff_ == False:
            if self.current_state_.mode != 'OFFBOARD' and (rospy.Time.now() - self.last_request_ > rospy.Duration(4.0)):
                response = self.set_mode_client_(0, 'OFFBOARD')  #请求解锁
                if response.mode_sent:
                    rospy.loginfo('Offboard enabled')
                self.last_request_ = rospy.Time.now()
            else:
                if (not self.current_state_.armed) and rospy.Time.now() - self.last_request_ > rospy.Duration(4.0):
                    response = self.arming_client_(True)  #请求起飞
                    if response.success:
                        rospy.loginfo('Vehicle armed')
                        self.is_takeoff_ = True
                    self.last_request_ = rospy.Time.now()
        else:
            pass

    def publishloopCallback(self, event):
        self.uav_target_pose_local_pub_.publish(self.uav_target_pose_local_)

if __name__ == '__main__':
    cc = CoreControllerNode()