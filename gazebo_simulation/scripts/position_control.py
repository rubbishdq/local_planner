#!/usr/bin/python
#-*- encoding: utf8 -*-

import threading
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, Pose
from snapstack_msgs.msg import State
from nav_msgs.msg import Odometry
from  quadrotor_msgs.msg import PositionCommand
class position_control_node:

    def __init__(self):
        rospy.init_node('position_control_node', anonymous=True)
        self.lock_ = threading.Lock()
        self.pos_ = Pose()
        self.pos_.position.z = 1.5
        self.goalSub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalCallback)
        self.mavcmdPub_ = rospy.Publisher('/iris/rc/cmd_pose_flu', Pose, queue_size=100)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.lock_.acquire()
            self.mavcmdPub_.publish(self.pos_)
            self.lock_.release()
            rate.sleep()
        pass

    def goalCallback(self, msg):
        self.lock_.acquire()
        self.pos_ = msg.pose
        self.pos_.position.z = 1.5
        self.lock_.release()


if __name__ == '__main__':
    pcn = position_control_node()

