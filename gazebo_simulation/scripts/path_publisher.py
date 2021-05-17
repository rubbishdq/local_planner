#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import threading
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from snapstack_msgs.msg import State

class path_publisher_node:

    def __init__(self):
        rospy.init_node('path_publisher_node', anonymous=True)
        self.path_ = Path()
        self.lock_ = threading.Lock()
        self.mavpose_ = PoseStamped()
        self.is_pose_init_ = False
        self.publish_start_ = False

        self.pathPub_ = rospy.Publisher('path', Path, queue_size=100)
        self.mavposeSub_ = rospy.Subscriber('pose', PoseStamped, self.mavposeCallback)
        self.begincommandSub_ = rospy.Subscriber('begin_command', Bool, self.begincommandCallback)

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            if self.is_pose_init_ and self.publish_start_:
                self.publishPath()
            rate.sleep()
        pass

    def mavposeCallback(self, msg):
        self.lock_.acquire()
        self.mavpose_ = msg
        self.is_pose_init_ = True
        self.lock_.release()

    def begincommandCallback(self, msg):
        self.publish_start_ = msg.data
    
    def publishPath(self):
        self.path_.header.stamp = rospy.Time.now()
        self.path_.header.frame_id = 'map'
        
        self.lock_.acquire()
        self.path_.poses.append(self.mavpose_)
        self.lock_.release()
        
        self.pathPub_.publish(self.path_)


if __name__ == '__main__':
    ppn = path_publisher_node()

