#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import threading
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path, Odometry
from snapstack_msgs.msg import State

def rostime2float(t):
    return (float(t.to_sec()) + float(t.to_nsec()) / 1e9)

class data_saving_node:

    def __init__(self):
        rospy.init_node('data_saving_node', anonymous=True)
        self.path_ = Path()
        self.volume_array_ = []
        self.path_lock_ = threading.Lock()
        self.volume_lock_ = threading.Lock()
        self.mavpose_ = PoseStamped()
        self.is_pose_init_ = False
        self.start_time_ = rospy.Time.now()
        self.last_pose_time_ = rospy.Time.now()
        self.mavpose_.header.stamp = self.last_pose_time_

        self.mavposeSub_ = rospy.Subscriber('/iris/pose', PoseStamped, self.mavposeCallback)
        self.knownvolumeSub_ = rospy.Subscriber('/iris/real_global_mapper/known_volume', PointStamped, self.knownvolumeCallback)
        self.savecommandSub_ = rospy.Subscriber('save_command', Bool, self.savecommandCallback)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.is_pose_init_ and not (self.mavpose_.header.stamp == self.last_pose_time_):
                self.updatePath()
            rate.sleep()
        pass

    def mavposeCallback(self, msg):
        self.mavpose_ = msg
        self.is_pose_init_ = True
        self.last_pose_time_ = msg.header

    def knownvolumeCallback(self, msg):
        self.volume_lock_.acquire()
        self.volume_array_.append([msg.point.x, msg.header.stamp])
        self.volume_lock_.release()

    def savecommandCallback(self, msg):
        if msg.data:
            self.saveData()
    
    def updatePath(self):
        self.path_.header.stamp = rospy.Time.now()
        self.path_.header.frame_id = 'map'
        
        self.path_lock_.acquire()
        self.path_.poses.append(self.mavpose_)
        self.path_lock_.release()

    def saveData(self):
        self.volume_lock_.acquire()
        self.path_lock_.acquire()
        f = open("local_explorer.txt", "w")
        f.write('%d\n' % (len(self.path_.poses)))
        for pose in self.path_.poses:
            f.write('%f %f %f %f\n' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, rostime2float(pose.header.stamp)))
        f.write('%d\n' % (len(self.volume_array_)))
        for volume in self.volume_array_:
            f.write('%f %f\n' % (volume[0], rostime2float(volume[1])))
        f.close()
        self.volume_lock_.release()
        self.path_lock_.release()
        

if __name__ == '__main__':
    dsn = data_saving_node()

