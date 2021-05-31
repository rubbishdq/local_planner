#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import threading
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class data_visualizer_node:

    def __init__(self):
        rospy.init_node('data_visualizer_node', anonymous=True)
        self.begin_time_ = rospy.Time.now()
        self.lp_path_ = self.initPath('/home/dqs/catkin_exp/data/lp_sp.txt')
        self.fuel_path_ = self.initPath('/home/dqs/catkin_exp/data/fuel_sp.txt', True)
        self.classic_path_ = self.initPath('/home/dqs/catkin_exp/data/classic_sp.txt')
        self.nbvp_path_ = self.initPath('/home/dqs/catkin_exp/data/nbvp_sp.txt')

        self.lppathPub_ = rospy.Publisher('lp_path', Path, queue_size=100)
        self.fuelpathPub_ = rospy.Publisher('fuel_path', Path, queue_size=100)
        self.classicpathPub_ = rospy.Publisher('classic_path', Path, queue_size=100)
        self.nbvppathPub_ = rospy.Publisher('nbvp_path', Path, queue_size=100)

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            self.publishPath(self.lp_path_, self.lppathPub_)
            self.publishPath(self.fuel_path_, self.fuelpathPub_)
            self.publishPath(self.classic_path_, self.classicpathPub_)
            self.publishPath(self.nbvp_path_, self.nbvppathPub_)
            rate.sleep()
        pass

    def initPath(self, file, use_direct_time = False):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'world'
        fr = open(file)
        traj_num = -1
        traj_mat = None        #prepare matrix to return
        index = 0
        for line in fr.readlines():
            if index == traj_num:
                break
            if index == 0:
                traj_num = int(line)
                print(traj_num)
            else:
                ps = PoseStamped()
                line_split = line.split(' ')
                if use_direct_time:
                    ps.header.stamp = self.begin_time_
                else:
                    ps.header.stamp = self.begin_time_ + rospy.Duration(float(line_split[3]))
                ps.header.frame_id = 'world'
                ps.pose.position.x = float(line_split[0])
                ps.pose.position.y = float(line_split[1])
                ps.pose.position.z = float(line_split[2])
                ps.pose.orientation.x = 0
                ps.pose.orientation.y = 0
                ps.pose.orientation.z = 0
                ps.pose.orientation.w = 0
                path_msg.poses.append(ps)
            index += 1
        return path_msg
    
    def publishPath(self, path_msg, path_publisher):
        path_msg.header.stamp = rospy.Time.now()
        path_publisher.publish(path_msg)


if __name__ == '__main__':
    dvn = data_visualizer_node()

