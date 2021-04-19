#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rospy
from std_msgs.msg import String, Bool
from faster_msgs.msg import Mode
from snapstack_msgs.msg import QuadGoal, State
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from behavior_selector.srv import MissionModeChange
import math

def quat2yaw(q, deg = False):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    if deg:
        return yaw / math.pi * 180
    else:
        return yaw


def yaw2quat(yaw, deg = False):
    if deg: 
        yaw = yaw / 180. * math.pi
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw/2)
    q.w = math.cos(yaw/2)
    return q


def angle_norm(ang):
    ang_norm = (ang+math.pi) % (2*math.pi) - math.pi
    return ang_norm


class Faster_Commands:

    def __init__(self):
        self.mode=Mode()
        self.pose = Pose()
        self.mode.mode=self.mode.ON_GROUND
        self.pubCmd = rospy.Publisher("rc/cmd", String, queue_size=1)
        self.pubGoal = rospy.Publisher("rc/cmd_pose_flu", Pose, queue_size=1)
        self.pubMode = rospy.Publisher("faster/mode",Mode,queue_size=1,latch=True) #TODO Namespace
        self.pubClickedPoint = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1,latch=True)
        self.pubRecordCommand = rospy.Publisher("record_command",Bool,queue_size=1,latch=True)

        self.is_ground_robot = False
        self.circle = True
        self.start_recording = True

        self.alt_taken_off = 1.5 #Altitude when hovering after taking off
        self.alt_ground = 0 #Altitude of the ground
        self.initialized=False

    #In rospy, the callbacks are all of them in separate threads
    def stateCB(self, data):
        self.pose = data.pose

        if(self.initialized==False):
            #self.pubFirstGoal()
            self.initialized=True

    #Called when buttom pressed in the interface
    def srvCB(self,req):
        if(self.initialized==False):
            print "Not initialized yet"
            return

        if req.mode == req.START and self.mode.mode==self.mode.ON_GROUND:
            print "Taking off"
            self.takeOff()
            print "Take off done"

        if req.mode == req.KILL:
            print "Killing"
            self.kill()

        if req.mode == req.END and self.mode.mode==self.mode.GO:
            print "Landing"
            self.land()
            print "Landing done"


    def sendMode(self):
        self.mode.header.stamp = rospy.get_rostime()
        self.pubMode.publish(self.mode)


    def takeOff(self):
        cmd_str = String()
        cmd_str.data = "OFFBOARD"
        self.pubCmd.publish(cmd_str)
        rospy.sleep(1)
        cmd_str.data = "ARM"
        self.pubCmd.publish(cmd_str)

        goal=self.pose
        if(self.is_ground_robot==True):
            self.alt_taken_off=self.pose.position.z
        #Note that self.pose.position is being updated in the parallel callback
        while(  abs(self.pose.position.z-self.alt_taken_off)>0.1  ): 
            goal.position.z = self.alt_taken_off
            self.sendGoal(goal)
        rospy.sleep(5) 
        # circle counterclockwise
        goal=self.pose
        if self.circle:
            list_length = 4
            ang_list = []
            for i in range(list_length):
                if i == list_length-1:
                    ang_list.append(0.0)
                else:
                    ang_list.append(2*math.pi*(i+1)/list_length)
            for ang in ang_list:
                while True:
                    yaw = quat2yaw(self.pose.orientation)
                    if abs(angle_norm(yaw - ang)) / math.pi * 180. < 10:
                        break
                    goal.orientation = yaw2quat(yaw + math.pi / 30.)
                    self.sendGoal(goal)
                    rospy.sleep(0.02)
            print("already circled")
            print(ang_list)
        rospy.sleep(1.5)
        if self.start_recording:
            record_cmd_msg = Bool()
            record_cmd_msg.data = True
            self.pubRecordCommand.publish(record_cmd_msg)
        self.mode.mode=self.mode.GO
        self.sendMode()


    def land(self):
        cmd_str = String()
        cmd_str.data = "AUTO.LAND"
        self.pubCmd.publish(cmd_str)
        '''
        goal = self.pose
        #Note that self.pose.position is being updated in the parallel callback
        while(abs(self.pose.position.z-self.alt_ground)>0.0):
            goal.position.z = max(goal.position.z-0.0035, self.alt_ground)
            self.sendGoal(goal)
        '''
        #Kill motors once we are on the ground
        self.kill()

    def kill(self):
        goal=Pose()
        self.sendGoal(goal)
        self.mode.mode=self.mode.ON_GROUND
        self.sendMode()

    def sendGoal(self, goal):
        self.pubGoal.publish(goal)

    def pubFirstGoal(self):
        msg=PoseStamped()
        msg.pose.position.x=self.pose.position.x
        msg.pose.position.y=self.pose.position.y
        msg.pose.position.z=self.alt_taken_off
        msg.pose.orientation.x=0.0
        msg.pose.orientation.y=0.0
        msg.pose.orientation.z=0.0
        msg.pose.orientation.w=1.0
        msg.header.frame_id="world"
        msg.header.stamp = rospy.get_rostime()
        self.pubClickedPoint.publish(msg)

                  
def startNode():
    c = Faster_Commands()
    s = rospy.Service("/change_mode",MissionModeChange,c.srvCB)
    rospy.Subscriber("pose", PoseStamped, c.stateCB)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('faster_commands')  
    startNode()
    print "Faster Commands started" 