#!/usr/bin/env python

import time
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from python_turtle.msg import turtle_msg
from python_turtle.srv import setcolor, setpose

def quat_from_rpy(roll, pitch, yaw):

    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

def rpy_from_quat(x, y, z, w):

    srcp = 2*(w*x + y*z)
    crcp = 1-2*(x*x + y*y)
    roll = math.atan2(srcp, crcp)

    sp = 2*(w*y - z*x)
    if math.fabs(sp) >= 1:
        pitch = (sp/math.fabs(sp))*math.pi/2
    else:
        pitch = math.asin(sp)
    
    sycp = 2*(w*z + x*y)
    cycp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(sycp, cycp)

    return roll, pitch, yaw

class CreateTurtle:
    def __init__(self):               			
        #initialization upon creation
        #message type initialziation
        self.turtle_pose=Pose()
        self.turtle=turtle_msg()

        self.lin_x = 0
        self.ang_z = 0
        self.dirty = False

        #subscriber initialization
        self.vel_sub=rospy.Subscriber('bot_cmd',Twist,self.drive_callback)
        
        #publisher initialization
        self.turtle_pub=rospy.Publisher('bot_pose',turtle_msg,queue_size=1)
        
        #service initialization
        self.pose_server=rospy.Service('bot_set_pose', setpose, self.set_pose)
        self.color_server=rospy.Service('bot_set_color', setcolor, self.set_color)

        # set rospy timer for drive update
        self.interval = 0.04
        rospy.Timer(rospy.Duration(self.interval), self.drive_update)

    def drive_callback(self, data):            
        #Drive function, update new position, this is the one referred in definition
        
        self.lin_x = data.linear.x
        self.ang_z = data.angular.z
        self.dirty = True
        
        # so strange here!!!!!!!!!!!!!!!!!
        # self.turtle.turtle_pose.position.x = data.position.x
        # self.turtle.turtle_pose.position.y = data.position.y
        # self.turtle.turtle_pose.orientation.z = <decode message type and update pose>

    def drive_update(self, timer):

        if not self.dirty:
            return

        qx = self.turtle.turtle_pose.orientation.x
        qy = self.turtle.turtle_pose.orientation.y
        qz = self.turtle.turtle_pose.orientation.z
        qw = self.turtle.turtle_pose.orientation.w
        x = self.turtle.turtle_pose.position.x
        y = self.turtle.turtle_pose.position.y

        roll, pitch, yaw = rpy_from_quat(qx, qy, qz, qw)

        # update orientation
        yaw = yaw + self.ang_z * self.interval
        
        # keep yaw between -pi and pi
        # Think: Why we use while loop but not if-else here?
        while yaw < -1*math.pi:
            yaw += 2*math.pi
        while yaw >= math.pi:
            yaw -= 2*math.pi

        print('yaw:',yaw)

        # to quaternion
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)

        # update position
        x = x + math.cos(yaw) * self.lin_x * self.interval
        y = y + math.sin(yaw) * self.lin_x * self.interval

        # clear cmd
        self.lin_x = 0
        self.ang_z = 0

        # update msg
        self.turtle.turtle_pose.orientation.x = qx
        self.turtle.turtle_pose.orientation.y = qy
        self.turtle.turtle_pose.orientation.z = qz
        self.turtle.turtle_pose.orientation.w = qw
        self.turtle.turtle_pose.position.x = x
        self.turtle.turtle_pose.position.y = y

        self.dirty = False

    def set_pose(self,req):		
        #update pose based on given pose
        self.turtle.turtle_pose=req.turtle_pose.pose
        return 1

    def set_color(self,req):	
        #set color based on given color
        self.turtle.color=req.color
        return 1

rospy.init_node('turtlebot_service', anonymous=True)
turtle_obj=CreateTurtle()

while not rospy.is_shutdown():
	turtle_obj.turtle_pub.publish(turtle_obj.turtle)
	time.sleep(0.01)