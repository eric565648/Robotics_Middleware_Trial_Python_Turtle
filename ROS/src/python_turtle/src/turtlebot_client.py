#!/usr/bin/env python

import rospy     #import ROS library
import math
import turtle
from geometry_msgs.msg import Twist
from python_turtle.msg import turtle_msg

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

#display setup
screen = turtle.Screen()
screen.bgcolor("lightblue")
#turtle setup
t1=turtle.Turtle()
t1.shape("turtle")

turtle_obj=turtle_msg()
def callback(data):
    global turtle_obj
    turtle_obj=data
    
def update():
    global turtle_obj
    if turtle_obj.color=="None":
        t1.penup()
    else:
        t1.pencolor(turtle_obj.color)

    t1.setpos(turtle_obj.turtle_pose.position.x,turtle_obj.turtle_pose.position.y)

    # A mistake here too. Should convert quaternion to rpy although it's in 2D.
    qx = turtle_obj.turtle_pose.orientation.x
    qy = turtle_obj.turtle_pose.orientation.y
    qz = turtle_obj.turtle_pose.orientation.z
    qw = turtle_obj.turtle_pose.orientation.w
    roll, pitch, yaw = rpy_from_quat(qx, qy, qz, qw)
    yaw_ = math.degrees(yaw)
    
    t1.seth(yaw_)

rospy.init_node('turtlebot_client', anonymous=False)
pub=rospy.Publisher('bot_cmd',Twist,queue_size=1)
sub=rospy.Subscriber('bot_pose',turtle_msg,callback)

while not rospy.is_shutdown():
    update()
    msg=Twist()	#create message object
    msg.linear.x=50
    msg.angular.z=1
    pub.publish(msg)