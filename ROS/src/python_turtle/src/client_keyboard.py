#!/usr/bin/env python

import rospy     #import ROS library
import math
import turtle
from geometry_msgs.msg import Twist
from python_turtle.msg import turtle_msg

import termios, fcntl, sys, os

#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

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

print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
try:
    #TODO: hold the script running with ROS/RR way
    while not rospy.is_shutdown():
        try:
            # update the turtle display
            update()

            unit_x = 0
            unit_z = 0
    
            #read input and print "command"
            c = sys.stdin.read()
            #TODO: ROS create message type variable, publish command
            #TODO: RR call drive function
            if "\x1b[A" in c:
                print("drive forward")          ####Drive forward
                unit_x += 1
            if "\x1b[B" in c:
                print("drive backward")         ####Drive backward
                unit_x -= 1               
            if "\x1b[C" in c:
                print("drive right")            ####Drive right
                unit_z -= 1
            if "\x1b[D" in c:
                print("drive left")             ####Drive left
                unit_z += 1
            if "q" in c:
                break

            msg=Twist()	#create message object
            msg.linear.x = 50 * unit_x
            msg.angular.z = 1 * unit_z
            pub.publish(msg)

        except IOError: pass
        except TypeError: pass
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)