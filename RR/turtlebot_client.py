from RobotRaconteur.Client import *     #import RR client library
import turtle
import sys
import math

#display setup
screen = turtle.Screen()
screen.bgcolor("lightblue")
#turtle setup
t1=turtle.Turtle()
t1.shape("turtle")

def update(turtle_obj,turtle_pose_wire):                    	#Update the display
    if turtle_obj.color=="None":		#update pen color on display
        t1.penup()
    else:
        t1.pencolor(turtle_obj.color)

    t1.setpos(turtle_pose_wire.InValue.x,turtle_pose_wire.InValue.y)
    yaw_ = math.degrees(turtle_pose_wire.InValue.angle)
    t1.seth(yaw_)

url='rr+tcp://localhost:2355/?service=turtlesim'
#take url from command line
if (len(sys.argv)>=2):
	url=sys.argv[1]

sub=RRN.SubscribeService(url)
turtle_obj = sub.GetDefaultClientWait(5)		#wait for 5 seconds timeout if no object returned
turtle_pose_wire=sub.SubscribeWire('turtle_pose_wire')		#subscribe to the wire name

while True:
    turtle_obj.drive(50,1)
    update(turtle_obj, turtle_pose_wire)		#update in display