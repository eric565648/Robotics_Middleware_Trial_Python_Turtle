from RobotRaconteur.Client import *     #import RR client library
import turtle
import termios, fcntl, sys, os
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

#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
try:
    #TODO: hold the script running with ROS/RR way
    while True:
        try:
            #read input and print "command"
            c = sys.stdin.read()

            unit_x = 0
            unit_z = 0

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
        
            turtle_obj.drive(50*unit_x,1*unit_z)
            update(turtle_obj, turtle_pose_wire)		#update in display

        except IOError: pass
        except TypeError: pass
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)