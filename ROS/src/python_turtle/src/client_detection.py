#!/usr/bin/env python

import rospy     #import ROS library
import math
import turtle
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from python_turtle.msg import turtle_msg
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

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

bridge=CvBridge()
def img_callback(img):

    global bridge

    #triggered when data received
    # from ros to cv2 image
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")	#convert ros image message to opencv object
    except CvBridgeError as e:
        print(e)
    
    if (cv_image is None):
        print("Bridging image false or no image data received")
        return

    image_size=len(cv_image)*len(cv_image[0]) #get image size
    image_dimension=np.array([len(cv_image),len(cv_image[0])])    #get image dimension

    # use hsv
    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Define the boundaries of "white", "yellow" and "red"
    # hsv_white1 = np.array([0,0,150])
    # hsv_white2 = np.array([180,100,255])
    hsv_blue1 = np.array([100, 150,0])
    hsv_blue2 = np.array([140, 255, 255])
    hsv_yellow1 = np.array([25,50,50])
    hsv_yellow2 = np.array([45,255,255])
    # The color "red" needs two set of boundaries cause it pass 255 to 0
    hsv_red1 = np.array([0,100,100])
    hsv_red2 = np.array([15,255,255])
    hsv_red3 = np.array([165,100,100])
    hsv_red4 = np.array([180,255,255])

    # detect red
    # filtered_red=cv2.inRange(cv_image,np.array([5,5,200]),np.array([200,200,255])) #filter the image with upper bound and lower bound in bgr format
    filtered_red1 = cv2.inRange(hsv_img, hsv_red1, hsv_red2)
    filtered_red2 = cv2.inRange(hsv_img, hsv_red3, hsv_red4)
    filtered_red = cv2.bitwise_or(filtered_red1, filtered_red2)
    # detect yellow
    filtered_yellow = cv2.inRange(hsv_img, hsv_yellow1, hsv_yellow2)
    # detect blue
    filtered_blue = cv2.inRange(hsv_img, hsv_blue1, hsv_blue2)
    
    
    #show filtered image
    cv2.namedWindow("Image Red")
    cv2.imshow("Image Red",filtered_red)
    cv2.namedWindow("Image Yellow")
    cv2.imshow("Image Yellow",filtered_yellow)
    cv2.namedWindow("Image Blue")
    cv2.imshow("Image Blue",filtered_blue)
    if cv2.waitKey(30)==-1:
        cv2.destroyAllWindows()

    # unit of twist
    unit_x = 0
    unit_z = 0

    #run color connected components to filter the counts and centroid
    def ccc_detection(img):
        retval, labels, stats, centroids=cv2.connectedComponentsWithStats(img) #run CCC on the filtered image
        idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.1*image_size))[0]    #threshold the components to find the best one
        for i in idx:
            if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
                return True
        return False
    # red
    if ccc_detection(filtered_red):
        print("red detected")
        unit_x += 1
    # yellow
    if ccc_detection(filtered_yellow):
        print("yellow detected")
        unit_x -= 1
    # blue
    if ccc_detection(filtered_blue):
        print("blue detected")
        unit_z += 1
    
    msg=Twist()	#create message object
    msg.linear.x = 50 * unit_x
    msg.angular.z = 1 * unit_z
    pub.publish(msg)
    
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
sub=rospy.Subscriber('bot_pose',turtle_msg,callback,queue_size=1)
sub_img = rospy.Subscriber('image_raw',Image, img_callback, queue_size=1)

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