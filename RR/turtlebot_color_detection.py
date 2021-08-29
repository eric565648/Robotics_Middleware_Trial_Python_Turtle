from RobotRaconteur.Client import *     #import RR client library
import turtle
import termios, fcntl, sys, os
import math
import cv2
import numpy as np

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


# turtlebot service subscribe
url='rr+tcp://localhost:2355/?service=turtlesim'
#take url from command line
# if (len(sys.argv)>=2):
# 	url=sys.argv[1]

sub=RRN.SubscribeService(url)
turtle_obj = sub.GetDefaultClientWait(5)		#wait for 5 seconds timeout if no object returned
turtle_pose_wire=sub.SubscribeWire('turtle_pose_wire')		#subscribe to the wire name

# image service subscribe
cam_url = 'rr+tcp://localhost:2356/?service=Webcam'
cam_sub = RRN.SubscribeService(cam_url)
cam_obj = cam_sub.GetDefaultClientWait(5)

def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

print("Running")
while True:
    try:

        unit_x = 0
        unit_z = 0

        cv_image = WebcamImageToMat(cam_obj.image)
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
    
        turtle_obj.drive(50*unit_x,1*unit_z)
        update(turtle_obj, turtle_pose_wire)		#update in display

    except IOError: pass
    except TypeError: pass