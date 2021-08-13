# ROS Survey
The structure of ROS is Publisher-Subscriber relationship among different nodes. The goal for this trial is to create python turtle node with service and subscriber, a python node as client publishing turtle actions, along with a given webcam image publisher to control the on screen turtle based on different color the webcam sees.

**Please time yourself for each Checkpoint.**
## ROS Resources:
* rospy: http://wiki.ros.org/rospy
* Catkin workspace: http://wiki.ros.org/catkin/workspaces
* ROS message: http://wiki.ros.org/msg
* ROS service: http://wiki.ros.org/srv
* Example rospy Publisher/Subscriber: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
* Full ROS online tutorial: http://wiki.ros.org/ROS/Tutorials
## Outline
### Setup
* [Catkin Workspace](#catkin-workspace)
* [Package](#package)
* [Message Types](#message-types)
* [Service Types](#service-types)
* [Build Workspace](#build-workspace)
### ROS Publisher
### ROS Subscriber
* [Create Turtlebot Server](#create-turtlebot-server)
* [Create Turtlebot Client](#create-turtlebot-client)


# Setup
## Catkin Workspace
For each ROS project there's a dedicated catkin workspace, and in this trial the workspace is `~/Robotics_Middleware_Trial_Python_Turtle/ROS` already created in the repo.
## Package
Unlike RobotRaconteur, ROS requires the workspace to build the content. All packages should be in `workspace/src/` folder. In this repository there's already a webcam package (`~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/webcam/`), so you'll need to [create another package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for python turtle:
```
cd ~/Robotics_Middleware_Trial_Python_Turtle/ROS #Eric: should be ROS/src
catkin_create_pkg python_turtle std_msgs geometry_msgs rospy
```
This creates a new package `python_turtle` under `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src`, with dependencies of `std_msgs`, `geometry_msgs` and `rospy`.

## Message Types
Similar to RobotRaconteur service definition, for ROS there're [message types](http://wiki.ros.org/Messages) and [service types](http://wiki.ros.org/Services). In the task we'll need to create our own message and service types.
When building ROS, many message and service types are built together, which you can look up online: http://wiki.ros.org/common_msgs. 
In the task we'll ask to create your own message type `turtle_ros`:
```
string name
geometry_msgs/Pose turtle_pose
string color
```
This bascially shows the message contains the name of the turtle, its pose and color. Since this message is part of the `python_turtle` package, create a folder under `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/python_turtle/` called `msg`. Then create a file named `turtle_msg.msg`, and copy above message definition into this file as your own message type.
#Eric: should have the cmd here for newbies

In order to let the compiler know and build the message for you, it's necessary to modify the `package.xml` as well as `CMakeLists.txt` under package `python_turtle`. So first open up `package.xml` and uncomment below two lines:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
#Eric: should have instruction here about ucomment
Then open up `CMakeLists.txt`, search for lines below:
```
find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   geometry_msgs
)
```
Add `message_generation` into `find_package`.

To export the message runtime dependency, inside `CMakeLists.txt` look for 
```
catkin_package(
...
)
```
Add `CATKIN_DEPENDS message_runtime` within `catkin_package`.

Then find the following block of code:
```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
Change it to 
```
add_message_files(
  FILES
  turtle_msg.msg
)
```
Finally, look for lines below:
```
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
...
# )
```
And change them to
```
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
```
By this point, the message type should be built together when building the package.

You can include the message type in the similar way of ROS Image type:
```
from python_turtle import turtle_msg
from geometry_msgs import Pose
```
And to create an object of that message type:
```
turtle_obj=turtle_msg()
turtle_obj.name="myturtle"
turtle_obj.turtle_pose=Pose()
turtle_obj.color="red"
```

## Service Types
A ROS service is similar to a function call, and in the task we'll ask you to create a `setpose` and `setcolor`:

`setpose`:
```
geometry_msgs/PoseStamped turtle_pose
---
int8 ret
```
`setcolor`:
```
string color
---
int8 ret
```
ROS service has to have a return type, so we can simply return an `int` instead of `void`. 

Generating a ROS service type is similar to generating a message type. First create a folder under `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/python_turtle/` called `srv`. Then create two files named `setpose.srv` and `setcolor.srv`, and copy above service definitions into these files as your own service types.
Some steps are overlapped when creating services and messages, so only a bit differences. Navigate to `CMakeLists.txt` under package `python_turtle`, open it up and look for code below:
```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
``` 
And modify it to
```
add_service_files(
  FILES
  setcolor.srv
  setpose.srv
)
```

To set up a service, it's necessary to initialize it first:
```
self.pose_server=rospy.Service('setpose', setpose, self.set_pose)
```
and the function is provided later in the same class:
```
def set_pose(self,req):
	self.turtle.turtle_pose=req.turtle_pose.pose
	return 1
```
Remember to build your workspace and source it to get your service types exposed.

## Build Workspace
Type in following commands to build your workspace:
```
cd ~/Robotics_Middleware_Trial_Python_Turtle/ROS
catkin_make #Eric: should have a source /opt/ros/melodic/setup.bash here
```
It should finish without errors, and generating `/build` and `/devel` folders under the same directory. However, to make sure your code knows what you've built, we need to source it:
```
$ echo 'source ~/Robotics_Middleware_Trial_Python_Turtle/ROS/devel/setup.bash' >> ~/.bashrc 
```
This step adds the command everytime you open up a new terminal. If errors like something not found or not built, try `source ~/Robotics_Middleware_Trial_Python_Turtle/ROS/devel/setup.bash` to source the workspace directly.

* **Checkpoint 1**: 
Build should be successful without any errors. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.

# ROS Publisher
#Eric: I think should have a concept explaination here
#Eric: The codes are squeezing together. It's a bit annoying.
Under `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/webcam/src/` there is a python script called `cam_pub.py`. At the very top, we include ROS library and message types:
``` 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
```
The `Webcam_impl()` class is a webcam class, which contains camera metadata and a `CaptureFrame()` function. Then take a look at `main`:
```
pub = rospy.Publisher('image_raw', Image, queue_size=0) 
rospy.init_node('webcam', anonymous=True) #Eric: Is it anonymous=True or False?
```
Here ROS node is initialized with a publisher, published to topic `image_raw` of type `Image` ([sensor_msgs/Image.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
```
while not rospy.is_shutdown(): 
```
This while loop holds the sciprt from exiting until ROS is shutdown, and inside the loop:
```
frame=webcam.CaptureFrame() 
pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8")) 
```
The image is captured and convert to ROS Image type, and finally published to the topic by the publisher.
To run this script, open a new terminal and run `$ roscore`. After that, you can run this script by `python cam_pub.py`.
For every ROS communication, there needs to be one and only one roscore running. To check if the images are successfully published or not, open up a new terminal and type in `$ rostopic echo image_raw`.
This way the terminal shall display the raw image data.
#Eric: should change the permission of the file to executable
#Eric: should tell them that if use rostopic echo for images, there will be a lot of number jumping in the terminal.
#Eric: should tell them how to terminate a rosrun or rostopic echo

* **Checkpoint 2**: 
After the publisher runs, in the separate window, `$ rostopic echo image_raw` will display the raw image data. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.

# ROS Subscriber

On your computer side, under `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/webcam/src/` there is a python script called `cam_sub.py`. We include ROS library and message types at the top.
Unlike a publisher, a subscriber subscribe to the topic, and trigger the `callback()` function. Inside main, 
```
rospy.init_node('stream_node', anonymous=True) #Eric: so is it True or False?
sub = rospy.Subscriber("image_raw",Image,callback)
rospy.spin()
```
ROS node is intialized, and a subscriber `sub` is set up to subscribe to ROS topic `image_raw`, with `Image` type, triggering `callback()` function. `rospy.spin()` keeps this script running until user shutdown. Now let's take a look at the `callback()` function.
`def callback(data)` means this function takes in an argument of `data`, which should be the message type specified in the subscriber setup (`Image`). 
```
try:
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")	#convert ros image message to opencv object
except CvBridgeError as e:
	print(e)
```
Above lines basically convert the `Image` type data into an OpenCV image object, so that it could be displayed out on screen. Note that the `callback()` is only triggered upon messaged received, so if you would like to use the data/message at other places, it's best to make it global.

* **Checkpoint 3**: 
There should be a popup window displaying the realtime image from webcam. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.
#Eric: The script is missing with the line #!/usr/bin/env python
#Eric: And also the executable permission setup.

## Create Turtlebot Server
All script about turtles should be in `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/python_turtle/src/` directory, so let's create a file named `turtlebot_service.py`. The function of this script is to keep track of the turtle (pose and color).
First, it's necessary to include ROS and relative libraries and messages:
```
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from python_turtle.msg import turtle_msg
from python_turtle.srv import setcolor, setpose
```
Here `Twist` and `Pose` messages are two standard ones in [geometry messages](http://wiki.ros.org/geometry_msgs), and `turtle_msg` is the one we created earlier.
Then let's also create a python class object called `create_turtle`, and initialize ROS publisher, subscriber and services:
```
class create_turtle:
	def __init__(self):               			#initialization upon creation
		#message type initialziation
		self.turtle_pose=Pose()
		self.turtle=turtle_msg()
		#subscriber initialization
		self.vel_sub=rospy.Subscriber('<drive topic name>',<message type>,self.drive_callback)
		#publisher initialization
		self.turtle_pub=rospy.Publisher('<turtle topic name>',<message type>,queue_size=1)
		#service initialization
		self.pose_server=rospy.Service('<service name>', <service type>, self.set_pose)
		self.color_server=rospy.Service('<service name>', <service type>, self.set_color)
```
Note that in the service there're function `drive_callback`, `set_pose` and `set_color`, therefore we'll also need to implement those so when a service is called or a subscriber receives message, the function will be triggered. Inside the same class,
```
	def drive_callback(self,data):            #Drive function, update new position, this is the one referred in definition
		self.turtle.turtle_pose.position.x=<decode message type and update pose>
		self.turtle.turtle_pose.position.y=<decode message type and update pose>
		self.turtle.turtle_pose.orientation.z=<decode message type and update pose>

	def set_pose(self,req):		#update pose based on given pose
		self.turtle.turtle_pose=req.turtle_pose.pose
		return 1
	def set_color(self,req):	#set color based on given color
		self.turtle.color=req.color
		return 1
```
After having a complete class, it's necessary to initialize a ROS node as well as create the class object:
```
rospy.init_node('<random node name>', anonymous=True)
turtle_obj=create_turtle()
```
Note that the goal for this script is to keep track of the turtle, so we also need to publish the turtle pose continuously.
```
while not rospy.is_shutdown():
	turtle_obj.turtle_pub.publish(turtle_obj.turtle)
	time.sleep(0.01)
```
By filling up the `<>` sections above, you should have a complete turtlebot server.

* **Checkpoint 4**: 
Make sure have one and only one `roscore` running.

Run the script with `$ python turtlebot_service.py`, it should be running with no error messages. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.

Eric: There are a lot of confusions here
Eric: 1. The drive callback is insufficient. You can't just conver the twist message to pose like that. There should be a inverse kinematic and control loop with a designated control frequency here.
Eric: 2. There shall be more hint to the messagetype etc.
Eric: 3. Change the permission of the script to executable.

## Create Turtlebot Client
Now let's create a simple ROS client for the turtlebot. Create a new file and name it `turtlebot_client.py`. First import the ROS and other essential libraries at top:
```
import rospy     #import ROS library
import turtle
from geometry_msgs.msg import Twist
from python_turtle.msg import turtle_msg
```
Similar to the `turtlebot.py`, we need to initialize the screen and a turtle first:
```
#display setup
screen = turtle.Screen()
screen.bgcolor("lightblue")
#turtle setup
t1=turtle.Turtle()
t1.shape("turtle")
```
Then it's necessary to create a variable keeping the turtle pose information from ROS subscriber callback:
```
turtle_obj=turtle_msg()
def callback(data):
	global turtle_obj
	turtle_obj=data
```
And it's also necessary to have an `update()` function to update display:
```
def update()
	global turtle_obj
	if turtle_obj.color=="None":
		t1.penup()
	else:
		t1.pencolor('<color>')

	t1.setpos(<x coordinate>,<y coordinate>)
	t1.seth(<angle>)
```
Eric: A : is mission here.
Eric: Should convert quaternion to rpy
Finally, we initialize ROS node as well as the publisher and subscriber:
```
rospy.init_node('<random node name>', anonymous=False)
pub=rospy.Publisher('<drive topic name>',<message type>,queue_size=1)
sub=rospy.Subscriber('<turtle topic name>',<message type>,callback)
```
Eric: It's better to remind the user that the topic name should match with the one in the previous script.
After ROS is initialized, let's also have the turtle running a circle:
```
while not rospy.is_shutdown():
        update()
	msg=Twist()	#create message object
	msg.linear.x=10
	msg.angular.z=10
	pub.publish(msg)  
```

* **Checkpoint 5**:
Run the script with `$ python turtlebot_client.py`, it bring up a window and drive the turtle in a circle. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.

All rospy scripts can run with `python` command, but make sure have one and only one `roscore` running.

Eric: Change the permission of the script to executable.

# Task
## 1
From tutorial above, you should have a complete turtlebot subscriber and a simple turtlebot publisher. Given `keyboard.py` under `~/Robotics_Middleware_Trial_Python_Turtle/Examples`, try creating a script `client_keyboard.py` under `~/Robotics_Middleware_Trial_Python_Turtle/ROS/src/python_turtle/src/` that display the turtle as well as reading in inputs from the keyboard to drive the turtle accordingly.

* **Checkpoint 6**:
Run the script with `$ python client_keyboard.py`, it bring up a window. In the terminal, arrow key press will drive the turtle on screen accordinly. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.

## 2
![](ROS.JPG)

The final goal is to create a server-client model above, with the turtle server node keeps track of the state of the turtle, the webcam captures image continuously and publish them, the client node subscribe and display the turtle with python turtle module and command the turtle through publisher based on image content. The webcam publisher is provided and the turtle server node is created through tutorial.

Given and detection example `Examples/detection.py`, create the final client node subscribing images from the webcam, process the image and publishing command to drive the turtle based on the color detected in your webcam.

* **Checkpoint 7**:
By pointing the webcam at different colors (R/G/B), the turtle on screen should drive based on the color seen. Please direct to `Robotics_Middleware_Trial_Python_Turtle/readme.md` for question post.

# General Recommendation
Eric: Why use `python script.py` but not `rosrun package script`?