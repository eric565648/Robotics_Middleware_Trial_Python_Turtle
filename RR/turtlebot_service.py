import threading
import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

import math

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
        
        #RR property
        self.turtle_pose=RRN.NewStructure("experimental.turtlebot_create.pose") #create RR structure obj
        self.color="None"

        self.lin_x = 0
        self.ang_z = 0
        self.dirty = False

        # set rospy timer for drive update
        self.interval = 0.04
        t = threading.Timer(self.interval, self.drive_update)
        t.start()

    def drive_callback(self, move_speed,turn_speed):            
        #Drive function, update new position, this is the one referred in definition
        
        self.lin_x = move_speed
        self.ang_z = turn_speed
        self.dirty = True
        
        # so strange here!!!!!!!!!!!!!!!!!
        # self.turtle.turtle_pose.position.x = data.position.x
        # self.turtle.turtle_pose.position.y = data.position.y
        # self.turtle.turtle_pose.orientation.z = <decode message type and update pose>

    def drive_update(self):

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

        self.turtle_pose_wire.OutValue = self.turtle_pose

        self.dirty = False

    def set_pose(self,turtle_pose):		
        #update pose based on given pose
        self.turtle_pose = turtle_pose
        self.turtle_pose_wire.OutValue=self.turtle_pose
        return 1

with RR.ServerNodeSetup("Turtlebot_Service", 2356):      #setup RR node with service name and port
    #Register the service type
    RRN.RegisterServiceTypeFromFile("robdef/experimental.turtlebot_create.robdef")               
    #create object
    create_inst=CreateTurtle()                
    #Register the service with definition and object
    RRN.RegisterService("turtlesim","experimental.turtlebot_create.turtlesim",create_inst)
    #Wait for program exit to quit
    input("Press enter to quit")