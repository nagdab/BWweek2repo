#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs

class WallFollower():
    angle=0
    e1=0
    e2=0
    right=True
    death=False
    
    def getError(self,goal,L,begin,end):
        return goal-(min(L[begin:end]))
        
    #get the angle
    def getSteeringCmd(self,error,fullLeft,fullRight):
        Kp =.6
        Kd = .7
        de= error-self.e2
        self.e2=self.e1
        self.e1=error

        u=Kp*error+Kd*de
        return u
    
    #passed to the subscriber
    def callback(self,msg):
        #get the laser information
        if self.right: #right
            error=self.getError(1, msg.ranges, 200, 540)
            if(error>-.5):
                self.angle=self.getSteeringCmd(error, -1, 1)
            else:
                self.angle=1
        else: #left
            error=self.getError(1, msg.ranges, 540,900)
            if(error>-.5):
                self.angle=self.getSteeringCmd(error, -1, 1)
            else:
                self.angle=-1
        
        self.death=min(msg.ranges[525:555])<.5
        self.drive_cmd.drive.steering_angle=self.angle

        if self.death:
            print "Wall follower dead"
            self.drive_cmd.drive.speed=-.1
        else:
            print "Angle is %f" % self.angle
            self.drive_cmd.drive.speed = self.speed
        self.drive.publish(self.drive_cmd) # post this message
        

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        rospy.sleep(1)
    
    def __init__(self,bool_direction):
        print "Beginning wall follow"
        #setup the node
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.right=bool_direction
        
        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        #sets the subscriber
        rospy.Subscriber('scan', LaserScan, self.callback)
        
        # set control parameters
        self.speed = .6 # constant travel speed in meters/second
        
        # fill out fields in ackermann steering message (to go straight)
        self.drive_cmd = AckermannDriveStamped()
        self.drive_cmd.drive.speed = self.speed

        rospy.spin() 
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
if __name__=="__main__":
    WallFollower(True)
