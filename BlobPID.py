import rospy
import math
import numpy
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from racecar.msg import BlobDetections
from numpy.core.defchararray import lower
import WallFollower

class BlobPID():
    done=False
    e1=0
    e2=0
    blobColor="RED"
    def die(self):
        self.done=True
	print("got killed")
        isRed = self.blobColor=="RED"
        print "Blob color is %s" % self.blobColor
        w = WallFollower.WallFollower(isRed)
	print(w.testparam)
        

    #get the angle
    def getSteeringCmd(self,error,fullLeft,fullRight):
        Kp = 1
        Kd = .7
        de= (error-self.e2)
        self.e2=self.e1
        self.e1=error
        u=Kp*error+Kd*de
        return u
    
    
    #passed to the subscriber
    def callback(self,msg):
        if self.done:
	    print("oh boy")
	    return
        try:
            print msg.sizes[0]
            
            self.drive_cmd.drive.steering_angle=self.getSteeringCmd(.5-msg.locations[0].x,-1,1)
            
            print(self.drive_cmd.drive.speed) 
            if(msg.colors[0].r>msg.colors[0].g):
                self.blobColor = "RED"
            else:
                self.blobColor = "GREEN"

            #Die if close enough  
            if msg.sizes[0].data>=9000.0: 
                self.die()
            
        except Exception, e:
            self.drive_cmd.drive.steering_angle = 0
	    print(e)
	print(self.drive_cmd.drive.steering_angle)
        self.drive.publish(self.drive_cmd) # post this message
    
    def __init__(self):
        #setup the node
        rospy.init_node('BlobPID', anonymous=False)
        
        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        #sets the subscriber
        rospy.Subscriber('blob_detections', BlobDetections, self.callback)
        
         # constant travel speed in meters/second
        speed = 2
        
        # fill out fields in ackermann steering message (to go straight)
        self.drive_cmd = AckermannDriveStamped()
        self.drive_cmd.drive.speed = speed
        rospy.spin()
def die():
    print "We dead"
    rospy.loginfo("I died")

if __name__=="__main__":
    try:
        BlobPID()
    except Exception:
        die()
