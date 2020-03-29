#Code of the RoVeR as of March 4th

#Imports given by the GOAT himself (Owen)
import rospy
import time

# start of wheel control code
# this is a required module for the drive communication
from wheel_control.msg import wheelSpeed  
rospy.init_node("controller")

#Class to control the speeds of the wheels and differentiate between different drive wheels 
class WheelController:

    def __init__(self):
        self.wheel_pub = rospy.Publisher("/gazebo_wheelControl/wheelSpeedTopic", wheelSpeed, queue_size=1)

    def drive_wheels(self, left, right):
        msg = wheelSpeed()
        msg.left = left
        msg.right = right
        msg.wheelMode = 0
        self.wheel_pub.publish(msg)
# end of wheel control code

# start of laser scan code
from sensor_msgs.msg import LaserScan

class LaserListener:

    def __init__(self):
        self.laserSub = rospy.Subscriber("/leddar/leddarData", LaserScan, self.laser_callback, queue_size=1)
        self.laserRanges = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

    def laser_callback(self, msg):
        self.laserRanges = msg.ranges
# end of laser scan code access laserRanges for an array of all measured distances from the laser sensors

# start of localization stuff
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class LocationHeading:

    def __init__(self):
        self.fixSub = rospy.Subscriber("/fix/metres", Point, self.fix_callback, queue_size=1)
        self.headingSub = rospy.Subscriber("/heading",Float32, self.heading_callback, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0

    def fix_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def heading_callback(self, msg):
        self.heading = msg.data
        
#This function will decide which way to turn
class turnBoi:

    def __init__(self, xCoordinate, yCoordinate, slowDown):
        self.leftCounter = 0
        self.rightCounter = 0
        self.tightCounter = 0
        self.pointBX = xCoordinate
        self.pointBY = yCoordinate
        self.slowSpeed = self.pointBY - slowDown

#This function looks at which beamns of the LiDAR are coming in contact with objects
    def scan(self):
        if laser.laserRanges[0] <= 1.3:
            collision = True
            self.leftCounter = 7
        elif laser.laserRanges[15] <= 1.3:
            collision = True
            self.rightCounter = 7
        elif laser.laserRanges[1] <= 1.5:
            collision = True
            self.leftCounter = 6
        elif laser.laserRanges[14] <= 1.5:
            collision = True
            self.rightCounter = 6
        elif laser.laserRanges[2] <= 1.7:
            collision = True
            self.leftCounter = 5
        elif laser.laserRanges[13] <= 1.7:
            collision = True
            self.rightCounter = 5
        elif laser.laserRanges[3] <= 2.1:
            collision = True
            self.leftCounter = 4
        elif laser.laserRanges[12] <= 2.1:
            collision = True
            self.rightCounter = 4
        elif laser.laserRanges[4] <= 2.6:
            collision = True
            self.leftCounter = 3
        elif laser.laserRanges[11] <= 2.6:
            collision = True
            self.rightCounter = 3
        elif laser.laserRanges[5] <= 3.6:
            collision = True
            self.leftCounter = 2
        elif laser.laserRanges[10] <= 3.6:
            collision = True
            self.rightCounter = 2
        elif laser.laserRanges[6] <= 5.9:
            collision = True
            self.leftCounter = 1
        elif laser.laserRanges[9] <= 5.9:
            collision = True
            self.rightCounter = 1
        else:
            collision = False
        return collision

#This function decides which way to turn
    def turnNow(self):
        print("Turning")
        if self.rightCounter < self.leftCounter:
            wheel.drive_wheels(1, -1)
            self.rightCounter = 0
            self.leftCounter = 0
            print("Left Turn")
        elif self.leftCounter < self.rightCounter:
            wheel.drive_wheels(-1, 1)
            self.rightCounter = 0
            self.leftCounter = 0
            print("Right Turn")

#This function detects if the rover can fit between onjects
    def tightDrive(self):
        for t in range (3,12):
            if laser.laserRanges[t] < 2:
                self.tightCounter = self.tightCounter + 1
        return self.tightCounter

#This function finds the Y-coordinate
    def detectClose(self):
        if(self.tightCounter > 1):
            wheel.drive_wheels(1, 1)
            
        def stoppyBoiY(self):
        if locHead.y < self.pointBY:
            wheel.drive_wheels(0.0, 0.0)
            print("--Stopping all wheels now--")
        elif locHead.y < self.slowSpeed:
            wheel.drive_wheels(0.1, 0.1)
            print("Slowing down now")
        else:
            wheel.drive_wheels(1, 1)
    
#This function finds the X-coordinate
    def stoppyBoiX(self):
        if locHead.x < self.pointBX:
            wheel.drive_wheels(0.0, 0.0)
            print("--Stopping all wheels now--")
        elif locHead.x < self.slowSpeed:
            wheel.drive_wheels(0.1, 0.1)
            print("Slowing down now")
        else:
            wheel.drive_wheels(1, 1)
                
# end of localization stuff

#initiallize classes to get and send data to gazebo
locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()
SKRRRT = turnBoi()
SKRRRT = turnBoi(0.0, -20.0, -5)
#end of initialization

# start of control loop snippet
while not rospy.is_shutdown():
    minRange = 50 #initialize minRange to a value larger than what will be recieved
    for x in range(0, 15): #iterate through the ranges list
        if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
            minRange = laser.laserRanges[x] #update the range
    if min Range < 3 and SKRRRT.scan():
        SKRRRT.turnNow()
    if minRange < 2:
        SKRRRT.detectCLose()
    else:
        wheel.drive_wheels(0.5, 0.5)
    print("Passed Function")