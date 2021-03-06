#Code of the RoVeR as of March 4th

#Imports given by the GOAT himself (Owen)
import rospy
import time

# start of wheel control code
# this is a required module for the drive communication
from wheel_control.msg import wheelSpeed  
rospy.init_node("controller")

#global slowDown

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
        self.pointBX = xCoordinate
        self.pointBY = yCoordinate
        self.slowSpeed = self.pointBY - slowDown
    
#Calc the number of beams touching an object on the left
    def distanceCalcLeft(self):
        for l in range (0,7):
            if laser.laserRanges[l] < 4:
                self.leftCounter = self.leftCounter + 1
        return self.leftCounter
    
##Calc the number of beams touching an object on the right
    def distanceCalcRight(self):
        for r in range (7,15):
            if laser.laserRanges[r] < 4:
                self.rightCounter = self.rightCounter + 1
        return self.rightCounter
            
    def turnNow(self):
        minRange = 50 #initialize minRange to a value larger than what will be recieved
        for x in range(0, 15): #iterate through the ranges list
            if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
                minRange = laser.laserRanges[x] #update the range
        if minRange < 2: #if there is something closer than 3m infront of the rover
            if (self.rightCounter>self.leftCounter):
                print("Object on the right")
                wheel.drive_wheels(1, -1)
                self.rightCounter = 0
                self.leftCounter = 0
                #SKRRRT=(LeftTurn())
                print("<<<Left Turn")
            if (self.rightCounter<self.leftCounter):
                print("Object on the left")
                wheel.drive_wheels(-1, 1)
                self.rightCounter = 0
                self.leftCounter = 0
                #SKRRRT=(RightTurn())
                print("Right Turn>>>")
        else:
            wheel.drive_wheels(0.5, 0.5)
            print("Driving Forward")
    
    def stoppyBoiY(self):
        if locHead.y < self.pointBY:
            wheel.drive_wheels(0.0, 0.0)
            print("--Stopping all wheels now--")
        elif locHead.y < self.slowSpeed:
            wheel.drive_wheels(0.1, 0.1)
            print("Slowing down now")
        else:
            wheel.drive_wheels(0.5, 0.5)
    
    def stoppyBoiX(self):
        if locHead.x < self.pointBX:
            wheel.drive_wheels(0.0, 0.0)
            print("--Stopping all wheels now--")
        elif locHead.x < self.slowSpeed:
            wheel.drive_wheels(0.1, 0.1)
            print("Slowing down now")
        else:
            wheel.drive_wheels(0.5, 0.5)
# end of localization stuff

#initiallize classes to get and send data to gazebo
locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()
#Input point B into the turnBoi Class as instance variables
#Example: (xCoordinate, yCoordinate) = (5.0, 10.0)
SKRRRT = turnBoi(-8.0, -20.0, -5)
#end of initialization

# start of control loop snippet
while not rospy.is_shutdown():
    minRange = 50 #initialize minRange to a value larger than what will be recieved
    for x in range(0, 15): #iterate through the ranges list
        if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
            minRange = laser.laserRanges[x] #update the range
    if minRange < 3:
        SKRRRT.distanceCalcLeft()
        SKRRRT.distanceCalcRight()
        SKRRRT.turnNow()
    else:
        SKRRRT.stoppyBoiY()
        SKRRRT.stoppyBoiX()
        
    print("Current Heading: ", locHead.heading, "Current x val: ", locHead.x, "RightMostLaser: ", laser.laserRanges[0])
    print("Current y val: ", locHead.y, "Current z val: ", locHead.z, "LeftMostLaser: ", laser.laserRanges[15])