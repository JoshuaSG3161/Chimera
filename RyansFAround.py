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

    def __init__(self):
        self.leftCounter = 0
        self.rightCounter = 0        
    
###Calc the number of beams touching an object on the left
##    def distanceCalcLeft(self):
##        for l in range (0,8):
##            if laser.laserRanges[l] < 4:
##                self.leftCounter = self.leftCounter + 1
##        return self.leftCounter
##    
####Calc the number of beams touching an object on the right
##    def distanceCalcRight(self):
##        for r in range (8,15):
##            if laser.laserRanges[r] < 4:
##                self.rightCounter = self.rightCounter + 1
##        return self.rightCounter

    def scan(self):
        if laser.laserRanges[0] <= 1.3:
            collision = True
            self.leftCounter = 7
        elif laser.laserRanges[15] <= 1.3:
            collision = True
            self.rightCounter = 7
##        elif laser.laserRanges[0] and laser.laserRanges[15] <= 1.3:
##            collision = True
##            wheel.drive_wheels(-0.5,-0.5)
##            time.sleep(3)
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

    def turnNow(self):
##        minRange = 50 #initialize minRange to a value larger than what will be recieved
##        for x in range(0, 15): #iterate through the ranges list
##            if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
##                minRange = laser.laserRanges[x] #update the range
##        if minRange < 3: #if there is something closer than 3m infront of the rover
        print("Turning")
        if self.rightCounter < self.leftCounter:
            wheel.drive_wheels(1, -1)
            self.rightCounter = 0
            self.leftCounter = 0
            #SKRRRT=(LeftTurn())
            print("Left Turn")
        elif self.leftCounter < self.rightCounter:
            wheel.drive_wheels(-1, 1)
            self.rightCounter = 0
            self.leftCounter = 0
            #SKRRRT=(RightTurn())
            print("Right Turn")
##        else:
##            wheel.drive_wheels(1, 1)
##            print("Driving Forward")   

                
# end of localization stuff

#initiallize classes to get and send data to gazebo
locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()
SKRRRT = turnBoi()
#end of initialization

# start of control loop snippet
while not rospy.is_shutdown():
    if SKRRRT.scan():
##        wheel.drive_wheels(0,0)
        SKRRRT.turnNow()
    else:
        wheel.drive_wheels(0.5,0.5)
    print("Passed Function")



