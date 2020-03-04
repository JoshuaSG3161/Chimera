#start of code snippets


import rospy  # this is the module required for all simulation communication
import time

start_time = time.time()

# start of wheel control code
from wheel_control.msg import wheelSpeed  # this is a required module for the drive communication

rospy.init_node("controller")

class WheelController:

    def __init__(self):
        self.wheel_pub = rospy.Publisher("/gazebo_wheelControl/wheelSpeedTopic", wheelSpeed, queue_size=1)

    def drive_wheels(self, left, right):
        # type: (float, float) -> None
        # left and right are numbers between -1 and 1
        msg = wheelSpeed()
        msg.left = left
        msg.right = right
        msg.wheelMode = 0
        self.wheel_pub.publish(msg)
        #print(msg)


# end of wheel control code

# start of laser scan code
from sensor_msgs.msg import LaserScan


class LaserListener:

    def __init__(self):
        self.laserSub = rospy.Subscriber("/leddar/leddarData", LaserScan, self.laser_callback, queue_size=1)
        self.laserRanges = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

    def laser_callback(self, msg):
        # type: (LaserScan) -> None
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
        # type: (Point) -> None
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def heading_callback(self, msg):
        # type: (Float32) -> None
        self.heading = msg.data
        
#This function will decide which way to turn
class turnBoi:

    
    def __init__(self):
        self.leftCounter = 0
        self.rightCounter = 0
    
#Calc the number of beams touching an object on the left
    def distanceCalcLeft(self):
        for l in range (0,7):
            if laser.laserRanges[l] < 5:
                self.leftCounter = self.leftCounter + 1
        return self.leftCounter
    
##Calc the number of beams touching an object on the right
    def distanceCalcRight(self):
        for r in range (8,15):
            if laser.laserRanges[r] < 5:
                self.rightCounter = self.rightCounter + 1
        return self.rightCounter
        
#Turn Rover Boi left
#    def LeftTurn():
#        minRangeL = 50
#        for x in range(0, 7): #iterate through the ranges list
#            if laser.laserRanges[x] < minRangeL: #if the current range is smaller than the smallest know range
#                minRangeL = laser.laserRanges[x] #update the range
#        if minRangeL < 5: #if there is something closer than 3m infront of the rover
#            wheel.drive_wheels(1, -1) #turn
#        else:
#            wheel.drive_wheels(1, 1) #go staright
        
#Turn Rover Boi right
#    def RightTurn():
#        minRangeR = 50
#        for x in range(8, 15): #iterate through the ranges list
#            if laser.laserRanges[x] < minRangeR: #if the current range is smaller than the smallest know range
#                minRangeR = laser.laserRanges[x] #update the range
#        if minRangeR < 5: #if there is something closer than 3m infront of the rover
#            wheel.drive_wheels(-1, 1) #turn
#        else:
#            wheel.drive_wheels(1, 1) #go staright
            
    def turnNow(self):
        minRange = 50 #initialize minRange to a value larger than what will be recieved
        for x in range(0, 15): #iterate through the ranges list
            if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
                minRange = laser.laserRanges[x] #update the range
        if minRange < 5: #if there is something closer than 3m infront of the rover
            print("Turningggggggggggggggggggg")
            if (self.rightCounter>self.leftCounter):
                wheel.drive_wheels(1, -1)
                #SKRRRT=(LeftTurn())
                print("Left Turn")
            if (self.rightCounter<self.leftCounter):
                wheel.drive_wheels(-1, 1)
                #SKRRRT=(RightTurn())
                print("Right Turn")
        else:
            wheel.drive_wheels(1, 1)
            print("Driving Forward")

        
# end of localization stuff


#initiallize classes to get and send data to gazebo
locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()
SKRRRT = turnBoi()
#end of initialization

# start of control loop snippet

while not rospy.is_shutdown():
    SKRRRT.distanceCalcLeft()
    SKRRRT.distanceCalcRight()
    SKRRRT.turnNow()
    print("Passed Function")
    
#    minRange = 50 #initialize minRange to a value larger than what will be recieved
#    for x in range(0, 15): #iterate through the ranges list
#        if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
#            minRange = laser.laserRanges[x] #update the range
#    if minRange < 5: #if there is something closer than 3m infront of the rover
#        wheel.drive_wheels(1, -1) #turn
#    else:
#        wheel.drive_wheels(1, 1) #go staright
#    if (time.time() - start_time) > 10:
#        wheel.drive_wheels(0,0)
#    print("Pizza Time")


#while not rospy.is_shutdown():  #this will run until gazebo is shut down or CTRL+C is pressed in the ubuntu window that is running this code
#    minRange = 50 #initialize minRange to a value larger than what will be recieved
#    y=0
#    z=0
#    for x in range(0, 15): #iterate through the ranges list
#        if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
#            if (x<7):
#                y =+ 1
#            else:
#                z =+ 0
#            minRange = laser.laserRanges[x] #update the range
#    #if minRange < 3: #if there is something closer than 3m infront of the rover
#    if (y>z):
#        wheel.drive_wheels(1, -1) #turn
#    if (z>y):
#        wheel.drive_wheels(-1,1)
#        #if laser.laserRanges[1:6] < 5: #If object on left, turn right
#            #wheel.drive_wheels(1, -1) #turn
#        #if laser.laserRanges[7:15] < 5:
#            #wheel.drive_wheels(-1, 1) #turn
#    else:
#        wheel.drive_wheels(1, 1) #go staright
##    if (start_time - time.time()) > 10000:
##        wheel.drive_wheels(0,0)


