# LEGO type:standard slot:1 autostart

import math
from spike import PrimeHub, Motor, MotorPair, ColorSensor
from spike.control import wait_for_seconds, Timer
from hub import battery
hub = PrimeHub()


import hub as hub2

import sys

#Preperation for parallel code execution
accelerate = True
run_generator = True
runSmall = True

lastAngle = 0
oldAngle = 0

gyroValue = 0

# Create your objects here.
hub = PrimeHub()

#PID value Definition
pRegler = 0.0
iRegler = 0.0
dRegler = 0.0

pReglerLight = 0.0
iReglerLight = 0.0
dReglerLight = 0.0

"""
Initialize Motor
left Motor: port F
right Motor: port B
"""

smallMotorA = Motor('F')
smallMotorD = Motor('B')

#Set variables based on robot
circumference = 17.6 #circumference of the wheel powered by the robot in cm
sensordistance = 7 #distance between the two light sensors in cm. Used in Tangent alignment 6.4 in studs

cancel = False
inMain = True

class DriveBase:

    def __init__(self, hub, leftMotor, rightMotor):
        self.hub = hub
        self.leftMotor = Motor(leftMotor)
        self.rightMotor = Motor(rightMotor)
        self.movement_motors = MotorPair(leftMotor, rightMotor)

    def gyroRotation(self, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, rotate_mode = 0, stopMethod = None, generator = None, stop = True):
        """
            This is the function that we use to make the robot turn the length of a specific angle or for the robot to turn until it senses a line. Even in this function the robot
            can accelerate and slow down. It also has Gyrosensor calibrations based on our experimental experience.
            Parameters
            -------------
            angle: The angle which the robot is supposed to turn. Use negative numbers to turn counterclockwise. Type: Integer. Default value: No default value
            startspeed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The percentage after which the robot reaches the maxspeed. Type: Float. Default: No default value
            brakeStart: The percentage after which the robot starts slowing down until it reaches endspeed. Type: Float. Default: No default value
            rotate_mode: Different turning types. 0: Both motors turn, robot turns on the spot. 1: Only the outer motor turns, resulting in a corner. Type: Integer. Default: 0
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """

        if cancel:
            return

        global run_generator, runSmall

        if generator == None:
            run_generator = False

        if rotate_mode == 0:
            startspeed = abs(startspeed)
            maxspeed = abs(maxspeed)
            endspeed = abs(endspeed)

        speed = startspeed

        #set standard variables
        rotatedDistance = 0
        steering = 1

        accelerateDistance = abs(angle * addspeed) 
        deccelerateDistance = abs(angle * (1 - brakeStart))

        #gyro sensor calibration
        angle = angle * (2400/2443) #experimental value based on 20 rotations of the robot

        #Setting variables based on inputs
        loop = True
        gyroStartValue = getGyroValue() #Yaw angle used due to orientation of the self.hub. This might need to be changed
        brakeStartValue = (angle + gyroStartValue) * brakeStart

        #Inversion of steering value for turning counter clockwise
        if angle < 0:
            steering = -1

        #Testing to see if turining is necessary, turns until loop = False

        while loop:
            if cancel:
                break

            if run_generator: #run parallel code execution
                next(generator)

            oldRotatedDistance = rotatedDistance
            rotatedDistance = getGyroValue() #Yaw angle used due to orientation of the self.hub. This might need to be changed
            speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, abs(1), abs(0))
            
            
            #Checking for variants
            #Both Motors turn, robot moves on the spot
            if rotate_mode == 0:
                self.movement_motors.start_tank_at_power(int(speed) * steering, -int(speed) * steering)
            
            #Only outer motor turns, robot has a wide turning radius
            elif rotate_mode == 1:
                if angle * speed > 0:
                    self.leftMotor.start_at_power(- int(speed))
                else:
                    self.rightMotor.start_at_power(+ int(speed))

            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
                    break
            elif abs(angle) <= abs(rotatedDistance - gyroStartValue):                   
                    loop = False
                    break



        #Stops movement motors for increased accuracy while stopping
        if stop:
            self.movement_motors.stop()

        run_generator = True
        runSmall = True

        return # End of gyroStraightDrive

def getGyroValue():

    #this method is used to return the absolute gyro Angle and the angle returned by this method doesn't reset at 180 degree
    global lastAngle
    global oldAngle
    global gyroValue

    #gets the angle returned by the spike prime program. The problem is the default get_yaw_angle resets at 180 and -179 back to 0
    angle = hub.motion_sensor.get_yaw_angle()

    if angle != lastAngle:
        oldAngle = lastAngle
        
    lastAngle = angle

    if angle == 179 and oldAngle == 178:
        hub2.motion.yaw_pitch_roll(0)#reset
        gyroValue += 179
        angle = 0
    
    if angle == -180 and oldAngle == -179:
        hub2.motion.yaw_pitch_roll(0) #reset
        gyroValue -= 180   
        angle = 0

    return gyroValue + angle

def speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance):
    """
        Used to calculate all the speeds in out programs. Done seperatly to reduce redundancy. Brakes and accelerates
        Parameters
        -------------
        speed: The current speed the robot has
        startspeed: Speed the robot starts at. Type: Integer. Default: No default value.
        maxspeed: The maximum speed the robot reaches. Type: Integer. Default: No default value.
        endspeed: Speed the robot aims for while braking, minimum speed at the end of the program. Type: Integer. Default: No default value.
        addspeed: Percentage of the distance after which the robot reaches the maximum speed. Type: Integer. Default: No default value.
        brakeStartValue: Percentage of the driven distance after which the robot starts braking. Type: Integer. Default: No default value.
        drivenDistance: Calculation of the driven distance in degrees. Type: Integer. Default: No default value.
    """    

    addSpeedPerDegree = (maxspeed - startspeed) / accelerateDistance 
    subSpeedPerDegree = (maxspeed - endspeed) / deccelerateDistance
    

    subtraction = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * subSpeedPerDegree
    addition = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * addSpeedPerDegree

    if abs(drivenDistance) > abs(brakeStartValue):

        if abs(speed) > abs(endspeed):
            speed = speed - subtraction
            
    elif abs(speed) < abs(maxspeed):

        speed = speed + addition

    return speed


hub2.motion.yaw_pitch_roll(0)

db = DriveBase(hub, 'F', 'B')
db.gyroRotation(-90, 25, 35, 25, rotate_mode=0) #same turn as in first rotation but this time turning using only one wheel rather than turning on the spot. Your speeds may need to be higher for this
hub.left_button.wait_until_pressed()
wait_for_seconds(0.15)
db.gyroRotation(90, 25, 35, 25, rotate_mode=1)

sys.exit("ended program successfully")
