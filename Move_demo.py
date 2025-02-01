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

    def gyroStraightDrive(self, distance, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, stopMethod=None, offset = 0, generator = None, stop = True):
        """
            This is the function that we use to make the robot go forwards or backwards without drifting. It can accelerate, it can slow down and there's also PID. You can set the values
            in a way where you can either drive until the entered distance has been achieved or until the robot senses a line.
            Parameters
            -------------
            distance: the distance that the robot is supposed to drive. Type: Integer. Default: No default value
            speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
            maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
            endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
            addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
            brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
            port: This value tells the program whether the robot is supposed to check for a black line with the specified light snsor. Type: String. Default: 0
            lightValue: This value tells the program the value the robot should stop at if port sees it. Type: Integer. Default: 0
            align_variant: Tells the robot to align itself to a line if it sees one. 0: No alignment. 1: standard alignment. 2: tangent based alignment Type: Integer. Default: 0
            detectLineStart: The value which we use to tell the robot after what percentage of the distance we need to look for the line to drive to. Type: Float. Default: 0
            offset: The value sends the robot in a direction which is indicated by the value entered. Type: Integer. Default: 0
            generator: Function executed while robot is executing gyroStraightDrive. Write the wanted function and its parameters here. Type: . Default: 0
            stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
            generator:  the generator that runs something parallel while driving. Default: No default value
            stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
        """

        if cancel:
            return

        global run_generator, runSmall
        global pRegler, iRegler, dRegler
        
        if generator == None:
            run_generator = False

        #Set starting speed of robot
        speed = startspeed
        #Sets PID values

        change = 0
        old_change = 0
        integral = 0
        steeringSum = 0

        invert = -1

        #Sets values based on user inputs
        loop = True


        gyroStartValue = getGyroValue()

        #Error check for distance
        if distance < 0:
            print('ERR: distance < 0')
            distance = abs(distance)

        #Calulation of degrees the motors should turn to
        #17.6 is wheel circumference in cm. You might need to adapt it
        rotateDistance = (distance / 17.6) * 360
        accelerateDistance = rotateDistance * addspeed
        deccelerateDistance = rotateDistance * (1 - brakeStart)

        #Inversion of target rotation value for negative values
        if speed < 0:
            invert = 1

        #Calculation of braking point
        self.left_Startvalue = self.leftMotor.get_degrees_counted()
        self.right_Startvalue = self.rightMotor.get_degrees_counted()
        brakeStartValue = brakeStart * rotateDistance
        drivenDistance = getDrivenDistance(self)

        while loop:
            if cancel:
                break
            if run_generator: #run parallel code execution
                next(generator)

            #Calculation of driven distance and PID values
            oldDrivenDistance = drivenDistance
            drivenDistance = getDrivenDistance(self)

            pidCalculation(speed)
            change = getGyroValue() - gyroStartValue #yaw angle used due to orientation of the self.hub


            currenSteering = (change * pRegler + integral * iRegler + dRegler * (change - old_change)) + offset + steeringSum*0.02

            currenSteering = max(-100, min(currenSteering, 100))
            #print("steering: " + str(currenSteering) + " gyro: " + str(change) + " integral: " + str(integral))

            steeringSum += change
            integral += change - old_change
            old_change = change

            #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight
            speed = speedCalculation(speed, startspeed,maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)
            self.movement_motors.start_at_power(int(speed), invert * int(currenSteering))


            if stopMethod != None:
                if stopMethod.loop():
                    loop = False
            elif rotateDistance < drivenDistance:                   
                    loop = False


        if stop:
            self.movement_motors.stop()
    
        run_generator = True
        runSmall = True

        return #End of gyroStraightDrive

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

def getDrivenDistance(data):
    #print(str(abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue)) + " .:. " + str(abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)))

    drivenDistance = (
                    abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue) + 
                    abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)) / 2

    return drivenDistance

def pidCalculation(speed):
    #golbally sets PID values based on current speed of the robot, allows for fast and accurate driving
    global pRegler
    global iRegler
    global dRegler
    #Important note: These PID values are experimental and based on our design for the robot. You will need to adjust them manually. You can also set them statically as you can see below
    if speed > 0:
        pRegler = -0.17 * speed + 12.83
        iRegler = 12
        dRegler = 1.94 * speed - 51.9
        if pRegler < 3.2:
            pRegler = 3.2
    else:
        pRegler = (11.1 * abs(speed))/(0.5 * abs(speed) -7) - 20
        iRegler = 10
        #iRegler = 0.02
        dRegler = 1.15**(- abs(speed)+49) + 88

def breakFunction(args):
    """
    Allows you to manually stop currently executing round but still stays in main. 
    This is much quicker and more reliable than pressing the center button.
    """
    global cancel, inMain
    if not inMain:
        cancel = True


hub2.motion.yaw_pitch_roll(0)

db = DriveBase(hub, 'F', 'B')
def exampleOne():
    db.gyroStraightDrive(50, -25, -75, -25) #drives in a straight line for 50cm
    wait_for_seconds(0.15)
    db.gyroStraightDrive(50, 25, 75, 25) #drives in a straight line for 50cm
    return
    
def exampleTwo():
    db.gyroRotation(-90, 25, 35, 25, rotate_mode=0) #same turn as in first rotation but this time turning using only one wheel rather than turning on the spot. Your speeds may need to be higher for this
    wait_for_seconds(0.15)
    db.gyroRotation(90, 25, 35, 25, rotate_mode=1)

class bcolors:
    BATTERY = '\033[32m'
    BATTERY_LOW = '\033[31m'

    ENDC = '\033[0m'

pReglerLight = 1.6
iReglerLight = 0.009
dReglerLight = 16

accelerate = True

hub2.button.right.callback(breakFunction)
gyroValue = 0

#Battery voltage printout in console for monitoring charge
if battery.voltage() < 8000: #set threshold for battery level
    print(bcolors.BATTERY_LOW + "battery voltage is too low: " + str(battery.voltage()) + " \n ----------------------------- \n >>>> please charge robot <<<< \n ----------------------------- \n"+ bcolors.ENDC)
else:
    print(bcolors.BATTERY + "battery voltage: " + str(battery.voltage()) + bcolors.ENDC)

#User Interface in Program for competition and instant program loading
main = True

programselect = 1 #Set the attachment the selection program starts on
hub.light_matrix.write(programselect)
db.movement_motors.set_stop_action("hold") #hold motors on wait for increased reliability


while main:
    cancel = False
    inMain = True

    #Program selection
    if hub.right_button.is_pressed(): #press right button to cycle through programs. cycling back isn't supported yet, but we are working on reallocating the buttons in the file system
        wait_for_seconds(0.15) #waiting prevents a single button press to be registered as multiple clicks
        programselect = programselect + 1
        hub.light_matrix.write(programselect) #show current selcted program
        hub.speaker.beep(85, 0.1) #give audio feedback for user

        if programselect == 1:
            hub.status_light.on('blue')
        elif programselect == 2:
            hub.status_light.on('black')
        elif programselect == 3:
            hub.status_light.on('white')
        elif programselect == 4:
            hub.status_light.on('white')
        elif programselect == 5:
            hub.status_light.on('red')        
        elif programselect == 6:
            hub.status_light.on('orange')
        #cycle to start of stack
        if programselect == 7:
            programselect = 1
            hub.light_matrix.write(programselect)
            hub.status_light.on('blue')

    #Program start
    if hub.left_button.is_pressed():
        inMain = False

        if programselect == 1:
            hub.status_light.on("blue")
            hub.light_matrix.show_image("DUCK")
            exampleOne()
            programselect = 2
            hub.light_matrix.write(programselect)

        elif programselect == 2:
            hub.status_light.on("black")
            hub.light_matrix.show_image("DUCK")
            exampleTwo()
            programselect = 3
            hub.light_matrix.write(programselect)

sys.exit("ended program successfully")
