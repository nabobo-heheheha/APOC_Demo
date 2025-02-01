from spike import Motor
from spike.control import wait_for_seconds

# Initialize the motor B
motor_b = Motor('B')  # Motor B

# turn the motor for 360 degrees
motor_b.run_for_degrees(360, speed=50)  # set the speed to 50

# start the motor
motor_b.stop()

# delay the motor
wait_for_seconds(2)

