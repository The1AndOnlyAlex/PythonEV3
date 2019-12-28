#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase






# ------Input--------
power = 60
target = 40
kp = float(0.65) # Proportional gain, Start value 1
kd = 1           # Derivative gain, Start value 0
ki = float(0.02) # Integral gain, Start value 0
direction = -1
minRef = 5
maxRef = 55
# -------------------

# Connect two large motors on output ports B and C and check that
# the device is connected using the 'connected' property.
#left_motor = motor.MediumMotor(OUTPUT_B);  assert left_motor.connected
#right_motor = motor.MediumMotor(OUTPUT_C); assert right_motor.connected
# One left and one right motor should be connected

# Connect color and touch sensors and check that they
# are connected.
# ir = InfraredSensor();    assert ir.connected
#ts = TouchSensor();        assert ts.connected
#col= ColorSensor();    assert col.connected
col = ColorSensor(Port.S3)

# Change color sensor mode
col.mode = 'COL-REFLECT'

def steering(course, power):

    power_left = power_right = power
    s = (50 - abs(float(course))) / 50

    if course >= 0:
        power_right *= s
        if course > 100:
            power_right = - power
    else:
        power_left *= s
        if course < -100:
            power_left = - power

    return (int(power_left), int(power_right))

run_count = 0
def run(power, target, kp, kd, ki, direction, minRef, maxRef, max_angle):
    global run_count
    lastError = error = integral = 0
    #left_motor.run_direct()
    #right_motor.run_direct()
    left_motor.run(power)#60)
    right_motor.run(power)#60)

    #while not btn.any() :
    #if(angle >= max_angle):
    #    return

    while not any(brick.buttons()) :
        refRead = col.reflection()
        error = target - (100 * ( refRead - minRef ) / ( maxRef - minRef ))
        derivative = error - lastError
        lastError = error
        integral = float(0.5) * integral + error
        course = (kp * error + kd * derivative +ki * integral) * direction
        for (motor, pow) in zip((left_motor, right_motor), steering(course, power)):
            motor.dc(pow)
        wait(0.01) # Aprox 100 Hz
        run_count =  run_count + 1
        if(run_count >= max_angle):
            return












# Play a sound.

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# The wheel diameter of the Robot Educator is 56 millimeters.
wheel_diameter = 56

# The axle track is the distance between the centers of each of the wheels.
# For the Robot Educator this is 114 millimeters.
axle_track = 114

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# The following loop makes the robot drive forward until it detects an
# obstacle. Then it backs up and turns around. It keeps on doing this
# until you stop the program.
#while True:
    # Begin driving forward at 200 millimeters per second.
    #robot.drive(200, 200)

    # Wait until an obstacle is detected. This is done by repeatedly
    # doing nothing (waiting for 10 milliseconds) while the measured
    # distance is still greater than 300 mm.

gyro_sensor = GyroSensor(Port.S2)
gyro_sensor.reset_angle(0)

def run1():
    color_sensor3 = ColorSensor(Port.S3)
    #robot.drive(200, 0)
    #wait(50)
    left_motor.run(100)
    while True:
        #robot.drive(200, 0)
        #if (color_sensor3.reflection() < 15):
            #wait(5)
            #break
        #else:
            #wait(5)
        angle = gyro_sensor.angle()
        brick.display.clear()
        brick.display.text(angle, (60, 50))
        if (angle >= 90):
            break
    robot.stop()
    wait(2000)
#run1()


color_sensor2 = ColorSensor(Port.S1)

def line_detect(target_color):
    robot.drive(100, 0)
    while True:
        cs2_value = color_sensor2.reflection()
        if (cs2_value <= target_color):
            break
    wait(100)
    robot.stop()

def turn_left(target_angle):
    right_motor.run(150)
    while True:
        angle = gyro_sensor.angle()
        if (angle <= target_angle):
            break
    robot.stop()

#while True:

#target_angle = 300
gyro_sensor.reset_angle(0)
run_count = 0

run(power, target, kp, kd, ki, direction, minRef, maxRef, 450)
line_detect(10)
turn_left(-90)

run(power, target, kp, kd, ki, direction, minRef, maxRef, 80)
line_detect(10)
turn_left(-180)

run(power, target, kp, kd, ki, direction, minRef, maxRef, 450)
line_detect(10)
turn_left(-270)

run(power, target, kp, kd, ki, direction, minRef, maxRef, 80)
line_detect(10)
turn_left(-360)

robot.stop()

    #line_detect(15)
    #turn_left(-90)

    #line_detect(15)
    #turn_left(-90)