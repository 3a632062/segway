#!/usr/bin/env micropython
"""Segway program."""

# Imports
from time import sleep
from ev3devlight.sensors import Gyro
from ev3devlight.motors import Motor
from filters import Highpass, Differentiator, Integrator

# Configure motors
left = Motor('outD')
left.activate_duty_mode()
right = Motor('outA')
right.activate_duty_mode()

# Configure Gyro
gyro = Gyro('in2', read_rate=True, read_angle=False, calibrate=True)
angle = 0

# Configure filters
looptime = 0.01
highpass = Highpass(looptime, 0, 0.01)
differentiator = Differentiator(looptime, 5)
integrator = Integrator(looptime)

# Parameters
deg2rad = 180/3.14
wheel_diameter = 4.32
cm_per_degree = 3.14/180*wheel_diameter/2
busytime = 0.005

# Control gains
gain_angle = 70
gain_rate = 1.2
gain_distance = 15
gain_speed = 3

# Main loop
while True:
    # Body angle
    rate = highpass.filter(gyro.rate)
    angle = integrator.integral(rate)

    # Wheel translation
    average_motor_degrees = (right.position + left.position)/2
    distance = average_motor_degrees * cm_per_degree
    speed = differentiator.derivative(distance)

    # Control signal
    duty = gain_angle*angle + gain_rate*rate + \
        + gain_distance*distance + gain_speed*speed

    # Motor actuation
    left.duty(duty)
    right.duty(duty)

    # Sleep
    sleep(looptime-busytime)
