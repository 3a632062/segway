#!/usr/bin/env micropython
"""Segway program."""

# Imports
from time import sleep, time
from ev3devlight.sensors import Gyro, Analog
from ev3devlight.motors import Motor
from ev3devlight.brick import Battery
from filters import Highpass, Differentiator, Integrator, Calibrator

# Configure motors
left = Motor('outD')
left.activate_duty_mode()
right = Motor('outA')
right.activate_duty_mode()

# Battery
battery = Battery()
nominal = 8
battery_scaling = nominal / battery.voltage

# Configure and calibrate LEGO Gyro
gyro = Gyro('in2', read_rate=True, read_angle=False, calibrate=True)
initial_bias = 0

# Configure filters
looptime = 0.02
highpass = Highpass(looptime, initial_bias, 0.02)
differentiator = Differentiator(looptime, 5)
integrator = Integrator(looptime)

# Parameters
deg2rad = 180/3.14
wheel_diameter = 4.32
cm_per_degree = 3.14/180*wheel_diameter/2
busytime = 0.005

# Control gains
gain_angle = 70
gain_rate = 1
gain_distance = 14
gain_speed = 2.7

# Initial conditions
angle = 0

# Main loop
while True:
    # Loop start
    start = time()

    # Body angle
    rate = highpass.filter(gyro.rate)
    angle = integrator.integral(rate)

    # Wheel translation
    average_motor_degrees = (right.position + left.position)/2
    distance = average_motor_degrees * cm_per_degree
    speed = differentiator.derivative(distance)

    # Control signal
    duty = (gain_angle*angle +
            gain_rate*rate +
            gain_distance*distance +
            gain_speed*speed)*battery_scaling

    # Motor actuation
    left.duty(duty)
    right.duty(duty)

    # Sleep until loop complete
    while time()-start < looptime:
        sleep(0.002)
