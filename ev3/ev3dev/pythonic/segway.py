#!/usr/bin/env micropython
"""Segway program."""

# Imports
from time import sleep, time
from ev3devlight.sensors import Gyro
from ev3devlight.motors import Motor
from ev3devlight.brick import Battery
from filters import Highpass, Differentiator, Integrator

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

# Configure highpass filters
looptime = 0.02
rate_highpass = Highpass(looptime, 0, 0.02)
angle_highpass = Highpass(looptime, 0, 0.02)

# Angle rate and double integrator
rate_integrator = Integrator(looptime)
angle_integrator = Integrator(looptime)

# Position differentiator and integrator
distance_integrator = Integrator(looptime)
distance_differentiator = Differentiator(looptime, 5)

# Parameters
deg2rad = 180/3.14
wheel_diameter = 4.32
cm_per_degree = 3.14/180*wheel_diameter/2
busytime = 0.005

# Control gains
gain_rate = 1
gain_angle = 70
gain_angle_sum = 2
gain_speed = 4.1
gain_distance = 14
gain_distance_sum = 2

# Main loop
while True:
    # Loop start
    start = time()

    # Body angle
    rate = rate_highpass.filter(gyro.rate)
    angle = angle_highpass.filter(rate_integrator.integral(rate))
    angle_sum = angle_integrator.integral(angle)

    # Wheel translation
    average_motor_degrees = (right.position + left.position)/2
    distance = average_motor_degrees * cm_per_degree
    distance_sum = distance_integrator.integral(distance)
    speed = distance_differentiator.derivative(distance)

    # Control signal
    duty = (gain_rate*rate +
            gain_angle*angle +
            gain_angle_sum*angle_sum +
            gain_speed*speed +
            gain_distance*distance +
            gain_distance_sum*distance_sum)*battery_scaling

    # Motor actuation
    left.duty(duty)
    right.duty(duty)

    # Sleep until loop complete
    while time()-start < looptime:
        sleep(0.002)
