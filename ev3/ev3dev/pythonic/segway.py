#!/usr/bin/env micropython
"""Segway program."""

# Imports
from ev3devlight.sensors import Gyro
from ev3devlight.motors import Motor
from ev3devlight.brick import Battery
from filters import Highpass, Differentiator, Integrator
from control import LoopTimer

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
gyro = Gyro('in2', read_rate=True, read_angle=False, calibrate=False)

# Configure highpass filters
loop_time = 0.02
rate_highpass = Highpass(loop_time, 0, 0.02)
angle_highpass = Highpass(loop_time, 0, 0.02)

# Angle rate and double integrator
rate_integrator = Integrator(loop_time)
angle_integrator = Integrator(loop_time)

# Position differentiator and integrator
distance_differentiator = Differentiator(loop_time, 15)
distance_error_integrator = Integrator(loop_time)
reference_speed_integrator = Integrator(loop_time)

# Parameters
deg2rad = 180/3.14
wheel_diameter = 4.32
cm_per_degree = 3.14/180*wheel_diameter/2

# Control gains
gain_rate = 1
gain_angle = 70
gain_angle_sum = 2
gain_speed = 6
gain_distance_error = 16
gain_distance_error_sum = 5

# Loop timer
timer = LoopTimer(loop_time)

# Main loop
while True:
    # Loop start
    timer.loop_start()

    # Body angle
    rate = rate_highpass.filter(gyro.rate)
    angle = angle_highpass.filter(rate_integrator.integral(rate))
    angle_sum = angle_integrator.integral(angle)

    # User Control
    reference_speed = 0
    steer_rate = 10

    # Wheel translation
    average_motor_degrees = (right.position + left.position)/2
    distance = average_motor_degrees * cm_per_degree
    speed_error = distance_differentiator.derivative(distance) - reference_speed

    # Position tracking
    reference_distance = reference_speed_integrator.integral(reference_speed)
    distance_error = distance - reference_distance
    distance_error_sum = distance_error_integrator.integral(distance_error)

    # Control signal
    duty = battery_scaling*(
        gain_rate*rate +
        gain_angle*angle +
        gain_angle_sum*angle_sum +
        gain_speed*speed_error +
        gain_distance_error*distance_error +
        gain_distance_error_sum*distance_error_sum
    )

    # Motor actuation
    left.duty(duty-steer_rate)
    right.duty(duty+steer_rate)

    # Sleep until loop complete
    timer.wait_for_completion()
