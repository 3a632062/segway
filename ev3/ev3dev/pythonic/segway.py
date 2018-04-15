#!/usr/bin/env micropython
"""Segway program."""

# Imports
from ev3devlight.sensors import Gyro, Remote
from ev3devlight.motors import Motor
from ev3devlight.brick import Battery
from filters import Highpass, Differentiator, Integrator
from control import LoopTimer

# Configure motors
left_motor = Motor('outD')
left_motor.activate_duty_mode()
right_motor = Motor('outA')
right_motor.activate_duty_mode()

# Battery
battery = Battery()
nominal = 8
battery_scaling = nominal / battery.voltage

# Configure and calibrate LEGO Gyro
gyro = Gyro('in2', read_rate=True, read_angle=False, calibrate=False)

# Configure remote control
remote = Remote('in4')

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


def user_control():
    """Get user defined desired speed and turnrate."""
    # User drive speeds (centimeter per sec)
    forward, stationary, backward = 5, 0, -5

    # Steering values (duty percentage)
    left, straight, right = 15, 0, -15

    # Controls for each remote button press
    actions = {
        'NONE': (stationary, straight),
        'LEFT_UP': (forward/2, right),
        'LEFT_DOWN': (backward/2, left),
        'RIGHT_UP': (forward/2, left),
        'RIGHT_DOWN': (backward/2, right),
        'BOTH_UP': (forward, straight),
        'LEFT_UP_RIGHT_DOWN': (stationary, right),
        'LEFT_DOWN_RIGHT_UP': (stationary, left),
        'BOTH_DOWN': (backward, straight),
        'BEACON': (stationary, straight),
        'BOTH_LEFT': (stationary, straight),
        'BOTH_RIGHT': (stationary, straight)
    }

    # return (speed, turn_rate) associated with button combination
    return actions[remote.button]


# Main loop
while True:
    # Loop start
    timer.loop_start()

    # User control
    reference_speed, turn_rate = user_control()

    # Body angle
    rate = rate_highpass.filter(gyro.rate)
    angle = angle_highpass.filter(rate_integrator.integral(rate))
    angle_sum = angle_integrator.integral(angle)

    # Wheel translation
    average_motor_degrees = (right_motor.position + left_motor.position)/2
    distance = average_motor_degrees * cm_per_degree
    speed = distance_differentiator.derivative(distance)

    # Position tracking
    reference_distance = reference_speed_integrator.integral(reference_speed)
    distance_error = distance - reference_distance
    distance_error_sum = distance_error_integrator.integral(distance_error)

    # Control signal
    duty = battery_scaling*(
        gain_rate*rate +
        gain_angle*angle +
        gain_angle_sum*angle_sum +
        gain_speed*speed +
        gain_distance_error*distance_error +
        gain_distance_error_sum*distance_error_sum
    )

    # Motor actuation
    left_motor.duty(duty-turn_rate)
    right_motor.duty(duty+turn_rate)

    # Sleep until loop complete
    timer.wait_for_completion()
