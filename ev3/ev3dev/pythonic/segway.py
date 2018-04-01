#!/usr/bin/env micropython
"""Segway program."""

from time import sleep
from ev3devlight.sensors import Gyro
from ev3devlight.motors import Motor
from ev3devlight.brick import print_vscode


class Filter():
    """Highpass filter that removes near-constant bias."""

    def __init__(self, initial_bias, forget_rate, sample_time):
        """Initialize filter."""
        self.bias = initial_bias
        # Approximate forget factor per sample
        self.alpha = forget_rate*sample_time

    def filter(self, sample):
        """Add new sample and give filtered output."""
        self.bias = (1-self.alpha)*self.bias + self.alpha*sample
        return sample - self.bias


class Differentiator():
    """Obtain time derivative of sampled signal."""

    # Example for window of 3. Consider these 3 saved samples and a new sample:
    #
    #    o                          ---|
    #                                  |
    #           o                      |   Derivative = (new-oldest)/(3*dt)
    #                  o               |
    #                         o     ---|
    #
    #    |------|------|------|
    #       dt     dt     dt
    #
    #    0      1      2
    #
    # (oldest)              (new)

    def __init__(self, window, sample_time):
        """Initialize differentiator."""
        self.window = window
        self.dt = sample_time
        self.samples = [0 for i in range(self.window)]

    def derivative(self, sample):
        """Store sample and return approximated derivative."""
        # Read and remove oldest sample
        oldest = self.samples.pop(0)
        # Add new sample
        self.samples.append(sample)
        # Return approximated derivative
        return (sample - oldest)/(self.window*self.dt)


# Configure hardware
gyro = Gyro('in2', read_rate=True, read_angle=False, calibrate=True)
motor = Motor('outD')

# Configure filters
looptime = 0.1
highpass = Filter(0, 0.001, looptime)
diff = Differentiator(3, looptime)

# Test filters
for i in range(int(10/looptime)):
    rate = highpass.filter(gyro.rate)
    speed = diff.derivative(motor.position)
    print_vscode(speed)
    sleep(looptime)
