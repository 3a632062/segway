"""Filters for constantly sampled data/measurements."""


class Highpass():
    """Highpass filter that removes near-constant bias."""

    def __init__(self, sample_time, initial_bias, forget_rate):
        """Initialize filter."""
        self.bias = initial_bias
        # Approximate forget factor per sample
        self.alpha = forget_rate*sample_time

    def filter(self, sample):
        """Add new sample and give filtered output."""
        self.bias = (1-self.alpha)*self.bias + self.alpha*sample
        return sample - self.bias


class Integrator():
    """Obtain time integral of sampled signal."""

    def __init__(self, sample_time):
        """Initialize at zero."""
        self.state = 0
        self.dt = sample_time

    def integral(self, sample):
        """Integrate using sample and return output."""
        self.state += sample*self.dt
        return self.state


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

    def __init__(self, sample_time, window):
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
