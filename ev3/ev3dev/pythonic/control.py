"""Control Systems."""

# Imports
from time import time, sleep


class LoopTimer():
    """Class for timing loops with desired constant loop times."""

    def __init__(self, loop_time):
        """Initialize loop timer."""
        self.loop_time = loop_time
        self.reset()

    def reset(self):
        """Reset the overal timer."""
        self.creation_time = time()
        self.loops = 0
        self.busy_time = 0
        self.violations = 0

    def loop_start(self):
        """Save time at loop start."""
        self.start = time()
        self.loops += 1

    def wait_for_completion(self):
        """Sleep until loop has completed."""
        # Time spent doing stuff
        time_spent = time()-self.start
        self.busy_time += time_spent

        # Time we have left
        time_left = self.loop_time - time_spent

        # Wait until loop is done
        if time_left > 0:
            sleep(time_left)
        else:
            self.violations += 1

    @property
    def loop_average(self):
        """Return average loop time."""
        assert self.loops > 0
        return (time()-self.creation_time)/self.loops

    @property
    def busy_average(self):
        """Return average time spent waiting."""
        assert self.loops > 0
        return self.busy_time/self.loops
