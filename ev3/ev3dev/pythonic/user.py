"""High-level user-specified control of the segway."""

from ev3devlight.sensors import Remote


def user_init():
    """User commands to run before the balance loop begins.

    This function is called at the start of the segway program
    before the balancing loop begins. You can use it to initialize
    sensors or configure any necessary settings or startup routines.

    In this example we initialize the IR sensor in remote control
    mode, and define a set of actions to choose from during the
    balancing loop.

    Any user_data that you generate here will be passed to
    the user_control function during the loop. In this case, we
    pass on the remote control object and the dictionary of actions.
    """
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

    # Initialize IR sensor as remote control
    remote = Remote('in4')

    # Collect the data that we'd later on like to pass to
    # the user_control function during the balance loop.
    user_data = (remote, actions)

    # Return the user data
    return user_data


def user_control(user_data):
    """Get user defined desired speed and turnrate.

    This function is called once during each loop of the
    segway balancing algorithm. It should return a tuple
    of two numbers:

    (reference_speed, turn_rate)

    These will be used to adjust the balancing algorithm 
    to make the robot drive forward, backward, and turn.

    The reference_speed is the desired speed of the segway
    in centimeters per second. The turn_rate is the amount
    of duty cycle percentage that we wish to add to one wheel
    and subtract from the other wheel to balance.    

    Make sure that any code within this function takes less
    than 10 ms or so. For comparison: the rest of the loop 
    takes about 5ms, most of which is spent reading the
    sensors. So, you can do a fair amount of calculations
    without issues.
    """
    # Unpack user data
    remote, actions = user_data

    # Read IR remote button
    button = remote.button

    # Return the speed and turnrate for this button
    return actions[button]
