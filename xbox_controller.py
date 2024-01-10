import threading
from inputs import get_gamepad
from base_controller import BaseController

PRESSED = 1
RELEASED = 0

X_BUTTON_EVENT = 'Key'
X_BUTTON_EVENT_CODE = 'BTN_NORTH'
B_BUTTON_EVENT = 'Key'
B_BUTTON_EVENT_CODE = 'BTN_EAST'
BUMPERS_EVENT = 'Absolute'
RIGHT_BUMPER_EVENT_CODE = 'ABS_RZ'
LEFT_BUMPER_EVENT_CODE = 'ABS_Z'

BUMPERS_MIN = 0
BUMPERS_MAX = 1023

MAX_VELOCITY = 50

class XboxController(BaseController):
    def __init__(self, shared_vars, shared_vars_lock):
        super().__init__(shared_vars, shared_vars_lock)

    def monitor_buttons_state(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type == X_BUTTON_EVENT and event.code == X_BUTTON_EVENT_CODE:
                    if event.state == PRESSED:
                        self.set_target_velocity(MAX_VELOCITY)
                    elif event.state == RELEASED and self.get_target_velocity() == MAX_VELOCITY:
                        self.set_target_velocity(0)

                if event.ev_type == B_BUTTON_EVENT and event.code == B_BUTTON_EVENT_CODE:
                    if event.state == PRESSED:
                        self.set_target_velocity(-MAX_VELOCITY)
                    elif event.state == RELEASED and self.get_target_velocity() == -MAX_VELOCITY:
                        self.set_target_velocity(0)

    def bumper_callback(self, event):
        val = (event.state - BUMPERS_MIN) / (BUMPERS_MAX - BUMPERS_MIN)
        if event.code == RIGHT_BUMPER_EVENT_CODE:
            return val  # turn right
        return -val  # turn left

    def monitor_bumper_state(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type == BUMPERS_EVENT:
                    self.set_steering_angle(self.bumper_callback(event))

    def update(self):
        """
        NOTE to developers:
            Unless you are implementing a controller that requires concurrency
            to retrieve values for steering angle and target velocity, you should
            not create separate threads in update() method, since this is already
            managed by the BaseController class.

            Instead, update() method should be used to implement the control logic
            only by the following steps:
                1. Build whatever logic you need to control the car (i.e: to set the
                   target velocity and steering angle)
                2. The, in this update() method, it should ideally call a method
                   that runs persistently. See the PIDController class for a better example
                   on how to properly extend the BaseController class, since this XboxController
                   class is a special case anyway.
        """
        # Start monitoring buttons and bumpers in separate threads
        button_thread = threading.Thread(target=self.monitor_buttons_state)
        button_thread.daemon = True
        button_thread.start()

        bumper_thread = threading.Thread(target=self.monitor_bumper_state)
        bumper_thread.daemon = True
        bumper_thread.start()
