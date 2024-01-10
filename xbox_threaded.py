import sys
import signal
import threading
from inputs import get_gamepad

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

MAX_VELOCITY = 50.

# A class to encapsulate gamepad input monitoring and state management
class GamepadMonitor:
    def __init__(self, shared_vars, shared_vars_lock):
        self.shared_vars = shared_vars
        self.shared_vars_lock = shared_vars_lock

        self.shared_vars['target_velocity'] = 0
        self.shared_vars['steering_angle'] = 0

    def monitor_buttons_state(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type == X_BUTTON_EVENT and event.code == X_BUTTON_EVENT_CODE:
                    if event.state == 1:
                        with self.shared_vars_lock:
                            self.shared_vars['target_velocity'] = MAX_VELOCITY
                    elif event.state == 0 and self.shared_vars['target_velocity'] == MAX_VELOCITY:
                        with self.shared_vars_lock:
                            self.shared_vars['target_velocity'] = 0

                if event.ev_type == B_BUTTON_EVENT and event.code == B_BUTTON_EVENT_CODE:
                    if event.state == 1:
                        with self.shared_vars_lock:
                            self.shared_vars['target_velocity'] = -MAX_VELOCITY
                    elif event.state == 0 and self.shared_vars['target_velocity'] == -MAX_VELOCITY:
                        with self.shared_vars_lock:
                            self.shared_vars['target_velocity'] = 0

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
                    with self.shared_vars_lock:
                        self.shared_vars['steering_angle'] = self.bumper_callback(event)
