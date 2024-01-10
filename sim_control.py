from xbox_threaded import *
import scipy.constants
import pybullet as p
import numpy as np
import threading
import time
import math

# Paths to the car and track models
TRACK_MODEL_PATH = "resources/meshes/barca_track.sdf"
CAR_MODEL_PATH = "resources/racecar_differential.urdf"
USE_REAL_TIME_SIM = False

# Joint indices for the car
WHEEL_JOINTS = [8,15]
STEERING_JOINTS = [0,2]
HOKUYO_JOINT = 4
ZED_CAMERA_JOINT = 5

# Gear data for the car
GEAR_DATA = [
    (9, 11, 1),
    (10, 13, -1),
    (9, 13, -1),
    (16, 18, 1),
    (16, 19, -1),
    (17, 19, -1),
    (1, 18, -1, 15),
    (3, 19, -1, 15)
]

# Setup pybullet and load the car and track
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -scipy.constants.g)

# Set to 240 Hz
p.setTimeStep(1./240.)

# So we can explicitly step through sim, see PyBullet docs (pg. 33)
p.setRealTimeSimulation(0)

# Load track and car
track = p.loadSDF(TRACK_MODEL_PATH, globalScaling=1)
car = p.loadURDF(CAR_MODEL_PATH, [0,0,.3])

# Initialize joints velocities to 0, and set force applied to 0
for joint in range(p.getNumJoints(car)):
	p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

# Create gear constraints and set parameters
for gear in GEAR_DATA:
    parent, child, gear_ratio = gear[:3]
    c = p.createConstraint(car, parent, car, child, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                           parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
    if len(gear) == 4:
        aux_link = gear[3]
        p.changeConstraint(c, gearRatio=gear_ratio, gearAuxLink=aux_link, maxForce=10000)
    else:
        p.changeConstraint(c, gearRatio=gear_ratio, maxForce=10000)

shared_vars = {'target_velocity': 0, 'steering_angle': 0}
shared_vars_lock = threading.Lock()

MAX_FORCE = 30
def control_car(hz=100., last_time=0., shared_vars=None, shared_vars_lock=None):
    while True:
        now_control_time = time.time()
        if now_control_time - last_time > 1./hz:
            with shared_vars_lock:
                target_velocity = shared_vars['target_velocity']
                steering_angle = shared_vars['steering_angle']

            # Set target_velocity and steering_angle for control
            for wheel in WHEEL_JOINTS:
                p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=MAX_FORCE)

            for steer in STEERING_JOINTS:
                p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=-steering_angle)

            p.stepSimulation()
            last_time = now_control_time
        time.sleep(0.01)  # Sleep to control the update rate

# Create and start threads for each task
control_hz = 240.
control_thread = threading.Thread(target=control_car, args=(control_hz, time.time(), shared_vars, shared_vars_lock))
control_thread.start()

gamepad_monitor = GamepadMonitor(shared_vars, shared_vars_lock)

# rhread to monitor the buttons event
buttons_process = threading.Thread(target=gamepad_monitor.monitor_buttons_state)
buttons_process.daemon = True
buttons_process.start()

# Thread to monitor the bumper event
bumper_thread = threading.Thread(target=gamepad_monitor.monitor_bumper_state)
bumper_thread.daemon = True
bumper_thread.start()

while True:
    pass
