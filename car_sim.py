# TODO: 
# - Provide user with transformation T from base link to all sensors on the car.
#   This page covers the theory related to this in great detail: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
# - Just like the controller where we passed around a shared_vars dictionary, we should do the same here for everything
#   else that the robot needs to share: it's odom, sensor readings (z), control inputs (u), etc. since these are
#   all needed for most of the algorithms we'll be implementing.
import math
import time
import numpy as np
import threading
import pybullet as p
import scipy.constants
from base_controller import start_controller
from xbox_controller import XboxController

class CarSimulation:
    def __init__(self):
        self.setup_simulation()
        self.pose = np.array([0.0, 0.0, 0.0])

    def setup_simulation(self):
        # PyBullet setup code here
        TRACK_MODEL_PATH = "resources/meshes/barca_track.sdf"
        CAR_MODEL_PATH = "resources/racecar_differential.urdf"
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

        p.connect(p.GUI)
        p.resetSimulation()
        p.setGravity(0, 0, -scipy.constants.g)
        p.setTimeStep(1. / 240.)
        p.setRealTimeSimulation(0)

        self.track = p.loadSDF(TRACK_MODEL_PATH, globalScaling=1)
        self.car = p.loadURDF(CAR_MODEL_PATH, [0, 0, .3])

        for joint in range(p.getNumJoints(self.car)):
            p.setJointMotorControl2(self.car, joint, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

        for gear in GEAR_DATA:
            parent, child, gear_ratio = gear[:3]
            c = p.createConstraint(self.car, parent, self.car, child, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                                   parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            if len(gear) == 4:
                aux_link = gear[3]
                p.changeConstraint(c, gearRatio=gear_ratio, gearAuxLink=aux_link, maxForce=10000)
            else:
                p.changeConstraint(c, gearRatio=gear_ratio, maxForce=10000)

        self.shared_vars = {'target_velocity': 0, 'steering_angle': 0}
        self.shared_vars_lock = threading.Lock()

    def compute_odometry(self, left_wheel_velocity, right_wheel_velocity, time_step):
        """
        This functions only simulate odometry.
        """
        # Distance between the wheels (wheelbase) and wheel radius
        wheelbase = 0.2  # Placeholder, set this to your robot's wheelbase
        wheel_radius = 0.05  # Placeholder, set this to your robot's wheel radius
        
        # Compute the robot's linear and angular velocities
        v_l = left_wheel_velocity * wheel_radius
        v_r = right_wheel_velocity * wheel_radius
        v = (v_r + v_l) / 2
        omega = (v_r - v_l) / wheelbase
        
        # Compute the change in pose
        delta_theta = omega * time_step
        delta_x = v * math.cos(self.pose[2]) * time_step
        delta_y = v * math.sin(self.pose[2]) * time_step
        
        # Update the robot's pose
        self.pose[2] += delta_theta
        self.pose[0] += delta_x
        self.pose[1] += delta_y
        
        # Optional: Add noise to the odometry
        noise_scale = 0.02  # Placeholder, set this to the level of noise you want
        self.pose += np.random.normal(0, noise_scale, size=self.pose.shape)
        
        # Keep theta within -pi to pi
        self.pose[2] = (self.pose[2] + np.pi) % (2 * np.pi) - np.pi

    def control_car(self):
        MAX_FORCE = 20
        while True:
            with self.shared_vars_lock:
                target_velocity = self.shared_vars['target_velocity']
                steering_angle = self.shared_vars['steering_angle']

            for wheel in [8, 15]: # 8 is left wheel, 15 is right wheel
                p.setJointMotorControl2(self.car, wheel, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=MAX_FORCE)

            for steer in [0, 2]: # 0 is left steer, 2 is right steer
                p.setJointMotorControl2(self.car, steer, p.POSITION_CONTROL, targetPosition=-steering_angle)

            # Assuming 8 and 15 are the wheel joint indices
            left_wheel_velocity = p.getJointState(self.car, 8)[1]  # Get velocity of the left wheel
            right_wheel_velocity = p.getJointState(self.car, 15)[1]  # Get velocity of the right wheel

            self.compute_odometry(left_wheel_velocity, right_wheel_velocity, 1. / 240.)
            print(f'Odom: x={self.pose[0]:.3f}, y={self.pose[1]:.3f}, theta={self.pose[2]:.3f}')

            p.stepSimulation()
            time.sleep(0.00001)

    def run_simulation(self):
        control_thread = threading.Thread(target=self.control_car)
        control_thread.start()

        start_controller(XboxController, self.shared_vars, self.shared_vars_lock)

        while True:
            pass

if __name__ == "__main__":
    simulation = CarSimulation()
    simulation.run_simulation()
