# TODO: Make a base class for x_sim.py files where x
# is any wheeled robot, since for a given robot r, they
# different methods are applied to control r and setup r. But,
# ideally, run_simulation() should be the same for all robots.
import time
import threading
import pybullet as p
import scipy.constants
from base_controller import start_controller
from xbox_controller import XboxController

class CarSimulation:
    def __init__(self):
        self.setup_simulation()

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

    def control_car(self):
        MAX_FORCE = 30
        control_hz = 240
        last_time = time.time()

        while True:
            now_control_time = time.time()
            if now_control_time - last_time > 1. / control_hz:
                with self.shared_vars_lock:
                    target_velocity = self.shared_vars['target_velocity']
                    steering_angle = self.shared_vars['steering_angle']

                for wheel in [8, 15]:
                    p.setJointMotorControl2(self.car, wheel, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=MAX_FORCE)

                for steer in [0, 2]:
                    p.setJointMotorControl2(self.car, steer, p.POSITION_CONTROL, targetPosition=-steering_angle)

                p.stepSimulation()
                last_time = now_control_time
            time.sleep(0.01)

    def run_simulation(self):
        control_thread = threading.Thread(target=self.control_car)
        control_thread.start()

        start_controller(XboxController, self.shared_vars, self.shared_vars_lock)

        while True:
            pass

if __name__ == "__main__":
    simulation = CarSimulation()
    simulation.run_simulation()

