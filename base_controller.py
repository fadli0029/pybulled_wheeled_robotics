import threading

class BaseController:
    def __init__(self, shared_vars, shared_vars_lock):
        self.shared_vars = shared_vars
        self.shared_vars_lock = shared_vars_lock

    def get_target_velocity(self):
        with self.shared_vars_lock:
            return self.shared_vars.get('target_velocity', 0)

    def set_target_velocity(self, velocity):
        with self.shared_vars_lock:
            self.shared_vars['target_velocity'] = velocity

    def get_steering_angle(self):
        with self.shared_vars_lock:
            return self.shared_vars.get('steering_angle', 0)

    def set_steering_angle(self, angle):
        with self.shared_vars_lock:
            self.shared_vars['steering_angle'] = angle

    def update(self):
        # Override this method in child classes to implement specific control logic
        pass

def start_controller(controller, shared_vars, shared_vars_lock):
    controller_instance = controller(shared_vars, shared_vars_lock)
    controller_thread = threading.Thread(target=controller_instance.update)
    controller_thread.daemon = True
    controller_thread.start()
