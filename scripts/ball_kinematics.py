import numpy as np

class Ball_Kinematics:
    # Assuming no bounces and only gravity

    def __init__(self, init_pos, init_vel):
        self.p_i = np.array(init_pos).reshape((3, 1))
        self.v_i = np.array(init_vel).reshape((3, 1))
        self.a = np.array([0, 0, -9.81]).reshape((3, 1))

    def compute_d(self, t):
        return self.v_i * (t) + 0.5 * self.a * (t)

    def compute_pos(self, t):
        return self.p_i + self.compute_d(t)

    def compute_time_intersect_x(self):
        return self.p_i[1, 0] / (-self.v_i[1, 0])
