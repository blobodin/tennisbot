class Ball_Kinematics:
    # Assuming no bounces and only gravity

    def __init__(self, init_pos, init_vel):
        self.p_i = np.array(init_pos)
        self.v_i = np.array(init_vel)
        self.a = np.array([0, 0, -9.81])

    def compute_pos(self, t):
        d = self.v_i * (t) + 0.5 * self.a * (t)
        return self.p_i + d

    def compute_time_intersect_x(self):
        return self.p_i[0] / (-self.v_i[0])
