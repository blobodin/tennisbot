import numpy as np

class Swing_Helper:
    def __init__(self, fore_back_point, ready_point, hit_point):
        # Define quantities for plane intersecting all three points
        vec1 = hit_point - fore_back_point
        vec2 = hit_point - ready_point
        self.norm_vec = np.cross(np.ndarray.flatten(vec1), np.ndarray.flatten(vec2)).reshape((3, 1))
        self.ref_point = hit_point

        # Define quantities for 2d parabolas on xy plane of form ax^2 + c = 0

        # Parabola for fore/back point to hit point
        self.a1 = (fore_back_point[0, 0] - hit_point[0, 0]) / (fore_back_point[1, 0]**2 - hit_point[1, 0]**2)
        self.c1 = hit_point[0, 0] - self.a1 * hit_point[1, 0]**2

        # Parabola for hit point to ready point
        self.a2 = (ready_point[0, 0] - hit_point[0, 0]) / (ready_point[1, 0]**2 - hit_point[1, 0]**2)
        self.c2 = hit_point[0, 0] - self.a2 * hit_point[1, 0]**2

    def project_onto_plane(self, pxy):
        numerator = -1 * (self.norm_vec[0, 0] * (pxy[0, 0] - self.ref_point[0, 0])
            + self.norm_vec[1, 0] * (pxy[1, 0] - self.ref_point[1, 0]))

        z = (numerator / self.norm_vec[2, 0]) + self.ref_point[2, 0]
        return np.array([pxy[0, 0], pxy[1, 0], z]).reshape((3, 1))

    def compute_2d_parabola(self, y, type):
        # Use first parabola if asked, otherwise use second parabola
        if type == "fore/back->hit":
            x = self.a1 * y**2 + self.c1
            return np.array([x, y]).reshape((2, 1))
        else:
            x = self.a2 * y**2 + self.c2
            return np.array([x, y]).reshape((2, 1))
