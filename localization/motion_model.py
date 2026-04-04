
import numpy as np
class MotionModel:

    def __init__(self, node):
        self.node = node

        ####################################
        # Read deterministic flag from params
        node.declare_parameter('deterministic', False)
        self.deterministic = node.get_parameter('deterministic').get_parameter_value().bool_value

        # Motion noise parameters (only used when deterministic=False)
        self.sigma_x = 0.02
        self.sigma_y = 0.02
        self.sigma_theta = 0.01
        ####################################

    def evaluate(self, particles, odometry):
        dx, dy, dtheta = odometry
        n = particles.shape[0]

        # When not deterministic, inject per-particle Gaussian noise into the
        # body-frame displacement. As noise -> 0 this converges to the
        # deterministic model, satisfying the lab requirement.
        if not self.deterministic:
            dx = dx + np.random.normal(0.0, self.sigma_x, n)
            dy = dy + np.random.normal(0.0, self.sigma_y, n)
            dtheta = dtheta + np.random.normal(0.0, self.sigma_theta, n)

        # Current heading of each particle
        theta = particles[:, 2].copy()

        # Convert body-frame displacement into world-frame motion
        particles[:, 0] += np.cos(theta) * dx - np.sin(theta) * dy
        particles[:, 1] += np.sin(theta) * dx + np.cos(theta) * dy
        particles[:, 2] = np.mod(theta + dtheta, 2 * np.pi)

        return particles

        ####################################
