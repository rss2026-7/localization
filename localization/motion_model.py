
import numpy as np
class MotionModel:

    def __init__(self, node):
        self.node = node

        # Proportional noise: sigma scales with the magnitude of each motion component.
        # alpha_trans: fraction of translational displacement added as x/y noise.
        # alpha_rot: fraction of rotational displacement added as heading noise.
        # alpha_rot_from_trans: heading noise that grows with translation (wheel slip).
        # min_*: noise floors so particles still spread when the robot is nearly still.
        self.alpha_trans = 0.15
        self.alpha_rot = 0.15
        self.alpha_rot_from_trans = 0.05
        self.min_trans_noise = 0.005  # meters
        self.min_rot_noise = 0.005    # radians

        # Deterministic mode: when True, no noise is added (for unit tests)
        node.declare_parameter('deterministic', False)
        self.deterministic = node.get_parameter('deterministic').get_parameter_value().bool_value

        ####################################

    def evaluate(self, particles, odometry):
        dx, dy, dtheta = odometry
        n = particles.shape[0]

        # Current heading of each particle
        theta = particles[:, 2].copy()

        if self.deterministic:
            # No noise — used for unit tests and deterministic simulation
            particles[:, 0] += np.cos(theta) * dx - np.sin(theta) * dy
            particles[:, 1] += np.sin(theta) * dx + np.cos(theta) * dy
            particles[:, 2] = (theta + dtheta + np.pi) % (2 * np.pi) - np.pi
        else:
            # Proportional noise: scales with how much the robot actually moved.
            trans = np.sqrt(dx**2 + dy**2)
            sigma_x     = self.alpha_trans * abs(dx)     + self.min_trans_noise
            sigma_y     = self.alpha_trans * abs(dy)     + self.min_trans_noise
            sigma_theta = (self.alpha_rot * abs(dtheta)
                           + self.alpha_rot_from_trans * trans
                           + self.min_rot_noise)

            noisy_dx = dx + np.random.normal(0.0, sigma_x, n)
            noisy_dy = dy + np.random.normal(0.0, sigma_y, n)
            noisy_dtheta = dtheta + np.random.normal(0.0, sigma_theta, n)

            # Convert body-frame motion into world-frame motion
            particles[:, 0] += np.cos(theta) * noisy_dx - np.sin(theta) * noisy_dy
            particles[:, 1] += np.sin(theta) * noisy_dx + np.cos(theta) * noisy_dy
            particles[:, 2] = (theta + noisy_dtheta + np.pi) % (2 * np.pi) - np.pi

        return particles

        ####################################
