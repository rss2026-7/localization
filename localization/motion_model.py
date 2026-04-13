
import numpy as np
class MotionModel:

    def __init__(self, node):
        self.node = node

        # Motion noise parameters (proportional to motion magnitude)
        # Fraction of translational displacement added as translational noise
        self.alpha_trans = 0.15
        # Fraction of translational displacement added as rotational noise
        self.alpha_rot_from_trans = 0.05
        # Fraction of rotational displacement added as rotational noise
        self.alpha_rot = 0.15
        # Minimum noise floor so particles stay diverse even when stationary
        self.min_trans = 0.005   # meters
        self.min_rot   = 0.005   # radians

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
            # Scale noise proportionally to the magnitude of the motion so that
            # a stationary car only gets the small floor noise instead of the
            # large fixed sigma that was previously causing particles to scatter.
            trans_mag = np.sqrt(dx**2 + dy**2)
            rot_mag   = abs(dtheta)

            sigma_xy    = self.alpha_trans * trans_mag + self.min_trans
            sigma_theta = (self.alpha_rot * rot_mag
                           + self.alpha_rot_from_trans * trans_mag
                           + self.min_rot)

            noisy_dx     = dx     + np.random.normal(0.0, sigma_xy,    n)
            noisy_dy     = dy     + np.random.normal(0.0, sigma_xy,    n)
            noisy_dtheta = dtheta + np.random.normal(0.0, sigma_theta, n)

            # Convert body-frame motion into world-frame motion
            particles[:, 0] += np.cos(theta) * noisy_dx - np.sin(theta) * noisy_dy
            particles[:, 1] += np.sin(theta) * noisy_dx + np.cos(theta) * noisy_dy
            particles[:, 2] = (theta + noisy_dtheta + np.pi) % (2 * np.pi) - np.pi

        return particles

        ####################################
