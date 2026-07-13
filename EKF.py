import numpy as np


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ExtendedKalmanFilter:
    def __init__(
        self,
        initial_state,
        initial_cov,
        process_noise,
        measurement_noise,
    ):
        self.x = np.asarray(initial_state, dtype=float).copy()      # (7,)
        self.p = np.asarray(initial_cov, dtype=float).copy()        # (7,7)
        self.q = np.asarray(process_noise, dtype=float).copy()      # (7,7)
        self.r = np.asarray(measurement_noise, dtype=float).copy()  # (3,3)

    def predict(self, dt):
        f_jac = self.jacobian_f(dt)
        self.x = self.f(dt)
        self.p = f_jac @ self.p @ f_jac.T + self.q

    def update(self, z):
        z = np.asarray(z, dtype=float)

        h_jac = self.jacobian_h()

        innovation = z - self.h()
        innovation[2] = normalize_angle(innovation[2])

        s = h_jac @ self.p @ h_jac.T + self.r

        try:
            s_inv = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            s_inv = np.eye(3)

        k = self.p @ h_jac.T @ s_inv

        self.x = self.x + k @ innovation
        self.p = (np.eye(7) - k @ h_jac) @ self.p

        return innovation

    def filter_pose(self, x_meas, y_meas, theta_meas, dt):
        """
        Combined predict + update.

        Returns:
            (
                x,
                y,
                theta,
                vx,
                vy,
                omega,
                innovation,
                p_trace,
                q_trace,
                r_trace,
            )
        """
        self.predict(dt)

        z = np.array([x_meas, y_meas, theta_meas], dtype=float)
        innovation = self.update(z)

        x_f = self.x[0]
        y_f = self.x[1]
        theta_f = np.arctan2(self.x[2], self.x[3])
        vx = self.x[4]
        vy = self.x[5]
        omega = self.x[6]

        p_trace = np.trace(self.p)
        q_trace = np.trace(self.q)
        r_trace = np.trace(self.r)

        return (
            x_f,
            y_f,
            theta_f,
            vx,
            vy,
            omega,
            innovation,
            p_trace,
            q_trace,
            r_trace,
        )

    # ------------------------------------------------------------------
    # State transition
    # State = [x, y, sin(theta), cos(theta), vx, vy, omega]
    # ------------------------------------------------------------------

    def f(self, dt):
        sin_theta = self.x[2]
        cos_theta = self.x[3]
        vx = self.x[4]
        vy = self.x[5]
        omega = self.x[6]

        theta = np.arctan2(sin_theta, cos_theta)
        theta_new = theta + omega * dt

        new_x = self.x.copy()
        new_x[0] += vx * dt
        new_x[1] += vy * dt
        new_x[2] = np.sin(theta_new)
        new_x[3] = np.cos(theta_new)

        return new_x

    def h(self):
        theta = np.arctan2(self.x[2], self.x[3])
        return np.array([
            self.x[0],
            self.x[1],
            theta,
        ])

    def jacobian_f(self, dt):
        f = np.eye(7)

        f[0, 4] = dt
        f[1, 5] = dt

        sin_theta = self.x[2]
        cos_theta = self.x[3]
        omega = self.x[6]

        theta = np.arctan2(sin_theta, cos_theta)
        theta_new = theta + omega * dt

        f[2, 6] = dt * np.cos(theta_new)
        f[3, 6] = -dt * np.sin(theta_new)

        return f

    def jacobian_h(self):
        h = np.zeros((3, 7))

        h[0, 0] = 1.0
        h[1, 1] = 1.0

        sin_theta = self.x[2]
        cos_theta = self.x[3]

        denom = sin_theta**2 + cos_theta**2

        if abs(denom) > 1e-12:
            h[2, 2] = cos_theta / denom
            h[2, 3] = -sin_theta / denom

        return h

    def set_noise_parameters(self, p_noise_p, p_noise_v, m_noise):
        # Position process noise
        self.q[0, 0] = p_noise_p
        self.q[1, 1] = p_noise_p

        # Velocity process noise
        self.q[4, 4] = p_noise_v
        self.q[5, 5] = p_noise_v

        # Measurement noise
        self.r[0, 0] = m_noise
        self.r[1, 1] = m_noise
        self.r[2, 2] = m_noise