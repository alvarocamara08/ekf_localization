
'''
ekf.py

Extended Kalman Filter for 2D vehicle localization.
Fuses odometry-based motion prediction with GPS measurements
to estimate vehicle pose (x, y, theta).
'''

# Import required libraries
import numpy as np


class EKF:
    '''
    Extended Kalman Filter for a kinematic bicycle model.

    Predicts vehicle pose using a motion model and corrects
    it with GPS observations using the EKF update equations.
    '''

    def __init__(self, x: np.ndarray, P: np.ndarray,
                 Q: np.ndarray, R: np.ndarray, wheelbase: float):
        self.x         = x          # estimated state [x, y, theta] (3,)
        self.P         = P          # state covariance matrix (3x3)
        self.Q         = Q          # process noise covariance (3x3)
        self.R         = R          # measurement noise covariance (2x2)
        self.wheelbase = wheelbase  # distance between axles [m]

    def predict(self, u: np.ndarray, dt: float) -> None:
        '''
        Propagates state and covariance using the bicycle motion model.

        Args:
            u:  Control input [v, delta] — velocity [m/s] and steering angle [rad].
            dt: Timestep [s].
        '''
        x, y, theta = self.x
        v, delta    = u

        # Predicted state
        self.x = np.array([
            x + v * np.cos(theta) * dt,
            y + v * np.sin(theta) * dt,
            theta + (v * np.tan(delta) / self.wheelbase) * dt])

        # Jacobian of motion model wrt state
        F = np.array([[1, 0, -v * np.sin(theta) * dt],
                      [0, 1,  v * np.cos(theta) * dt],
                      [0, 0,  1]])

        # Propagate covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z: np.ndarray) -> None:
        '''
        Corrects state estimate using a GPS measurement.

        Args:
            z: GPS measurement [x, y] in meters.
        '''
        # Observation Jacobian (GPS measures x and y directly)
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])

        # Innovation
        y = z - H @ self.x

        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Correct state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    def get_state(self) -> np.ndarray:
        '''Returns the current estimated state [x, y, theta].'''
        return self.x.copy()