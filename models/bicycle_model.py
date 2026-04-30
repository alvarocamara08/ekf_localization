'''
bicycle_model.py

Kinematic bicycle model for a rear-axle referenced Ackermann-steered vehicle.
State: (x, y, theta) — position and heading of the rear axle.
Input: steering angle delta [rad], timestep dt [s].
'''

# Import required libraries
import numpy as np

class BicycleModel:
    '''
    Kinematic bicycle model for an Ackermann-steered vehicle.

    Models the vehicle as a two-wheeled bicycle referenced at the rear axle.
    Assumes constant velocity and small angle approximations do NOT apply —
    full trigonometric equations are used.
    '''

    def __init__(self, x: float, y: float, theta: float, v: float, wheelbase: float):
        self.x         = x          # x position [m]
        self.y         = y          # y position [m]
        self.theta     = theta      # heading angle [rad]
        self.v         = v          # longitudinal velocity [m/s]
        self.wheelbase = wheelbase  # distance between axles [m]

    def update(self, delta: float, dt: float) -> None:
        '''
        Integrates the kinematic equations forward by one timestep.

        Args:
            delta: Steering angle [rad].
            dt:    Timestep duration [s].
        '''
        self.x     += self.v * np.cos(self.theta) * dt
        self.y     += self.v * np.sin(self.theta) * dt
        self.theta += (self.v * np.tan(delta) / self.wheelbase) * dt

    def get_state(self) -> np.ndarray:
        '''Returns the current vehicle state as [x, y, theta].'''
        return np.array([self.x, self.y, self.theta])