'''
path_generator.py

Functions to generate 2D reference paths as arrays of (x, y) waypoints.
'''

# Import required libraries
import numpy as np

def generate_straight_line(length: float, num_points: int) -> np.ndarray:
    '''
    Generates a straight horizontal path along the x-axis.

    Args:
        length:     Total length of the path [m].
        num_points: Number of waypoints.

    Returns:
        Path as array of shape (num_points, 2).
    '''
    x = np.linspace(0, length, num_points)
    y = np.zeros(num_points)
    return np.column_stack((x, y))

def generate_circle(radius: float, num_points: int) -> np.ndarray:
    '''
    Generates a circular path centered at the origin.

    Args:
        radius:     Circle radius [m].
        num_points: Number of waypoints.

    Returns:
        Path as array of shape (num_points, 2).
    '''
    theta = np.linspace(0, 2 * np.pi, num_points)
    x     = radius * np.cos(theta)
    y     = radius * np.sin(theta)
    return np.column_stack((x, y))