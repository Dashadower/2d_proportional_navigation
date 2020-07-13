import numpy as np
from numpy.linalg import norm

class Vector2D:
    def __init__(self, initial_x, initial_y):
        self.array = np.array([initial_x, initial_y])

    def rotate(self, deg=None, rad=None):
        """Rotate array by degrees or rads. positive is counterclockwise"""
        if deg:
            rad = np.deg2rad(deg)

        rotation_matrix = np.array([[np.cos(rad), -np.sin(rad)],
                                    [np.sin(rad), np.cos(rad)]])

        self.array = rotation_matrix @ self.array

    def vector2deg(self):
        unit_vec = self.array / norm(self.array)
        return np.rad2deg(np.arctan2(unit_vec[1], unit_vec[0]))

    def anglebetween(self, vector, degrees=True):
        rads = np.arccos(np.dot(self.array, vector.array) / (norm(self.array) * norm(vector.array)))
        return np.rad2deg(rads) if degrees else rads

    def __add__(self, other):
        if isinstance(other, np.ndarray):
            return self.array + other
        elif isinstance(other, Vector2D):
            add_val = self.array + other.array
            return Vector2D(add_val[0], add_val[1])
        else:
            raise ValueError(f"Addition only supported with np.ndarray or Vector2D types (received type {type(other)})")

    def __radd__(self, other):
        if isinstance(other, int):
            raise ValueError(f"Received type {type(other)} for rhs addition. Perhaps you are attempting rhs addition with np.ndarray? Use lhs instead.")
        return self.__add__(other)

    def __repr__(self):
        return f"Vector2D - {self.array}"

    def __str__(self):
        return self.__repr__()

if __name__ == '__main__':
    g = Vector2D(1, 0)
    g2 = Vector2D(4, 5)
    arr = np.array([2, 3])
    #print(g + g2.array)
    #g += g2
    #g.rotate(90)
    print(g, g.vector2deg())
    print(g2, g2.vector2deg())
    print(g.anglebetween(g2))
