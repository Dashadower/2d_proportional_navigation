from vector import Vector2D
import numpy as np
from numpy.linalg import norm


class MovingObject:
    def __init__(self, name, start_x: float, start_y: float, vel_x: float, vel_y: float,max_speed: float = 10, turn_rate_deg: float = 1, radius: float = 2):
        self.name = name
        self.turn_rate_deg = turn_rate_deg
        self.radius = radius

        self.location_arr = np.array([start_x, start_y], dtype=np.float64)
        self.speed_vector = Vector2D(vel_x, vel_y)
        self.max_speed = max_speed

    def tick(self):
        self.location_arr += self.speed_vector.array
        return self.location_arr

    def rotate(self, deg=None, rad=None):
        if rad:
            deg = np.rad2deg(rad)
        if abs(deg) > self.turn_rate_deg:  # enforce turn rate
            #print("turn rate exceeded")
            deg = self.turn_rate_deg if deg > 0 else -self.turn_rate_deg
        self.speed_vector.rotate(deg=deg)

    def __repr__(self):
        return f"MovingObject - {self.name}: location: ({self.location_arr[0]}, {self.location_arr[1]}) velocity: ({self.speed_vector.array[0]}, {self.speed_vector.array[1]})"


class PursuitObject(MovingObject):
    def __init__(self, *args, **kwargs):
        self.previous_pursuit_location_arr = None
        self.previous_pursuit_velocity_arr = None
        self.previous_target_location_arr = None
        self.previous_target_velocity_arr = None
        super().__init__(*args, **kwargs)

    def tick(self):
        self.previous_pursuit_location_arr = self.location_arr.copy()
        self.location_arr += self.speed_vector.array
        return self.location_arr

    def calculate_los_rate(self, target_object: MovingObject):
        """
        Calculate LOS rate based on 1 tick DL/dt
        since APN formulas use rads, this function also returns values as rads
        :param target_object:
        :return: LOS rate in radians
        """
        new_target_location_arr = target_object.location_arr
        new_target_velocity_arr = target_object.speed_vector.array
        dt = 1  # los difference time (1 tick)
        old_los = self._anglebetweenarr(self.previous_pursuit_location_arr, self.previous_target_location_arr, degrees=False)
        new_los = self._anglebetweenarr(self.speed_vector.array, new_target_location_arr, degrees=False)
        return (new_los - old_los) / dt

    def calculate_closing_speed(self, target_object: MovingObject):
        """
        Calculate current closing speed for 1 tick. Should be distance_unit/tick
        :param target_object:
        :return:
        """
        pos_diff = self.location_arr - target_object.location_arr
        return -(np.dot((self.speed_vector.array - target_object.speed_vector.array), pos_diff) / norm(pos_diff))

    def _anglebetweenarr(self, arr1: np.ndarray, arr2: np.ndarray, degrees=True):
        rads = np.arccos(np.dot(arr1, arr2) / (norm(arr1) * norm(arr2)))
        return np.rad2deg(rads) if degrees else rads

    def update_target_data(self, target_object: MovingObject):
        self.previous_target_location_arr = target_object.location_arr.copy()
        self.previous_target_velocity_arr = target_object.speed_vector.array.copy()



