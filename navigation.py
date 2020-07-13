from objects import MovingObject, PursuitObject
import numpy as np
from numpy.linalg import norm
# All these functions return the rotation degree(or rads) per tick for the pursuit object

def pure_pursuit(pursuit_obj: PursuitObject, target_obj: MovingObject):
    """
    Pure-Pursuit Homing
    A_cmd = LOS
    Missile velocity vector points at the target alongside LOS
    """

    los_vector = (target_obj.location_arr - pursuit_obj.location_arr)
    los_angle = pursuit_obj.speed_vector.anglebetween(los_vector)  # los angle
    return los_angle

def simple_proportional_navigation(pursuit_obj: PursuitObject, target_obj: MovingObject, N=3):
    """
    Simple Proportional Navigation

    A_cmd = N * theta_LOS
    N: Navigation Gain (proportional constant normally 3~5)
    theta_LOS: LOS Rotation Rate

    Missile velocity vector rotation matches LOS rate
    Acceleration is commanded normal to missile velocity vector
    """
    a_cmd_rad = N * pursuit_obj.calculate_los_rate(target_obj)
    return np.rad2deg(a_cmd_rad)

def true_proportional_navigation(pursuit_obj: PursuitObject, target_obj: MovingObject, N=3):
    """
    True Proportional Navigation

    A_cmd = N * V_c * theta_LOS
    N: Navigation Game
    V_c: Range Closing Rate
    theta_LOS: LOS Rotation Rate

    """

    a_cmd_rad = N * pursuit_obj.calculate_closing_speed(target_obj) * pursuit_obj.calculate_los_rate(target_obj)
    return np.rad2deg(a_cmd_rad)

def augumented_proportional_navigation(pursuit_obj: PursuitObject, target_obj: MovingObject, N=3):
    """
    Augumented Proportional Navigation

    A_cmd = N * V_c * theta_LOS + (N * n_T)/2
    N: Navigation Game
    V_c: Range Closing Rate
    theta_LOS: LOS Rotation Rate
    n_T: Target Acceleration rate normal to LOS
    """
    dt = 1  # los difference time (1 tick)
    old_los = pursuit_obj._anglebetweenarr(pursuit_obj.previous_pursuit_location_arr, pursuit_obj.previous_target_location_arr,degrees=False)
    old_normal_velocity = np.dot(pursuit_obj.previous_target_velocity_arr, old_los / norm(old_los))
    new_los = pursuit_obj._anglebetweenarr(pursuit_obj.location_arr, target_obj.location_arr, degrees=False)
    new_normal_velocity = np.dot(target_obj.speed_vector, new_los / norm(new_los))
    normal_acceleration_rate = (new_normal_velocity - old_normal_velocity) / dt
    a_cmd_rad = N * pursuit_obj.calculate_closing_speed(target_obj) * pursuit_obj.calculate_los_rate(target_obj) + (N * normal_acceleration_rate) / 2

    return np.rad2deg(a_cmd_rad)
