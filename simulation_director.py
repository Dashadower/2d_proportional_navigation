from objects import MovingObject, PursuitObject
import numpy as np


# simulation loop
# 1. call tick() (moves all objects based on respective velocity vectors)
# 2. Update screen if using UI
# current state is reference state for upcoming velocity vector adjustments
# 3. call navigation function to adjust PursuitObject's velocity vector
# 4. Receive velocity vector adjustments for targetobject
# 5. Call PursuitObject's update_data, which updates previous data
# 6. Goto 1, actuating adjustments made in steps 3 and 4

class SimulationDirector:
    def __init__(self, prey_object, pursuit_object: PursuitObject):
        self.prey_object = prey_object
        self.pursuit_object = pursuit_object

    def tick(self):
        self.prey_object.tick()

        get_user_input()

        self.pursuit_object.calculate_solution()
        self.pursuit_object.update_target_data()