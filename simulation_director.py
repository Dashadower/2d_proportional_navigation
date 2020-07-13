from objects import MovingObject
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
    def __init__(self, objects: list):
        self.objects = objects

    def tick(self):
        for mobject in self.objects:
            mobject.tick()