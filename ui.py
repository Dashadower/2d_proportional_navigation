import tkinter as tk
from objects import MovingObject, PursuitObject
from typing import Union, Tuple, List
import numpy as np


class SimulationCanvas(tk.Canvas):
    def __init__(self, master: Union[tk.Tk, tk.Frame], *args, **kwargs):
        super().__init__(master=master, *args, **kwargs)
        self.origin: Tuple[int, int] = (0, self.winfo_height())
        self.autorun = tk.BooleanVar(self, value=False)
        self.pursuit_obj_color = "#FF0000"
        self.prey_obj_color = "#00FF00"
        self.pursuit_obj_width = 10
        self.pursuit_obj_length = 20
        self.prey_obj_width = 10
        self.prey_obj_length = 20
        self.speed_vector_draw_scale = 20
        self.focus_set()

    def cartesian2screen(self, x: Union[int, np.ndarray], y: Union[int, None] = None):
        #x, y -> origin[0] + x, origin[1] - y
        if not y and (isinstance(x, np.ndarray) or isinstance(x, tuple) or isinstance(x, list)):
            y = x[1]
            x = x[0]
        screen_x = x + self.origin[0]
        screen_y = -y + self.origin[1]
        return screen_x, screen_y

    def rotate_bbox(self, bbox: List[Tuple[int, int]], center: Tuple[int, int], degrees: float):
        rad = np.radians(degrees)
        cos_val = np.cos(rad)
        sin_val = np.sin(rad)
        cx, cy = center
        new_bbox = []
        for x_old, y_old in bbox:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_bbox.append([x_new + cx, y_new + cy])
        return new_bbox

    def create_cartesian_line(self, x0, y0, x1, y1, *args, **kwargs):
        screen_x0 = x0 + self.origin[0]
        screen_x1 = x1 + self.origin[0]
        screen_y0 = self.origin[1] - y0
        screen_y1 = self.origin[1] - y1
        self.create_line(screen_x0, screen_y0, screen_x1, screen_y1, *args, **kwargs)

    def render(self, prey_objects: List[MovingObject], pursuit_objects: List[PursuitObject], draw_los=True, draw_speed=True, recalculate_origin=True):
        self.delete(tk.ALL)
        if recalculate_origin:
            self.origin = (0, self.winfo_height())

        for obj in prey_objects:
            obj_x = obj.location_arr[0]
            obj_y = obj.location_arr[1]
            bbox = [(obj_x-self.prey_obj_length/2, obj_y-self.prey_obj_width/2),
                    (obj_x-self.prey_obj_length/2, obj_y+self.prey_obj_width/2),
                    (obj_x+self.prey_obj_length/2, obj_y+self.prey_obj_width/2),
                    (obj_x+self.prey_obj_length/2, obj_y-self.prey_obj_width/2)]

            coords = self.rotate_bbox(bbox, (obj_x, obj_y), obj.speed_vector.vector2deg())
            self.create_polygon([self.cartesian2screen(x) for x in coords], fill=self.prey_obj_color)
            if draw_speed:
                self.create_cartesian_line(obj_x, obj_y,
                                           obj_x + obj.speed_vector.array[0] * self.speed_vector_draw_scale,
                                           obj_y + obj.speed_vector.array[1] * self.speed_vector_draw_scale,
                                            arrow=tk.LAST, fill="red")
            if draw_los:
                for pursuit_obj in pursuit_objects:
                    self.create_cartesian_line(obj_x, obj_y, pursuit_obj.location_arr[0], pursuit_obj.location_arr[1])

        for obj in pursuit_objects:
            obj_x = obj.location_arr[0]
            obj_y = obj.location_arr[1]
            bbox = [(obj_x-self.pursuit_obj_length/2, obj_y-self.pursuit_obj_width/2),
                    (obj_x-self.pursuit_obj_length/2, obj_y+self.pursuit_obj_width/2),
                    (obj_x+self.pursuit_obj_length/2, obj_y+self.pursuit_obj_width/2),
                    (obj_x+self.pursuit_obj_length/2, obj_y-self.pursuit_obj_width/2)]

            coords = self.rotate_bbox(bbox, (obj_x, obj_y), obj.speed_vector.vector2deg())
            self.create_polygon([self.cartesian2screen(x) for x in coords], fill=self.pursuit_obj_color)
            if draw_speed:
                self.create_cartesian_line(obj_x, obj_y,
                                           obj_x + obj.speed_vector.array[0] * self.speed_vector_draw_scale,
                                           obj_y + obj.speed_vector.array[1] * self.speed_vector_draw_scale,
                                            arrow=tk.LAST, fill="red")

default_ticks = 200
current_ticks = 50
import random
gs = random.uniform(0, 1)
gs *= random.choice([1, -1])
def test(cv, prey, pursiot):
    from navigation import pure_pursuit, simple_proportional_navigation, true_proportional_navigation, augumented_proportional_navigation

    for obj in prey + pursiot:
        obj.tick()

    for ob in pursiot:
        #print(ob.speed_vector, pure_pursuit(ob, prey[0]))
        solution = augumented_proportional_navigation(ob, prey[0])
        #print(solution)
        ob.rotate(deg=solution)
        ob.update_target_data(prey[0])
    for ob in prey:
        '''global current_ticks, gs
        if current_ticks > 0:
            ob.rotate(deg=gs)
            current_ticks -= 1
        else:
            current_ticks = 50
            gs = random.uniform(0, 1)
            gs *= random.choice([1, -1])'''
        #ob.rotate(deg=-2)
        pass


    cv.render(prey, pursiot)
    cv.after(50, lambda: test(cv, prey, pursiot))

def on_a(tt):
    print("on_a")
    tt.rotate(deg=-2)

if __name__ == '__main__':
    root = tk.Tk()
    root.attributes('-fullscreen', True)
    g = SimulationCanvas(root)
    g.pack(expand=tk.YES, fill=tk.BOTH)
    #g.create_rectangle((100.545, 90, 150, 180))
    p = MovingObject("p1", 500, 500, 4.0, 4.0, 1, turn_rate_deg=3)
    p2 = PursuitObject("pursuit", 100, 400, 17.5, 0.0, 1, turn_rate_deg=3)
    g.bind("a", lambda e: p.rotate(deg=40))
    g.bind("d", lambda e: p.rotate(deg=-40))
    p2.update_target_data(p)
    root.after(100, lambda: g.render([p], [p2]))
    root.after(1000, lambda: test(g, [p], [p2]))
    root.mainloop()
