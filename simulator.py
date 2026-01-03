import pyray as rl
import numpy as np
from dataclasses import dataclass
import constants
import simstate

@dataclass
class Ball:
    m: float
    x: float
    y: float
    xvel: float
    yvel: float
    radius: float
    color: rl.Color = rl.BLUE

class InputBox:
    def __init__(self, x, y, w, h, label, default_str="", box_color=rl.LIGHTGRAY, label_color=rl.BLACK):
        self.rect = rl.Rectangle(x, y, w, h)
        self.label = label
        self.text = default_str
        self.active = False
        self.box_color = box_color
        self.label_color = label_color
    
    def handle_input(self):
        if rl.check_collision_point_rec(rl.get_mouse_position(), self.rect):
            if rl.is_mouse_button_pressed(0):
                self.active = True
        elif rl.is_mouse_button_pressed(0):
            self.active = False
        
        if self.active:
            key = rl.get_char_pressed()
            while key > 0:
                if 32 <= key <= 125:
                    self.text += chr(key)
                key = rl.get_char_pressed()
            if rl.is_key_pressed(rl.KeyboardKey.KEY_BACKSPACE):
                self.text = self.text[:-1]
    
    def draw(self):
        if self.active:
            rl.draw_rectangle_rec(self.rect, rl.RED)
        else:
            rl.draw_rectangle_rec(self.rect, self.box_color)
        rl.draw_text(self.label, int(self.rect.x) - 2 * rl.measure_text(self.label, 10) - 10, int(self.rect.y + self.rect.height // 2 - 10), 20, self.label_color)
        rl.draw_text(self.text, int(self.rect.x) + 5, int(self.rect.y) + 5, 20, rl.BLACK)

# ASSUMPTION: b1 and b2 are colliding
def apply_correction(b1, b2):
    pos_1 = np.array([b1.x, b1.y], dtype=float)
    pos_2 = np.array([b2.x, b2.y], dtype=float)
    
    R = b1.radius + b2.radius
    dist = np.linalg.norm(pos_2 - pos_1)
    if dist < 1e-3: # on top of each other, do nothing (TODO: design better behavior)
        return
    n = (pos_2 - pos_1) / dist
    
    inv_m1 = 1/b1.m
    inv_m2 = 1/b2.m
    
    pen = R - dist
    slop = 1e-3 * R
    alpha = 0.5 # alpha in [0.2, 0.8] typically
    if pen > 0:
        del_p = (max(pen - slop, 0)/(inv_m1 + inv_m2)) * alpha * n
        pos_1 -= inv_m1 * del_p
        pos_2 += inv_m2 * del_p
    b1.x, b1.y = pos_1
    b2.x, b2.y = pos_2

# ASSUMPTION: b1 and b2 are colliding.
def handle_bounces(b1, b2, e):
    pos_1 = np.array([b1.x, b1.y], dtype=float)
    pos_2 = np.array([b2.x, b2.y], dtype=float)
    
    vel_1 = np.array([b1.xvel, b1.yvel], dtype=float)
    vel_2 = np.array([b2.xvel, b2.yvel], dtype=float)
    
    
    n = pos_2 - pos_1
    dist = np.linalg.norm(n)
    if dist < 1e-3: # on top of each other, do nothing
        return
    n /= dist
    
    
    vel_n = np.dot((vel_2 - vel_1),  n)
    if vel_n >= 0: # already diverging, do nothing
        return

    inv_m1 = 1/b1.m
    inv_m2 = 1/b2.m
    
    j = -1 * (1 + e) * vel_n / (inv_m1 + inv_m2) # chatgpt formula
    
    v1x, v1y = vel_1
    v2x, v2y = vel_2
    new_vel_1 = [v1x - j/b1.m * n[0], v1y - j / b1.m * n[1]]
    new_vel_2 = [v2x + j/b2.m * n[0], v2y + j / b2.m * n[1]]
    
    # velocities are updated
    b1.xvel, b1.yvel = new_vel_1
    b2.xvel, b2.yvel = new_vel_2
    
    # handle position correction
    apply_correction(b1, b2)
    

def submit_boxes(inputs):
    new_mass = 0
    new_x_vel = 0
    new_y_vel = 0
    new_radius = 0
    new_color = None
    
    for box in inputs:
        if box.label == 'mass':
            try:
                new_mass = float(box.text)
            except ValueError:
                simstate.alert_user = constants.alert_length
                return None
        if box.label == 'x velocity':
            try:
                new_x_vel  = float(box.text)
            except ValueError:
                simstate.alert_user = constants.alert_length
                return None
        if box.label == 'y velocity':
            try:
                new_y_vel = float(box.text)
            except ValueError:
                simstate.alert_user = constants.alert_length
                return None
        if box.label == 'radius':
            try:
                new_radius = float(box.text)
            except ValueError:
                simstate.alert_user = constants.alert_length
                return None
        if box.label == 'color':
            new_color = getattr(rl, box.text.upper(), None)
    
    if new_mass <= 0 or new_radius <= 0:
        simstate.alert_user = constants.alert_length
        return None

    if new_color is None:
        new_color = rl.BLUE
    
    mouse_x, mouse_y = rl.get_mouse_x(), rl.get_mouse_y()
    return Ball(new_mass, mouse_x, mouse_y, new_x_vel, new_y_vel, new_radius, new_color)

def handle_gravity_physics(balls):
    for i, b1 in enumerate(balls):
        force_x = 0
        force_y = 0
        
        for j, b2 in enumerate(balls):
            if i == j:
                continue
            numerator = constants.G * b1.m * b2.m
            denominator = ((b2.x - b1.x)**2 + (b2.y - b1.y)**2)**(3/2)
            
            force_x += numerator * (b2.x - b1.x) / denominator if denominator > 0.001 else 0
            force_y += numerator * (b2.y - b1.y) / denominator if denominator > 0.001 else 0
        
        # now we have forces, use that to calculate accelerations and update positions
        accel_x = force_x / b1.m
        accel_y = force_y / b1.m
        b1.xvel += accel_x
        b1.yvel += accel_y
    
    # handle universal gravity
    for b in balls:
        b.yvel += constants.universal_gravity
    
    # update positions based on velocities
    for b in balls:
        b.x += b.xvel
        b.y += b.yvel

def handle_collisions_other_balls(balls):
    for idx, b1 in enumerate(balls):
        center1 = (b1.x, b1.y)
        for b2 in balls[idx+1:]:
            center2 = (b2.x, b2.y)
            if rl.check_collision_circles(center1, b1.radius, center2, b2.radius):
                handle_bounces(b1, b2, constants.bounciness)

def handle_collisions_border(balls):
    for b in balls:
        r = b.radius
        left, right, top, bottom = b.x - r, b.x + r, b.y - r, b.y + r
        
        if left <= 0:
            b.x = r
            b.xvel *= -1 * constants.bounciness
        if right >= constants.screen_width:
            b.x = constants.screen_width - r
            b.xvel *= -1 * constants.bounciness
        if bottom >= constants.screen_height:
            b.y = constants.screen_height - r
            b.yvel *= -1 * constants.bounciness
        if top <= 0:
            b.y = r
            b.yvel *= -1 * constants.bounciness

def handle_creation(balls, inputs, submit_box, close_box):
    mouse_pos = rl.get_mouse_position()
    for box in inputs:
        box.handle_input()
    if rl.check_collision_point_rec(mouse_pos, submit_box.rect) and rl.is_mouse_button_pressed(0):
        ball = submit_boxes(inputs)
        if ball is not None:
            balls.append(ball)
        simstate.menu_on = False
        simstate.cooldown = constants.cooldown_seconds
    if rl.check_collision_point_rec(mouse_pos, close_box.rect) and rl.is_mouse_button_pressed(0):
        simstate.menu_on = False
        simstate.cooldown = constants.cooldown_seconds

def main():
    screen_width = constants.screen_width
    screen_height = constants.screen_height
    
    rl.set_target_fps(constants.fps)
    rl.init_window(screen_width, screen_height, 'Physics simulator! [v1]')
    
    balls = [] # all objects to iterate over
    
    while not rl.window_should_close():
        simstate.cooldown = max(0, simstate.cooldown - 1/constants.fps)
        simstate.alert_user = max(0, simstate.alert_user - 1/constants.fps)
        if rl.is_mouse_button_down(0) and not simstate.menu_on and not simstate.cooldown:
            mouse_x = rl.get_mouse_x()
            mouse_y = rl.get_mouse_y()
            
            inputs = [
                InputBox(mouse_x + 20, mouse_y + 30, 150, 50, 'mass'),
                InputBox(mouse_x + 20, mouse_y + 110, 150, 50, 'x velocity'),
                InputBox(mouse_x + 20, mouse_y + 190, 150, 50, 'y velocity'),
                InputBox(mouse_x + 20, mouse_y + 270, 150, 50, 'radius'),
                InputBox(mouse_x + 20, mouse_y + 350, 150, 50, 'color'),
            ]
            submit_box = InputBox(mouse_x + 20, mouse_y + 430, 150, 50, 'submit', box_color=rl.GREEN)
            close_box = InputBox(mouse_x + 20, mouse_y - 50, 150, 50, 'close', box_color=rl.RED)
            simstate.menu_on = True
        
        if simstate.menu_on:
            handle_creation(balls, inputs, submit_box, close_box)
        
        handle_gravity_physics(balls)
        handle_collisions_other_balls(balls)
        handle_collisions_border(balls)
        
        # start drawing all objects
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        for b in balls:
            rl.draw_circle(round(b.x), round(b.y), b.radius, b.color)
        
        if simstate.menu_on:
            for box in inputs:
                box.draw()
            submit_box.draw()
            close_box.draw()
        
        if simstate.alert_user:
            to_draw = "Your input parameters were invalid!"
            rl.draw_text(to_draw, constants.screen_width // 2 - rl.measure_text(to_draw, 40) // 2, constants.screen_height // 2, 40, rl.RED)
        rl.end_drawing()
    rl.close_window()
    
if __name__ == "__main__":
    main()