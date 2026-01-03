import pyray as rl
import numpy as np

class Ball:
    def __init__(self, m, x, y, xvel, yvel, radius, color=None):
        self.m = m
        self.x = x
        self.y = y
        self.xvel = xvel
        self.yvel = yvel
        self.radius = radius
        if color is not None: # avoid fallback to falsy for C-bindings
            self.color = color
        else:
            self.color = rl.BLUE

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
    '''
    Docstring for handle_bounces
    
    :param b1: Ball 1
    :param b2: Ball 2
    REQUIREMENT: Ball 1 and ball 2 must be colliding.
    
    Does not return, modifies the data of b1 and b2.
    '''
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

def validate_inputs(new_mass_str, new_x_str, new_y_str, new_radius_str, new_color_str, screen_width, screen_height):
    if not new_mass_str or not new_x_str or not new_y_str or not new_radius_str or not new_color_str:
        return False
    mass_valid = int(new_mass_str) > 0 and int(new_mass_str) <= 10**9
    radius_valid = int(new_radius_str) > 0 and int(new_radius_str) < min(screen_height, screen_width)
    
    color_str = new_color_str.upper()
    color_valid = color_str in ("LIGHTGRAY", "GRAY", "DARKGRAY", "YELLOW", "GOLD", 'ORANGE', 'PINK', 'RED', 'MAROON', 'GREEN', 'LIME', 'DARKGREEN', 'SKYBLUE', 'BLUE', 'DARKBLUE', 'PURPLE', 'VIOLET', 'DARKPURPLE', 'BEIGE', 'BROWN', 'DARKBROWN', 'WHITE', 'BLACK', 'MAGENTA', 'RAYWHITE') #  blank not valid color
    return mass_valid and radius_valid and color_valid
    

# May have issues with int(non-numeric string) throwing errors. TODO: add try-catch
def submit_boxes(input):
    new_mass = 0
    new_x_vel = 0
    new_y_vel = 0
    new_radius = 0
    new_color = None
    for box in input:
        if box.label == 'mass':
            try:
                new_mass = int(box.text)
            except ValueError:
                return None
        if box.label == 'x velocity':
            try:
                new_x_vel  = int(box.text)
            except ValueError:
                return None
        if box.label == 'y velocity':
            try:
                new_y_vel = int(box.text)
            except ValueError:
                return None
        if box.label == 'radius':
            try:
                new_radius = int(box.text)
            except ValueError:
                return None
        if box.label == 'color':
            try:
                new_color = getattr(rl, box.text.upper(), None)
            except ValueError:
                return None
    
    menu_on = False
    cooldown = 0.5
    
    mouse_x, mouse_y = rl.get_mouse_x(), rl.get_mouse_y()
    return Ball(new_mass, mouse_x, mouse_y, new_x_vel, new_y_vel, new_radius, new_color)

def main():
    screen_width = 900
    screen_height = 900
    
    rl.set_target_fps(50)
    
    rl.init_window(screen_width, screen_height, 'Physics simulator! [v1]')
    
    balls = [] # all objects to iterate over
    
    G = 100
    bounciness = 0.9
    universal_gravity = 0.0
    
    menu_on = False
    
    mouse_x, mouse_y = 0, 0
    
    font_size = 40
    cooldown = 0
    
    while not rl.window_should_close():
        cooldown = max(0, cooldown - 1/60) # HACK
        if rl.is_mouse_button_down(0) and not menu_on and not cooldown:
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
            menu_on = True
        
        if menu_on:
            mouse_pos = rl.get_mouse_position()
            for box in inputs:
                box.handle_input()
            if rl.check_collision_point_rec(mouse_pos, submit_box.rect) and rl.is_mouse_button_pressed(0):
                ball = submit_boxes(inputs)
                if ball is not None:
                    balls.append(ball)
                menu_on = False
                cooldown = 0.5
            if rl.check_collision_point_rec(mouse_pos, close_box.rect) and rl.is_mouse_button_pressed(0):
                menu_on = False
                cooldown = 0.5
        
            
            if rl.check_collision_point_rec(mouse_pos, close_box.rect) and rl.is_mouse_button_down(0):
                ...
        for b1 in balls:
            force_x = 0
            force_y = 0
            
            for b2 in balls:
                numerator = G * b1.m * b2.m
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
            b.yvel += universal_gravity
        
        # update positions based on velocities
        for b in balls:
            b.x += b.xvel
            b.y += b.yvel
        
        # collision handling with other balls
        for idx, b1 in enumerate(balls):
            center1 = (b1.x, b1.y)
            for b2 in balls[idx+1:]:
                center2 = (b2.x, b2.y)
                if rl.check_collision_circles(center1, b1.radius, center2, b2.radius):
                    handle_bounces(b1, b2, bounciness)
        
        # collision handling with border
        for b in balls:
            r = b.radius
            left, right, top, bottom = b.x - r, b.x + r, b.y - r, b.y + r
            
            if left <= 0:
                b.x = r
                b.xvel *= -1 * bounciness
            if right >= screen_width:
                b.x = screen_width - r
                b.xvel *= -1 * bounciness
            if bottom >= screen_height:
                b.y = screen_height - r
                b.yvel *= -1 * bounciness
            if top <= 0:
                b.y = r
                b.yvel *= -1 * bounciness
        
        # start drawing all objects
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        for b in balls:
            rl.draw_circle(round(b.x), round(b.y), b.radius, b.color)
        
        if menu_on:
            for box in inputs:
                box.draw()
            submit_box.draw()
            close_box.draw()
        rl.end_drawing()
    rl.close_window()
    

    
    
    
if __name__ == "__main__":
    main()