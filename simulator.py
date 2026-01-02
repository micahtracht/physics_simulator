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
        if color:
            self.color = color
        else:
            self.color = rl.BLUE

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


def main():
    screen_width = 900
    screen_height = 900
    
    rl.set_target_fps(50)
    
    rl.init_window(screen_width, screen_height, 'Physics simulator! [v1]')
    
    balls = [] # all objects to iterate over
    ball1 = Ball(1, screen_width//2 + 100, screen_height//2 + 100, 20, 2, 20, rl.GREEN)
    ball2 = Ball(1, screen_width//2 - 100, screen_height//2 + 100, -2, -2, 20, rl.BLACK)
    balls.append(ball1)
    balls.append(ball2)
    
    G = 0
    bounciness = 0.9
    universal_gravity = 0.1
    
    new_mass = 0
    new_x_vel = 0
    new_y_vel = 0
    new_radius = 0
    new_color = rl.WHITE
    
    mass_str = ""
    new_x_str = ""
    new_y_str = ""
    new_radius_str = ""
    new_color_str = ""
    
    
    # initialize boxes with starting positions,  will be overriden when user clicks.
    boudning_box = rl.Rectangle(0, 0, 200, 900) # MAY CHANGE NUMBERS
    mass_box = rl.Rectangle(0, + 20, 0 + 30, 150, 50)
    x_vel_box = rl.Rectangle(0 + 20, 0 + 110, 150, 50)
    y_vel_box = rl.Rectangle(0 + 20, 0 + 190, 150, 50)
    radius_box = rl.Rectangle(0 + 20, 0 + 270, 150, 50)
    color_box = rl.Rectangle(0 + 20, 0 + 350, 150, 50)
    submit_box = rl.Rectangle(20, 430, 150, 50)
    
    menu_on = False
    
    mouse_x, mouse_y = 0, 0
    
    font_size = 40
    
    while not rl.window_should_close():
        if rl.is_mouse_button_down(0) and not menu_on:
            mouse_x = rl.get_mouse_x()
            mouse_y = rl.get_mouse_y()
            boudning_box = rl.Rectangle(mouse_x, mouse_y, 190, 500) # MAY CHANGE NUMBERS
            mass_box = rl.Rectangle(mouse_x + 20, mouse_y + 30, 150, 50)
            x_vel_box = rl.Rectangle(mouse_x + 20, mouse_y + 110, 150, 50)
            y_vel_box = rl.Rectangle(mouse_x + 20, mouse_y + 190, 150, 50)
            radius_box = rl.Rectangle(mouse_x + 20, mouse_y + 270, 150, 50)
            color_box = rl.Rectangle(mouse_x + 20, mouse_y + 350, 150, 50)
            submit_box = rl.Rectangle(mouse_x + 20, mouse_y + 430, 150, 50)
            print(boudning_box.x)
            print(rl.get_mouse_x())
            menu_on = True
        
        # handle if user clicked submit
        if menu_on:
            if rl.check_collision_point_rec(rl.get_mouse_position(), mass_box):
                key_press = rl.get_char_pressed()
                if key_press >= 32 and key_press <= 150:
                    mass_str += key_press
            
            if rl.check_collision_point_rec(rl.get_mouse_position(), x_vel_box):
                ...
            
            if rl.check_collision_point_rec(rl.get_mouse_position(), submit_box):
                ... # TODO: handle submission by updating values
                # TODO: make new ball
            
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
            rl.draw_rectangle_rec(boudning_box, rl.BLACK)
            rl.draw_rectangle_rec(mass_box, rl.GRAY)
            rl.draw_rectangle_rec(x_vel_box, rl.RED)
            rl.draw_rectangle_rec(y_vel_box, rl.RED)
            rl.draw_rectangle_rec(radius_box, rl.BLUE)
            rl.draw_rectangle_rec(color_box, rl.PURPLE)
            rl.draw_rectangle_rec(submit_box, rl.GREEN)
            
            # draw text
            rl.draw_text('mass:', mouse_x - 180, mouse_y + 20, font_size, rl.GRAY)
            rl.draw_text('x velocity:', mouse_x - 220, mouse_y + 100, font_size, rl.RED)
            rl.draw_text('y velocity:', mouse_x - 220, mouse_y + 180, font_size, rl.RED)
            rl.draw_text('radius:', mouse_x - 180, mouse_y + 260, font_size, rl.BLUE)
            rl.draw_text('color:', mouse_x - 180, mouse_y + 340, font_size, rl.PURPLE)
            rl.draw_text('submit:', mouse_x - 180, mouse_y + 420, font_size, rl.GREEN)
        rl.end_drawing()
    rl.close_window()
    

    
    
    
if __name__ == "__main__":
    main()