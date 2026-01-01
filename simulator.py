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
    
    while not rl.window_should_close():
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
        rl.end_drawing()
    rl.close_window()
    

    
    
    
if __name__ == "__main__":
    main()