import pyray as rl
import numpy as np

class ball:
    def __init__(self, m, x, y, xvel, yvel, radius):
        self.m = m
        self.x = x
        self.y = y
        self.xvel = xvel
        self.yvel = yvel
        self.radius = radius
        
# ASSUMPTION: b1 and b2 are colliding.
def handle_bounces(b1, b2, e):
    '''
    Docstring for handle_bounces
    
    :param b1: Ball 1
    :param b2: Ball 2
    REQUIREMENT: Ball 1 and ball 2 must be colliding.
    
    RETURNS:
    New velocities of ball 1 and ball 2. (tuple(np.array(2 values)))
    '''
    pos_1 = np.array([b1.x, b1.y])
    pos_2 = np.array([b2.x, b2.y])
    
    vel_1 = np.array([b1.xvel, b1.yvel])
    vel_2 = np.array([b2.xvel, b2.yvel])
    
    
    collision_normal = pos_2 - pos_1
    collision_normal = collision_normal / np.linalg.norm(collision_normal)
    
    
    velocity_along_normal = np.dot((vel_2 - vel_1), collision_normal)
    print(velocity_along_normal)
    if velocity_along_normal >= 0:
        return (vel_1, vel_2)

    impulse_magnitude = -1 * (1 + e) * velocity_along_normal / (1/b1.m + 1/b2.m) # chatgpt formula (TODO: fix div by 0)
    
    v1x, v1y = vel_1
    v2x, v2y = vel_2
    new_vel_1 = [v1x - impulse_magnitude/b1.m * collision_normal[0], v1y - impulse_magnitude / b1.m * collision_normal[1]]
    new_vel_2 = [v2x + impulse_magnitude/b2.m * collision_normal[0], v2y + impulse_magnitude / b2.m * collision_normal[1]]
    
    return (new_vel_1, new_vel_2) # vel for b1, vel for b2

def main():
    screen_width = 900
    screen_height = 900
    
    rl.set_target_fps(4)
    
    rl.init_window(screen_width, screen_height, 'Physics simulator! [v1]')
    
    balls = [] # all objects to iterate over
    ball1 = ball(1, screen_width//2, screen_height//2, 0, 0, 50)
    ball2 = ball(1, screen_width * (3/4), screen_height * (3/4), 0, 0, 100)
    ball3 = ball(10, screen_width // 2 + 50, screen_height // 2 + 50, 10, 10, 10)
    balls.append(ball1)
    balls.append(ball2)
    balls.append(ball3)
    
    G = 2500
    bounciness = 1.01
    
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
        
        # update positions based on velocities
        for b in balls:
            b.x += b.xvel
            b.y += b.yvel
        
        # collision detection 
        for idx, b1 in enumerate(balls):
            center1 = (b1.x, b1.y)
            for b2 in balls[idx+1:]:
                center2 = (b2.x, b2.y)
                if rl.check_collision_circles(center1, b1.radius, center2, b2.radius):
                    b1vel, b2vel = handle_bounces(b1, b2, bounciness)
                    b1velx, b1vely = float(b1vel[0]), float(b1vel[1])
                    b2velx, b2vely = float(b2vel[0]), float(b2vel[1])
                    
                    b1.xvel, b1.yvel, b2.xvel, b2.yvel = b1velx, b1vely, b2velx, b2vely
        # start drawing all objects
        
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        for b in balls:
            rl.draw_circle(round(b.x), round(b.y), b.radius, rl.BLUE)
        rl.end_drawing()
    rl.close_window()
    

    
    
    
if __name__ == "__main__":
    main()