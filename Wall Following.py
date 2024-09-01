from controller import Robot
from math import pi as PI, atan2, dist


point_coef  = 1
R, L = 0.0205, 0.026
threshold = 0.035
turn_velocity = 6

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
sampling_period = 1  # in ms


left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

gps = robot.getDevice("gps")
gps.enable(sampling_period)

compass = robot.getDevice("compass")
compass.enable(sampling_period) 

f = robot.getDevice('ds front')
fr = robot.getDevice('ds front right')
fl = robot.getDevice('ds front left')
r = robot.getDevice('ds right')
l = robot.getDevice('ds left')
f.enable(sampling_period)
r.enable(sampling_period)
l.enable(sampling_period)
fr.enable(sampling_period)
fl.enable(sampling_period)

# goal coordinates:
goal_x, goal_y = 0, -0.5

boundry_wall = False
losing_wall = False
turning_in_the_end = False
chase_goal = False
teta_flag = 0
left_dist = 0
counter = 0


def get_robot_heading(compass_value):
    rad = atan2(compass_value[1], compass_value[0])
    bearing = (rad - PI/2) / PI * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading
    
def get_angle() -> list[float, float, float]:
    x, y, _ = gps.getValues()
    teta = get_robot_heading(compass.getValues())*PI/180

    point = dist((goal_x, goal_y), (x, y))
    angle_plus_teta = -teta + atan2(goal_y - y, goal_x - x)
    if angle_plus_teta > PI:
        angle_plus_teta -= 2 * PI
    elif angle_plus_teta < -PI:
        angle_plus_teta += 2 * PI
    
    angle = atan2(goal_y - y, goal_x - x)
    if angle > PI:
        angle -= 2 * PI
    elif angle < -PI:
        angle += 2 * PI

    return point,angle_plus_teta,angle
    
def set_speed() -> list[float, float]:
    point, angle_plus_teta, angle = get_angle()
    increment_value = 0.01
    global point_coef , L
    angle_coef = -10
    angle_plus_teta_coef = 60
    v = point_coef*point
    w = angle_plus_teta_coef*angle_plus_teta + angle_coef*angle
    
    if point_coef < 6- increment_value:
        point_coef += increment_value

    velocity_left = (v - L*w/2)
    velocity_right = (v + L*w/2)
    if not (-6.28 <= velocity_left <= 6.28) or not (-6.28 <= velocity_right  <= 6.28):
        coeff = (6.279)/ max(abs(velocity_left), abs(velocity_right))   
        velocity_left *= coeff
        velocity_right  *= coeff
    return velocity_left, velocity_right

def get_sensor_values():
    front = f.getValue()
    left = l.getValue()
    right = r.getValue()
    front_right = fr.getValue()
    front_left = fl.getValue()
    
    return front, right, left, front_right, front_left
    
def wall_status():
    strings = ['FRONT', 'RIGHT', 'LEFT', 'FRONT_RIGHT', 'FRONT_LEFT']
    dists = get_sensor_values()
    for i in range(len(dists)):
            if dists[i] <= threshold:
                return strings[i]
    return None


def turn(string):
    if string == 'right' :
       return turn_velocity, -turn_velocity
    else :
       return -turn_velocity, turn_velocity
   
def handle_front_wall():
    global losing_wall, boundry_wall, teta_flag
    global left_dist
    left_velocity, right_velocity = turn('left')
    losing_wall = True
    teta_flag = get_robot_heading(compass.getValues())
    point_coef = 1
    return left_velocity, right_velocity


def wall_following():
    global losing_wall, boundry_wall, teta_flag
    global left_dist, counter, turning_in_the_end, chase_goal
    wall_side = wall_status()

    counter += 1
    if losing_wall:
        left_velocity, right_velocity = turn('left')
        theta = get_robot_heading(compass.getValues())
        if abs(theta - teta_flag) > 88:
            losing_wall = False
            print(f'END OF TURN {counter}')
            boundry_wall = True
            
    elif boundry_wall:
        front, right, _, _, _ = get_sensor_values()
        
        if front <= threshold:
            losing_wall = True
            teta_flag = get_robot_heading(compass.getValues())
            boundry_wall = False
        boundry_speed = 4
        velocity_coef = 60
        left_velocity, right_velocity = [boundry_speed]*2
        left_velocity -= velocity_coef * (threshold - right)
        right_velocity += velocity_coef * (threshold - right)
        
        if dist(gps.getValues()[:2], (goal_x, goal_y)) < 10**(-1):
            print('CHASING GOAL')
            boundry_wall = False
            chase_goal = True
             
    elif chase_goal:
        return set_speed()    
        
    else:
        
        if wall_side is None:
            return set_speed()     
            
        elif wall_side == 'FRONT':
            left_velocity, right_velocity = handle_front_wall()
        
        elif wall_side == 'FRONT_LEFT':
            left_velocity, right_velocity = turn('right')
            
        elif wall_side == 'FRONT_RIGHT':
            left_velocity, right_velocity = turn('left') 
        
        elif wall_side == 'LEFT':
            boundry_wall = True
            left_velocity, right_velocity = turn('right')
            
        elif wall_side == 'RIGHT':
            boundry_wall = True
            left_velocity, right_velocity = turn('left')
        
        else:
            return set_speed()
     
    if not (-6.28 <= left_velocity <= 6.28) or not (-6.28 <= right_velocity  <= 6.28):
        coeff = (6.279)/ max(abs(left_velocity), abs(right_velocity ))   
        left_velocity *= coeff
        right_velocity  *= coeff
    return left_velocity, right_velocity

while robot.step(timestep) != -1:

    left_velocity, right_velocity = wall_following()
    
    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)
