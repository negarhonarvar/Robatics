from controller import Robot
import matplotlib.pyplot as plt
import math
import numpy as np
from math import pi as PI, atan2, dist

point_coef = 1
R, L= 0.0205, 0.026
threshold = 0.035
turn_velocity = 6
boundry_wall = False
losing_wall = False
teta_flag = 0  # Initialize teta_flag as a global variable

robot = Robot()
timestep = int(robot.getBasicTimeStep())
sampling_period = 1  # in ms

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
front_sensor = robot.getDevice('ds front')
right_sensor = robot.getDevice('ds right')
left_sensor = robot.getDevice('ds left')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)
front_sensor.enable(sampling_period)
right_sensor.enable(sampling_period)
left_sensor.enable(sampling_period)

hit_point = (100, 100)  # Initial hit point
hit_count = 0
chase_goal = False

samples_x = []
samples_y = []

# Goal coordinates:
goal_x = 0.
goal_y = -0.5

Lines = []

def get_robot_heading(compass_value):
    rad = atan2(compass_value[1], compass_value[0])
    bearing = (rad - PI/2) / PI * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading

def distance_to_line(point, line_start, line_end):
    x1, y1 = line_start
    x2, y2 = line_end
    x0, y0 = point

    line_length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    distance = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / line_length
    
    return distance

def split(P):
    threshold = 0.0001
    line_start = P[0]
    line_end = P[-1]
    distances = [distance_to_line(point, line_start, line_end) for point in P]
    max_dist_index = np.argmax(distances)
    if distances[max_dist_index]>threshold :
        split(P[:max_dist_index+1])
        split(P[max_dist_index:])
    else:
        Lines.append((P[0], P[1]))

def get_robot(compass_value):
    rad = atan2(compass_value[1], compass_value[0])
    bearing = (rad - PI / 2) / PI * 180.0
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
    
    if point_coef < 6- increment_value :
        point_coef += increment_value

    velocity_left = (v - L*w/2)
    velocity_right = (v + L*w/2)
    if not (-6.28 <= velocity_left <= 6.28) or not (-6.28 <= velocity_right  <= 6.28):
        coeff = (6.279)/ max(abs(velocity_left), abs(velocity_right))   
        velocity_left *= coeff
        velocity_right  *= coeff
    return velocity_left, velocity_right

def get_sensor_values(front_sensor,left_sensor, right_sensor):
    front = front_sensor.getValue()
    left = left_sensor.getValue()
    right = right_sensor.getValue()

    return front, right, left

def wall_status():
    strings = ['FRONT', 'RIGHT', 'LEFT']
    distances = get_sensor_values(front_sensor,left_sensor, right_sensor)
    for i in range(len(distances)):
        if distances[i] <= threshold:
            return strings[i]
    return None

def turn(string):
    if string == 'right' :
       return turn_velocity, -turn_velocity
    else :
       return -turn_velocity, turn_velocity


def handle_front_wall():
    global losing_wall, boundry_wall
    left_velocity, right_velocity = turn('left')
    losing_wall = True
    teta_flag = get_robot(compass.getValues())
    point_coef = 1
    return left_velocity, right_velocity

def wall_following():
    global losing_wall, boundry_wall, hit_count, hit_point, chase_goal,teta_flag
    obstacle = wall_status()

    if losing_wall:
        left_velocity, right_velocity = turn('left')
        theta = get_robot(compass.getValues())
        if abs(theta - teta_flag) > 88:
            losing_wall = False
            boundry_wall = True

    elif boundry_wall:
        front, right, _ = get_sensor_values(front_sensor,left_sensor, right_sensor)

        if front <= threshold:
            losing_wall = True
            teta_flag = get_robot(compass.getValues())
            boundry_wall = False
        senosor_coef = 200
        boundry_speed = 6
        left_velocity, right_velocity = [boundry_speed] * 2
        left_velocity -= senosor_coef * (threshold - right)
        right_velocity += senosor_coef * (threshold - right)

        x, y = gps.getValues()[:2]

        if dist((x, y), hit_point) < 10**(-2) and (hit_count > 1000):
            boundry_wall = False
            chase_goal = True

        if dist((x, y), (goal_x, goal_y)) < dist(hit_point, (goal_x, goal_y)):
            hit_point = (x, y)
            hit_count += 1

    elif chase_goal:
        return set_speed()

    else:
        if obstacle is None:
            return set_speed()

        elif obstacle == 'FRONT':
            left_velocity, right_velocity = handle_front_wall()

        elif obstacle == 'LEFT':
            boundry_wall = True
            left_velocity, right_velocity = turn('right')

        elif obstacle == 'RIGHT':
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

    teta = get_robot(compass.getValues())
    left_velocity, right_velocity = wall_following()
    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)
    sensors = get_sensor_values(front_sensor,left_sensor, right_sensor)
    for i in range (0,2):
        if sensors[i]<0.05 :
            x_pose = gps.getValues()[0]
            y_pose = gps.getValues()[1]
            distance_sensor_value_to_metr = sensors[i]*2/1000
            x = math.cos((teta*math.pi/180)-PI/2) * distance_sensor_value_to_metr+x_pose
            y = math.sin((teta*math.pi/180)-PI/2) * distance_sensor_value_to_metr+y_pose
            samples_x.append(x)
            samples_y.append(y)
            continue
                
    if robot.getTime() > 120 :
        break

plt.title("Identified dotes before Split and Merge")
plt.scatter(samples_x, samples_y)
plt.show()

# for easier implimentation of split and merge function , 
# its better to merge sample_x and sample_y arrays into one

Coordinates = [(samples_x[i], samples_y[i]) for i in range(len(samples_x))]

split(Coordinates)

print(Lines)

#splitted lines:
for i in range(len(Lines)):
    
    starting_point = Lines[i][0]
    finishing_point = Lines[i][1]
    
    plt.plot([starting_point[0], finishing_point[0]], [starting_point[1], finishing_point[1]], c='blue')

plt.title("room map after split and before merge")
plt.legend()
plt.show()

# merging: 
for i in range(len(Lines)):
    
    starting_point = Lines[i][0]
    finishing_point = Lines[i][1]
    new_line = Lines[(i+1)%len(Lines)][0]
    
    plt.plot([starting_point[0], finishing_point[0]], [starting_point[1], finishing_point[1]], c='black')
    plt.plot([finishing_point[0], new_line[0]], [finishing_point[1], new_line[1]], c='black')
plt.title("room map after split and merge")
plt.legend()
plt.show()
