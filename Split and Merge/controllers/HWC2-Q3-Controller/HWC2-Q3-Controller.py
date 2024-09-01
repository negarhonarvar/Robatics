
"""HWC2-Q3 controller."""
import math
from controller import Robot
import matplotlib.pyplot as plt
import numpy as np


# Create an instance of robot
robot = Robot()
Time_limit = 25.0
TIME_STEP = 32
Lines = []

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
    
def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[1], compass_value[0])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading


# Load Devices such as sensors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set the motors to rotate for ever
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# But with no velocity :)
left_motor.setVelocity(0)
right_motor.setVelocity(0)

sampling_period = 1  # in ms

# compass
compass = robot.getDevice('compass')
compass.enable(sampling_period)

# distance sensor
distance_sensor = robot.getDevice('distance sensor')
distance_sensor.enable(sampling_period)

# take some dummy steps in environment for safe initialization of the initial heading and position
robot.step(20)
initial_distance_sensor_value = distance_sensor.getValue()

# General parameters
max_speed_left = 1.  # Angular speed in rad/s
max_speed_right = 0.

left_motor.setVelocity(max_speed_left)
right_motor.setVelocity(max_speed_right)

samples_x = []
samples_y = []

# this approach helps us to scan 360 only one time
teta = 360.
compareable_teta = 0.

while robot.step(TIME_STEP) != -1:
    # print bumper value
    compareable_teta = teta
    teta = get_robot_heading(compass.getValues())
    # print(teta)
    distance_sensor_value = distance_sensor.getValue()
    distance_sensor_value_to_metr = distance_sensor_value * 2 / 1000
    x = math.cos(teta*math.pi/180) * distance_sensor_value_to_metr
    y = math.sin(teta*math.pi/180) * distance_sensor_value_to_metr
    samples_x.append(x)
    samples_y.append(y)
    
    
    robot.step()
    if teta > compareable_teta:
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
