"""HW1-Q2-C1 controller."""
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time
from controller import Robot


# Get robot's heading in degree based on compass values
def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[0], compass_value[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading


# Create an instance of robot
robot = Robot()
Time_limit = 10.0
TIME_STEP = 64

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
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)

robot.step(1000)  # take some dummy steps in environment for safe initialization of the initial heading and position
initial_gps_value = gps.getValues()
initial_compass_value = compass.getValues()

# General parameters
max_speed_left = 1.  # Angular speed in rad/s
max_speed_right = 1.

left_motor.setVelocity(max_speed_left)
right_motor.setVelocity(max_speed_right)

coordinates_x = []
coordinates_y = []

head = []
start_time = time.time()
timeArray = []

while robot.step(TIME_STEP) != -1:
    current_coordinate = gps.getValues()  # current robot position via gps sensor
    coordinates_x.append(current_coordinate[0])
    coordinates_y.append(current_coordinate[1])
    head.append(get_robot_heading(compass.getValues()))
    timeArray.append(robot.getTime())
    robot.step()
    if robot.getTime() > Time_limit:
        break

# Create a line plot
plt.plot(coordinates_x, coordinates_y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('X-Y Plot')

# Show the plot
plt.show()

plt.plot(timeArray, head)
plt.xlabel('Time in Seconds')
plt.ylabel('Heading degree')
plt.title('θ-t Plot')

plt.show()

