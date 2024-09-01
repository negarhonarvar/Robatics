"""HW1-Q2-C1 controller."""
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time
from controller import Robot

# in Q2 and Q3 we implemented the rest of the code and also calculated the forward kinematics of the robot
# in this part we implement 1 function to calculate right and left wheel velocity based on r and l of the robot
# and v and w. the formulas we implement in this functions are described in lecture 2 page 12 by dr. salimi
# the following link ,webots documentation on e-puck, is used for exact values of l and r
# https://cyberbotics.com/doc/guide/epuck?version=R2021b#e-puck-model

# the calculation we want to implement is as follows:

# V=r/2*(φ_1+φ_2)   =>>> V/L = r/(2*l)*(φ_1+φ_2) (1)
# W=r/(2*l)*(φ_1-φ_2) (2)
#        =>>>	(1-2)  => V/L-W =  r/l*(φ_2)  => φ_2= (V/L-W)*L/R   
#        =>>>	(1+2) => V/L+W = = r/l*(φ_1 )  => φ_1= (V/L+W)*L/R   



def get_robot_Wheel_Speed_right(v,w):
    r=0.0205
    l=0.026
    phi_1=(v+(w*l))/r
    return phi_1
 
def get_robot_Wheel_Speed_left(v,w):
    r=0.0205
    l=0.026
    phi_2=(v-(w*l))/r
    return phi_2
 
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
Time_limit = 30.0
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
# Angular speed in rad/s
# phi_1 for right wheel and phi_2 for left wheel
v = 0.
w = 0.5
max_speed_right = get_robot_Wheel_Speed_right(v,w)

max_speed_left = get_robot_Wheel_Speed_left(v,w)

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


