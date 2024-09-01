"""HWC2-Q1-Part3 controller."""
from controller import Robot, Motor
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, cos, sin, pi

time_delay = 20

R, L = 0.02, 0.026

def get_robot_heading(compass_value):
    rad = atan2(compass_value[1], compass_value[0])
    bearing = (rad - pi/2) / pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading
    

def kinematic(phi1, phi2, teta):
    v = R*(phi1 + phi2) / 2
    w = R*(phi1 - phi2) / (2*L)

    
    x_dot    = cos(-teta)*v
    y_dot    = -sin(-teta)*v
    
    return x_dot, y_dot, w
    
def new_loc(x, y, teta, x_dot, y_dot, teta_dot, t):
    return x+t*x_dot, y+t*y_dot, teta+t*teta_dot

def get_speed(s0, s1, t0, t1):
    return (s1 - s0)/(t1-t0)


robot = Robot()

timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))


sampling_period = 1  # in ms
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")

# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)

left_wheel_PoSensor = robot.getDevice("left wheel sensor")
right_wheel_PoSensor = robot.getDevice("right wheel sensor")

left_wheel_PoSensor.enable(sampling_period)
right_wheel_PoSensor.enable(sampling_period)


now, before = 0, 0 
spin1, spin2 = 0, 0
spin1_before, spin2_before = 0, 0

x, y, teta = 0, 0, 0
xs, ys, tetas, ts = [x], [y], [teta], [0]


while robot.step(timestep) != -1:

    V1 = sin(robot.getTime())
    V2 = -cos(robot.getTime())
    
    left_motor.setVelocity(V1)
    right_motor.setVelocity(V2)    
    
    spin1_before = spin1
    spin1 = right_wheel_PoSensor.getValue()
    
    spin2_before = spin2
    spin2 = left_wheel_PoSensor.getValue()

    before = now
    now = robot.getTime()
    
    phi1 = get_speed(spin1_before, spin1, before, now)
    phi2 = get_speed(spin2_before, spin2, before, now)
    
    x_dot, y_dot, teta_dot = kinematic(phi1, phi2, teta)
    
    x, y, teta = new_loc(x, y, teta, x_dot, y_dot, teta_dot, now-before)
    
    xs.append(x)
    ys.append(y)
    tetas.append((teta*180/pi)%360)
    ts.append(now)
    
    X, Y = gps.getValues()[0], gps.getValues()[1]
    TETA = get_robot_heading(compass.getValues())
    
    if robot.getTime() > time_delay:
        break
    
plt.plot(xs, ys)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('X-Y Plot')

plt.show()

plt.plot(ts, tetas)
plt.xlabel('Time in Seconds')
plt.ylabel('Heading degree')
plt.title('θ-t Plot')

plt.show()

