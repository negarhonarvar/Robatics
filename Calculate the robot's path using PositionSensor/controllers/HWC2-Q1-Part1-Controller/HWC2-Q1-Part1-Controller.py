"""HWC2-Q1-Part1 controller."""
import math
import matplotlib.pyplot as plt
import time
from controller import Robot

r = 0.02
l = 0.026

# Get robot's heading in degree based on compass values
def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[1], compass_value[0])
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
left_position_sensor = robot.getDevice('left wheel sensor')
right_position_sensor = robot.getDevice('right wheel sensor')

# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)
left_position_sensor.enable(sampling_period)
right_position_sensor.enable(sampling_period)

robot.step(1000)  # take some dummy steps in environment for safe initialization of the initial heading and position
initial_gps_value = gps.getValues()
initial_compass_value = compass.getValues()
initial_left_position_sensor_value = left_position_sensor.getValue()
initial_right_position_sensor_value = right_position_sensor.getValue()

print(initial_left_position_sensor_value)
print(initial_right_position_sensor_value)

# General parameters
max_speed_left = 1.  # Angular speed in rad/s
max_speed_right = 1.

left_motor.setVelocity(max_speed_left)
right_motor.setVelocity(max_speed_right)

# kinematic
teta = 0

x_dot_robot = (r / 2) * (max_speed_right + max_speed_left)
teta_dot = (r / 2 * l) * (max_speed_right - max_speed_left)

x_dot = math.cos(teta) * x_dot_robot
y_dot = math.sin(teta) * x_dot_robot

print(x_dot)
print(y_dot)
print(teta_dot)

coordinates_x = []
coordinates_y = []

position_sensor_coordinates_x = []
position_sensor_coordinates_y = []

head = []

position_sensor_head = []

start_time = time.time()
timeArray = []

while robot.step(TIME_STEP) != -1:
    left_position_sensor_value = left_position_sensor.getValue()
    right_position_sensor_value = right_position_sensor.getValue()
    
    current_coordinate = gps.getValues()  # current robot position via gps sensor
    coordinates_x.append(current_coordinate[0])
    coordinates_y.append(current_coordinate[1])
    head.append(get_robot_heading(compass.getValues()))
    time = robot.getTime()
    timeArray.append(time)
    
    # calc position_sensor cord
    position_sensor_coordinates_x.append(time * x_dot)
    position_sensor_coordinates_y.append(time * y_dot)
    position_sensor_head.append(time * teta_dot)
    
    robot.step()
    if robot.getTime() > Time_limit:
        break

print(position_sensor_coordinates_x)
print(position_sensor_coordinates_y)
print(position_sensor_head)

# Create a line plot
plt.plot(coordinates_x, coordinates_y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('X-Y Plot')

# Show the plot
plt.show()

# Create a line plot
plt.plot(position_sensor_coordinates_x, position_sensor_coordinates_y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('X-Y Plot')

# Show the plot
plt.show()


# Create a line plot
plt.plot(timeArray, head)
plt.xlabel('Time in Seconds')
plt.ylabel('Heading degree')
plt.title('θ-t Plot')

# Show the plot
plt.show()

# Create a line plot
plt.plot(timeArray, position_sensor_head)
plt.xlabel('Time in Seconds')
plt.ylabel('Heading degree')
plt.title('θ-t Plot')

# Show the plot
plt.show()
