from controller import Robot

robot = Robot()

time_step = int(robot.getBasicTimeStep()/2)

left_wheel = robot.getDevice('left wheel motor')
right_wheel = robot.getDevice('right wheel motor')

max_speed_left = 4.0
max_speed_right = 4.0

left_wheel.setVelocity(max_speed_left)
right_wheel.setVelocity(max_speed_right)

left_wheel.setPosition(float("inf"))
right_wheel.setPosition(float("inf"))

touch_sensor = robot.getDevice('touch sensor')
touch_sensor.enable(int(time_step/4))

max_force = 0.0
while robot.step(time_step) != -1:

    if touch_sensor.getValue() > 0:        
        collision_force = touch_sensor.getValue()/10.0
        if collision_force > max_force:
            max_force = collision_force
    if touch_sensor.getValue() > 5.0:
        break

left_wheel.setVelocity(0.0)
right_wheel.setVelocity(0.0)
print(max_force)
