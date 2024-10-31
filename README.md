# Controllers for epoch in webots
For better understanding of the codes, please first check webots documents regarding epoch

## Basic Controllers
In the first section we shall implement a series of basic controllers for epoch.
### Applying diffrent Angular velocity
In order to better understand the concepts of epoch movement we code a simple controller in which by changing values of parameters ' max_speed_left ' and ' max_speed_right ' we can access and adjust angular velocity of each wheel. we use ' GPS ' and ' Compass ' sensors to plot the movement of our robot. x-y of robot movement and time-teta plot of robot heading in each time plot is avaiable for 3 diffrent set of angular velocity values :

Part 1 : φ˙ 1 = 1rad/s, φ˙ 2 = 1rad/s

<img src="https://github.com/user-attachments/assets/7d633139-4592-421f-a8c5-c91d8e20c629" width = "400" >

<img src="https://github.com/user-attachments/assets/30fdf348-b154-4f53-902c-dfc8afb39abe" width = "400" >

Part 2 : φ˙ 1 = 1rad/s, φ˙ 2 = −1rad/s

<img src="https://github.com/user-attachments/assets/d06a6a94-6708-4957-855a-860eef42c106" width = "400" >

<img src="https://github.com/user-attachments/assets/f9b2b1f9-5cb1-45e6-afea-e087e2b52c88" width = "400" >

Part 3 : φ˙ 1(t) = sin t rad/s, φ˙ 2(t) = −cos t rad/s

<img src="https://github.com/user-attachments/assets/38af5997-e02c-405a-b45e-2e8777057047" width = "400" >

<img src="https://github.com/user-attachments/assets/c8ca2b33-2a16-408e-bed9-290d05ee9584" width = "400" >


### Forward Cinematic Function
To calculate the final position and orientation of the robot after each command, we will use the following kinematic equations for a differential drive robot:

Kinematic Equations:
Given the initial position 
(𝑥,𝑦,𝜃) and the velocities of the left and right wheels 𝑣1 and 𝑣2, along with the time duration 𝑡, and the distance between the two wheels 𝑙, the following equations are used:
If 𝑣1=𝑣2 (Straight motion):The robot moves in a straight line.
Δ𝑥=𝑣1⋅𝑡⋅cos(𝜃)
Δ𝑦=𝑣1⋅𝑡⋅sin(𝜃)
𝜃𝑛=θ (no change in orientation)
If 𝑣1≠𝑣2(Turning motion):The robot follows an arc of a circle.
Angular velocity: 𝜔=(𝑣2−𝑣1)/𝑙
Radius of the turn: 𝑅=𝑙/2⋅((𝑣1+𝑣2)/(𝑣2−𝑣1))
Change in orientation: Δθ=ω⋅t
Position change:Δx=R⋅(sin(θ+Δθ)−sin(θ))  , Δy=−R⋅(cos(θ+Δθ)−cos(θ))
Final orientation: 𝜃𝑛=θ+Δθ
Initial Conditions: x=1.5m , y=2m  , 𝜃=𝜋/2 rad  ,l=0.5m
Commands:
𝑐1=(𝑣1=0.3 m/s,𝑣2=0.3 m/s,𝑡=3s):Since v1=v2, the robot moves straight.
c2 = the robot turns in place.
c3 =The robot turns with a radius.

#### robot's positions and orientations after executing each command:
After Command 1:
Final position: (x1,y1)=(1.5,2.9)m
Final orientation: θ1=2πrad≈1.57rad
After Command 2:
Final position: (x2,y2)=(1.5,2.9)m (No change in position)
Final orientation: 𝜃2≈1.17rad
After Command 3:
Final position: (𝑥3,𝑦3)≈(1.64,3.04)m
Final orientation: 𝜃3≈0.37 rad

we can check the correctness of our function with plots generated after each command.

### Inverse cinematic model of a diffrential driven robot
we model a function which calculates angular velocity of each wheel of epoch robot using its linear speed and angular speed , for example for the following cases :
Case 1: 
v=3m/s,ω=0.1rad/s
The robot moves in a curved trajectory. The orientation (𝜃) increases gradually over time.

<img src="https://github.com/user-attachments/assets/2d5db6f0-413c-4514-89e4-901de1d8f3c1" width = "400" >

<img src="https://github.com/user-attachments/assets/0287a5e2-a685-421a-bd45-18477dce2a07" width = "400" >

Case 2: 
v=0m/s,ω=0.5rad/s
The robot rotates in place without any translational movement, resulting in a circular path with no change in 𝑋 or 𝑌 position. The orientation (𝜃) increases more rapidly over time.

<img src="https://github.com/user-attachments/assets/ffb8106d-9911-42fd-bbae-c024aca88003" width = "400" >

<img src="https://github.com/user-attachments/assets/f2dbdc1d-59d3-4a6d-b0f4-3823cec17ecd" width = "400" >

##  guiding epoch towards the center of a circle
We want to design a controller that guides our robot towards the center of a circle with a random heading degree .The robot starts at a certain point on the circle's perimeter, and the circle has a radius of 0.5 meters. We need to consider different initial positions of the robot and simulate its movement towards the center using the designed controller.
A sample plot initial position = 10:

<img src="https://github.com/user-attachments/assets/1031ee99-f29d-461b-b170-342f5723f088" width = "400" >

## Collosion Detection
In this section we have a simple forward movement but its toward a box which we aim to measure the input force to our robot using TouchSensor

<img src="https://github.com/user-attachments/assets/a8afa5b9-1ad8-4ea9-aedc-02ac9fc703ed" width = "400" >


## Calculate the robot's path using PositionSensor
calculate the robot's path using three different sensors: GPS, Compass, and PositionSensor. For each case,  we follow these steps:

    Use Compass and the GPS sensor to determine the robot's path. Plot the robot's position (X-Y) and its orientation angle (θ) over time.
    Use only the PositionSensor to determine the robot's path. Plot the X-Y position and the orientation angle (θ) over time.
    Finally, compare the plots generated by each method. You should analyze how similar or different the paths are when using different sensors.

For each scenario, consider the following angular velocity profiles for the robot's wheels:

    ϕ1˙=1 rad/s,ϕ2˙=1 rad/sϕ1​˙​=1 rad/s,ϕ2​˙​=1 rad/s
    ϕ1˙=1 rad/s,ϕ2˙=−1 rad/sϕ1​˙​=1 rad/s,ϕ2​˙​=−1 rad/s
    ϕ1˙(t)=sin⁡(t) rad/s,ϕ2˙(t)=−cos⁡(t) rad/sϕ1​˙​(t)=sin(t) rad/s,ϕ2​˙​(t)=−cos(t) rad/s

The term "time step tt" refers to the simulation's time intervals. You can check the plots under related directory plot directory for each three of the scenario's.

## Split and Merge
In this section we shall create a map of the environment shown below using Split and Merge algorithm as explained :

<img src="https://github.com/user-attachments/assets/0450f6d9-9ae1-416d-80eb-92217cb6be3e" width = "400" >


Environment:

<img src="https://github.com/user-attachments/assets/ad7000c4-b9a6-4958-a271-e5a1a04bbe60" width = "400" >


Gathered maps :


<img src="https://github.com/user-attachments/assets/b86434be-f478-4def-8a1c-68a2a3fd5ea5" width = "400" >


<img src="https://github.com/user-attachments/assets/94b46f31-de9f-4282-aaef-da7c8b455709" width = "400" >


<img src="https://github.com/user-attachments/assets/d7e26ca7-ffbd-4595-bf9d-95ed43ab07d3" width = "400" >


# Bug Algorithms in a Maze
We shall implement Bug1 , Bug2 and wall following algorithm for guiding an epoch rebot through a maze to a target position.


<img src="https://github.com/user-attachments/assets/fdbef732-7f0d-437a-9ad8-a44e5469da70" width = "400" >

<img src="https://github.com/user-attachments/assets/ed615e48-3640-433d-b337-4eb0f6675363" width = "400" >

## Bug 1 and Map Gathering
Bug 1 is implemented mostly similar to bug2 but with a few number of differences  :
1. **Exploration Mechanism:**
   - **Bug1:** The robot uses a comprehensive exploration strategy, meaning it will extensively search and explore the environment to find its way.
   - **Bug2:** The robot makes decisions based on proximity to the target, which can limit unnecessary exploration.

2. **Chase Goal:**
   - **Bug1:** The robot doesn't necessarily chase the goal directly but instead focuses on a strategy to avoid obstacles and explore the environment.
   - **Bug2:** The robot continuously checks if it can move directly towards the goal along the M-line (a direct line from start to goal). If it can, it will resume moving towards the goal.

3. **Path Comparison:**
   - **Bug1:** The robot may take a more complex and longer path due to its exploration-focused approach.
   - **Bug2:** The robot tends to follow a more direct path, with less deviation, as it tries to stick to the M-line whenever possible.

In summary, **Bug2** is generally more efficient than **Bug1** in environments with obstacles because it tries to minimize unnecessary exploration and follows a more direct path towards the goal.

<img src="https://github.com/user-attachments/assets/cb769ad6-d03f-43ce-9302-a3e9c2afa854" width = "400" >


Using Distance Sensor with Threshold 0.05 for Gathering a Map of Maze:
The distance sensor is used to detect obstacles. The threshold of 0.05 implies that when the sensor detects an object closer than this distance, it triggers a response (e.g., the robot considers this a boundary or obstacle).
In essence, the robot uses the Bug1 algorithm for navigation while applying the Split and Merge method to process sensor data, particularly from a distance sensor, to identify and manage obstacles effectively. The threshold helps in distinguishing when an obstacle is close enough to require the robot to adjust its path.

<img src="https://github.com/user-attachments/assets/085b3d90-7ef0-452e-849c-00daab557eb3" width = "400" >

## Bug 2
This code's algorithm is designed to guide a robot using the "wall-following" approach. This method is commonly used in environments with obstacles and mazes to prevent the robot from colliding with obstacles and to help it find a path around them. The algorithm operates in several main states:

1. Initial State:
   The robot starts at an initial point with coordinates `Init_x` and `Init_y` and aims to reach a target point with coordinates `Goal_x` and `Goal_y`. To achieve this goal, a line called the "M-line" is drawn from the initial point to the target point. The robot's goal is to follow this line as long as it doesn't encounter any obstacles.

2. Calculating the M-Line:
   At the start of the code, the slope and bias of this line are calculated using the equation \( y = ax + b \). These values are then used to determine whether the robot is on this line or not. Due to sensor errors, it is nearly impossible for the robot to be exactly on this line, so a threshold (`thresh`) is defined. If the computed value for the robot's position falls within this threshold, the robot is considered to be on the line.

   The function used for this calculation is:
   ```python
   def M_Line(point, thresh=10**(-3)):
       global slope, bias
       result = abs(slope * point[0] - point[1] + bias) / sqrt(slope**2 + 1)
       return result <= thresh
   ```

3.Reading Sensor Values:
   This problem uses a set of sensors, including GPS, Compass, and 5 Distance Sensors. The 5 sensors provide distance values such that:
   - If there is no obstacle in the sensor's direction, the sensor value is 1.0.
   - As the robot gets closer to an obstacle, the sensor's value decreases, reaching near zero upon collision. To prevent the robot from hitting an obstacle, sensor values are continuously monitored, and if one of them falls below a threshold (set to 0.035), the robot begins to turn.

4. Determining the Wall State:
   While moving, several states related to the walls may occur:
   - **Losing_wall:** If the robot is moving away from the walls, this is detected through sensor values, and the robot attempts to reorient towards the walls.
   - **Boundary_wall:** If there are walls around the robot, it continues moving along them and may transition to other states.
   - **On_Mline:** If it is detected that the robot is on the M-line and there are no walls within the threshold distance ahead, it continues moving in that direction until it gets too close to a wall.
   - In the final state, the robot detects a wall closer than the threshold and starts turning to avoid the obstacle.

Considering the overall BUG2 algorithm, the robot may traverse around the maze multiple times, eventually reaching its goal by following the M-line. The implemented algorithm includes a mechanism for making turning decisions that reduce exploration but increase exploitation. This mechanism records the robot's last turn direction and chooses to turn in the opposite direction the next time it needs to avoid an obstacle.

<img src="https://github.com/user-attachments/assets/32a42d7c-6fa6-4bec-9ef9-947e7ab051c9" width = "400" >


## Wall Following
This aproach is mostly similar to bug2 but the only priority for our robot is to stay close to wall. The path taken with this algorithm is shown below :

<img src="https://github.com/user-attachments/assets/4d53db20-f006-43e0-a37d-2098bf923b08" width = "400" >
