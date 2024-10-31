# Controllers for GCTronic' e-puck in webots
For better understanding of the codes, please first check webots documents regarding GCTronic' e-puck

## Basic Controllers
In the first section we shall implement a series of basic controllers for e-puck.
### Applying diffrent Angular velocity
In order to better understand the concepts of e-puck movement we code a simple controller in which by changing values of parameters ' max_speed_left ' and ' max_speed_right ' we can access and adjust angular velocity of each wheel. we use ' GPS ' and ' Compass ' sensors to plot the movement of our robot. x-y of robot movement and time-teta plot of robot heading in each time plot is avaiable for 3 diffrent set of angular velocity values :

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

##  guiding GCTronic' e-puck towards the center of a circle
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

