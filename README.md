# Controllers for epoch in webots
For better understanding of the codes, please first check webots documents regarding epoch

## Basic Controllers
In the first section we shall implement a series of basic controllers for epoch.
### Applying diffrent Angular velocity
In order to better understand the concepts of epoch movement we code a simple controller in which by changing values of parameters ' max_speed_left ' and ' max_speed_right ' we can access and adjust angular velocity of each wheel. we use ' GPS ' and ' Compass ' sensors to plot the movement of our robot. x-y of robot movement and time-teta plot of robot heading in each time plot is avaiable for 3 diffrent set of angular velocity values :
Part 1 : φ˙ 1 = 1rad/s, φ˙ 2 = 1rad/s

![Q1-tt](https://github.com/user-attachments/assets/7d633139-4592-421f-a8c5-c91d8e20c629)

![Q1-xy](https://github.com/user-attachments/assets/30fdf348-b154-4f53-902c-dfc8afb39abe)

Part 2 : φ˙ 1 = 1rad/s, φ˙ 2 = −1rad/s

![Figure_1](https://github.com/user-attachments/assets/d06a6a94-6708-4957-855a-860eef42c106)

![Figure_2](https://github.com/user-attachments/assets/f9b2b1f9-5cb1-45e6-afea-e087e2b52c88)

Part 3 : φ˙ 1(t) = sin t rad/s, φ˙ 2(t) = −cos t rad/s

![Figure_1](https://github.com/user-attachments/assets/38af5997-e02c-405a-b45e-2e8777057047)

![Figure_2](https://github.com/user-attachments/assets/c8ca2b33-2a16-408e-bed9-290d05ee9584)


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

![Q4-1-tt](https://github.com/user-attachments/assets/2d5db6f0-413c-4514-89e4-901de1d8f3c1)

![Q4-1-xy](https://github.com/user-attachments/assets/0287a5e2-a685-421a-bd45-18477dce2a07)

Case 2: 
v=0m/s,ω=0.5rad/s
The robot rotates in place without any translational movement, resulting in a circular path with no change in 𝑋 or 𝑌 position. The orientation (𝜃) increases more rapidly over time.

![Q4-2-tt](https://github.com/user-attachments/assets/ffb8106d-9911-42fd-bbae-c024aca88003)

![Q4-2-xy](https://github.com/user-attachments/assets/f2dbdc1d-59d3-4a6d-b0f4-3823cec17ecd)

##  guiding epoch towards the center of a circle
We want to design a controller that guides our robot towards the center of a circle with a random heading degree .The robot starts at a certain point on the circle's perimeter, and the circle has a radius of 0.5 meters. We need to consider different initial positions of the robot and simulate its movement towards the center using the designed controller.
A sample plot initial position = 10:

![Figure_1](https://github.com/user-attachments/assets/1031ee99-f29d-461b-b170-342f5723f088)


