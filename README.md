# Projects in Modern Robotics Specialization
**Sadha Sivam M
19ME10055**

## 1. Forward Dynamics Simulation - Free fall of a UR5 robotic arm




A MATLAB code that builds on the Modern Robotics library to simulate the motion of the robot and visualize the resulting motion of the robot by using CoppeliaSim. The robot used for simulation was UR5, a popular 6-DOF industrial robot arm.  The robot has geared motors at each joint, but in this project, the effects of gearing, such as friction and the increased apparent inertia of the rotor was ignored.  

The code simulates the motion of the UR5 for a specified amount of time (in seconds), from a specified initial configuration (at zero velocity), when zero torques are applied to the joints.  In other words, the robot simply falls in gravity. Gravity is g = 9.81g=9.81m/s^2 in vertically downward direction(-z axis), i.e., gravity acts downward.   The motion was simulated with at a number of integration steps per second.  The program calculates and records the robot joint angles at each step. This data will be saved as a .csv file, where each row has six numbers separated by commas. This .csv file was used for animation with the CoppeliaSim UR5 csv animation scene.

<img width="228" alt="455px-Ur5-img" src="https://user-images.githubusercontent.com/81227060/123660359-36e72280-d851-11eb-85bb-8ea039a4d07b.png">

The video of the simulation carried out can be found in the corresponding folder in the repository.

## 2. Motion planning using Probablistic Roadmap(PRM) and A* Search

 The program uses PRM to find a path for a point robot moving in a plane among the obstacles. The coordinates and size of the obstacles are described in a csv file and is used as input to the program. 
 
 Program chooses its own random samples in the (x, y) C-space, which is the square [-0.5, 0.5] x [-0.5, 0.5]. It also employs a straight-line planner as the local planner between any two sample points.The start configuration is at (x, y) = (-0.5, -0.5), the bottom left corner of the square environment, and the goal configuration is at (x, y) = (0.5, 0.5), the top right corner of the square environment. Ther program also accounts for collision checking. Given a straight line segment from (x1, y1) to (x2, y2), the collision checker in the program checks to see if the line segment intersects a circle (all the obstacles are circles). The output of the program is the three files: nodes.csv, edges.csv, and path.csv. 
 
 Finally, the program uses A* search algorithm to find the path with shortest distance.
 
 ![simulation (1)](https://user-images.githubusercontent.com/81227060/123668398-b1677080-d858-11eb-8fb1-9d6591f68de2.png)
 
 
## 3. Evaluating Form Closure

A rigid body is said to be in form closure grasp if the object motion is geometrically constrained in all directions by the contacts in its surroundings. The program determines if a planar rigid body, subject to a specified set of stationary point contacts, is in form closure.

The program will take as input a list of stationary point contacts on the body, each specified by the (x,y) contact location and the direction of the contact normal. The program uses a linear programming method to achieve the objective. A function called 'linprog' in MATLAB is used in this program. The output of your program will be binary: the body is either in form closure or not in form closure.

![aadsd](https://user-images.githubusercontent.com/81227060/123670468-dfe64b00-d85a-11eb-9221-a859a1d25185.png)

## 4. Determining Stability of an Assembly

Given a set of frictional contacts acting on a body, it is in force closure if the positive span of the wrench cones is the entire wrench space. Another way of saying this is that the contacts can theoretically resist any wrench(combination of forces and moments) applied to the body.The program determines whether a assembly of rigid bodies in a given configuration is stable or collapsing by determining force closure for each individual rigid body.

Your program will take as input: a description of the static mass properties of each of the N bodies: the (x,y) location of the center of mass and the total mass; a description of the contacts - the (x,y) location of the contact; the contact normal direction into the first body involved in the contact and and the friction coefficient μ at the contact.The output of your program will be binary: it is either possible or impossible for the assembly to remain standing.

## 5. Mobile Manipulation using KUKA YouBot

The programs put together makes a software: 
- Plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm)
- Performs odometry as the chassis moves 
- Performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down.

The final output of the software will be a comma-separated values (csv) text file that specifies the configurations of the chassis and the arm, the angles of the four wheels, and the state of the gripper (open or closed) as a function of time. This specification of the position-controlled youBot will then be "played" on the CoppeliaSim simulator.

![640px-Youbot-capstonebig](https://user-images.githubusercontent.com/81227060/123673510-2d17ec00-d85e-11eb-902c-c1c5ea8508ad.png)

The complete simulation video can be found in the corresponding folder in the repository.
