# ecse373_f21_group8_lab6
[Given Simulation](#the-given-simulation)

[Launch Everything](#launching-everything)

[Launch only Gazebo](#launching-only-gazebo)

[Starts up the competition](#Starts-up-the-competition)

[Find location](#Find-location)

[Action Server](#Action-Server)

# The Given Simulation
<p> In this README, we refer to a given simulation (this is what is used in the code as well) which can be cloned from: https://github.com/cwru-eecs-373/ecse_373_ariac.git
<p> This package is a modified version of that used for the Ariac 2019 competition - it provides an inverse kinematics library as well customizations to adapt the competition to this course.

___

# Launching Everything

<p> Launching everything means launching the given Ariac simulation at as well as starting (or trying to start) the competition, receving and managing orders, and finding object locations.

To launch everything you use:
```
roslaunch ariac2019_package araic.launch
```
___
  
# Launching Only Gazebo
  
<p> This only launches the given Ariac simulation.
 
To launch only ecse use:

```
roslaunch ecse_373_ariac ecse_373_ariac.launch
```
___
  
# Starts up the competition
  
The node will display if the service was called successfully.


Shown in the picture as


Competition service called successfully: competition started successfully!

___
  
# Find location
When we use cheating methods to find the location of the part, we can get the location of the part directly without traversing 10 cameras.
We can know position and orientation.


Shown in the picture as


position: x = 0.545921, y = 0.156182, z = 0.017853
orientation: x = -0.338098, y = -0.619911, z = 0.328681, w =0.627192
___
  
# Action Server

The Action Sever implementation provides access to ongoing results reporting and completion feedback.

Shown in the picture as
 

Action Server returned with status: [6] SUCCEEDED
___
  
# Picture
![image](https://github.com/yxs1090/ariac2019_package/blob/main/README/sendpix0.jpg)
