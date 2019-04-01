# IRL with dynamical system

Includes mouse_perturbation ROS package, the marker package, and the IRL matlab code.

## Getting Started
remember the ros program in this folder is not the updated one, the updated one is in 


### Prerequisites



```
Give examples
```

### Files

#### FOLDER: mouse\_perturbation_robot 			

It includes c++ code, experiment protocol, modulation dynamical system.

##### MotionGenerator

the constant to be specified: numObstacle (1 or 2). 

##### MouseInterface
Declare mouseMessage and publish it to the motionGenerator (in my PC). If we use the code in KUKA robot-arm PC, then the mouse data is published by another node "spacenav".



#### FOLDER: obstacle\_with\_sample\_from_dynamics 		
matlab code, experiment IRL node.

### Usage of ROS code




### Usage of MATLAB code

launch MATLAB  
`$cd /usr/local/MATLAB/R20XXx/bin/`  
`$./matlab`  
addpaths.m

## Running the codes

`$cd catkin_ws/`  
`$catkin_make clean`  
`$catkin_make`  
`$source devel/setup.bash`

`$roscore`  

`$roslaunch lwr_simple_example sim.launch` OR
`$roslaunch lwr_simple_example real.launch`

`$roslaunch lwr_fri lwr_fri_console` typye control inside and starting working with the robotarm

`$roslaunch spacenav_node classic.launch`
`$rostopic echo -c /spacenav/joy`

`$rosrun using_markers basic_shapes`
`$rosrun using_markers arrow`

`$rosrun motion_example moveToDesiredJoints 42 45 0 -75 60 -45 -45` - this is the horizontal end effector orientation.

`$rosrun motion_example moveToDesiredJoints 42 45 0 -75 0 55 0`
downward end effector 

`$rosrun rqt_reconfigure rqt_reconfigure`

`$rosrun mouse_perturbation_robot mouseInterface /dev/input/event#`

`$rosrun mouse_perturbation_robot motionGenerator`

Check the 3D mouse   
`$ls /dev/input`
`$sudo su`

Check messages
`$rostopic echo -c /lwr/ee_pose`

`$rosrun rqt_graph rqt_graph`

Parametere setting in the robotarm PC:  
damping eigen - 90/1000  
damping eigen - 90/500  
rot stiff - 11/50  
rot damping - 2/50  

### code procedure

start the ROS core



start the mouse interface node

start the motion generator node


### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```


## Deployment

Add additional notes about how to deploy this on a live system
