# IRL with dynamical system

## Getting Started

### Prerequisites



```
Give examples
```

### Files

#### mouse\_perturbation_robot 			

It includes c++ code, experiment protocol, modulation dynamical system.

##### MotionGenerator

the constant to be specified: numObstacle (1 or 2). 

##### MouseInterface
declare mouseMessage and publish it to the motionGenerator (in my PC). If we use the code in KUKA robot-arm PC, then the mouse data is published by another node "spacenav".



#### obstacle\_with\_sample\_from_dynamics 		
matlab code, experiment irl node.

### Usage of ROS code




### Usage of MATLAB code

launch MATLAB $cd /usr/local/MATLAB/R20XXx/bin/
$./matlab


addpaths.m


## Running the codes

Explain how to run the automated tests for this system

Parametere setting in the robotarm PC:
90/1000
90/500
11/50
2/50

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
