# Kinematic MPC and dynamic gain schedulling state feedback for controlling an autonomous vehicle

This project allows you to solve the autonomous guidance problem using advanced control theory. 
The innovative part in this project is the use of a Takagi-Sugeno (TS) representation of the kinematic vehicle model. This allows us to solve a non-linear optimization problem as a pseudo-linear one and, hence, achieving a very low elapsed time at each optimization.

### Prerequisites

For running the project you need to instal Matlab 2017b or newer versions and YALMIP. Moreover, it is necessary to install gurobi solver for performing the linear optimizations.

### Installing

For installing the packages I refer to the following links:

* [YALMIP](https://yalmip.github.io/download/)
* [GUROBI](http://www.gurobi.com/downloads/download-center?campaignid=193283256&adgroupid=8992997136&creative=203314797799&keyword=gurobi&matchtype=e&gclid=CjwKCAjwr-PYBRB8EiwALtjbzw9ozJIT_lkMjDiedWlTAO7XXB494569fFt3ZNJYy1GRJL_hSkMOyBoCRT4QAvD_BwE)

## Description

### The vehicle model
There has been used two different models. One for the kinematic control and another for the dynamic control.
The kinematic model is known as the vehicle mass point model. The dynamic one models the dynamics of the single track bicycle model and the tire model.

### The trajectory planning
We use a ploinomial based algorithm for computing the references in an offline manner. This stage provides at every instant of time the needed references to the controller.

### The Kinematic MPC
At this point a Model Predictive Controller is built and solved at every control iteration for finding the optimal control actions (linear and angular velocities). Due to the cascade scheme, these control actions will be the references of the next control loop (the dynamic control loop).

### The gain scheduling Dynamic controller
The dynamic controller has been built as a state feedback controller but using the gain schedulling methodology. First of all, the design is computed by solving a LQR-LMI problem that returns the set of controllers at every vertex of the polytope. This politope is a convex region formed by the limits of the schedulling variables (consult Takagi-Sugeno theory in the reference).

## Running the tests

For running this project you must include all the folder in the Matlab path. Then, just run 
```
DISCRETE_KinDyn_MPC_SF.m
```
At this point you are going to be asked about what algorithm to choose between three.
```
1.- DISCRETE frozen-MPC control 
2.- DISCRETE NL-MPC control 
3.- DISCRETE References-MPC control 
```
In every case the dynamic control is the same.

## Graphical results
![](https://github.com/euge2838/Autonomous_Guidance_MPC_and_LQR-LMI/blob/master/circuit.png)
![alt text](https://github.com/euge2838/Autonomous_Guidance_MPC_and_LQR-LMI/blob/master/elapsedTime.png)
![alt text](https://github.com/euge2838/Autonomous_Guidance_MPC_and_LQR-LMI/blob/master/errorsDISTURB.png)

## References
* Eugenio Alcalá, Vicenç Puig and Joseba Quevedo. TS-MPC for Autonomous Vehicles including a dynamic TS-MHE-UIO.
