# Particle Swarm Optimization based Target Search
Over the years, multiple algorithms have been put forward and have proven to be viable for the path planning of each of the nodes in these multi-robot systems, in an efficient manner given various constraints, and while allowing for decentralized communication mechanisms, and balancing efficiency, exploration, and exploitation with varying number of robots. Some examples include Genetic algorithms, Bee Colony Optimization, Differen-tial Evolution, Gravitational Search, Bat algorithm, and particle swarm optimization.

## Algorithm 
By definition, particle swarm optimization (PSO) is a computational methodthat optimizes a problem by iteratively trying to improve a candidate solution
with regard to a given measure of quality. This algorithm traditionally finds usagein mathematical optimization problems, where the global minimum of a function is located by multiple nodes, and hence can be extrapolated as an ideal solution to multi-robot target searching in unknown environments. 
The objective is to find a global best particle based on a distance based heuristic and accordingly assign velocities to all bots in the swarm based on the coordinates of the global best particle's position based on the following equation.

![equation](https://latex.codecogs.com/svg.image?v_i^{t&plus;1}=wv_i^t&plus;c_1r_1(pB_i^t-x_i^t)&plus;c_2r_2(gB_i^t-x_i^t))


where *pB* is the personal best coordinate for each particle and *gB* is the global best coordinate of the system. This process is iterated until we reach sufficiently close to our desired target.

## Implementation
This package utilizes multiple particle (robot) nodes communicating with a single master (ROS Master) server witht the help of ROS parameters to find the global best particle and to assign respective trajectories to the bots based on the above equation.

## Prerquisites
- ROS Melodic
- Ubuntu 18.04

## Usage
Clone the repository in your workspace and build the packages
```
cd ~/ws/src/
git clone https://github.com/Harshavardhan341/pso_planner
cd ~/ws
catkin_make 
```
To run the simulation:

```
roslaunch pso_planner multi_turtlebot.launch nr:= 5

```
Publish the desired goal on the /goal topic 






