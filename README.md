# Particle Swarm Optimization based Target Search
Over the years, multiple algorithms have been put forward and have proven to be viable for the path planning of each of the nodes in these multi-robot systems, in an efficient manner given various constraints, and while allowing for decentralized communication mechanisms, and balancing efficiency, exploration, and exploitation with varying number of robots. Some examples include Genetic algorithms, Bee Colony Optimization, Differen-tial Evolution, Gravitational Search, Bat algorithm, and particle swarm optimization.

## Algorithm 
By definition, particle swarm optimization (PSO) is a computational methodthat optimizes a problem by iteratively trying to improve a candidate solution
with regard to a given measure of quality. This algorithm traditionally finds usagein mathematical optimization problems, where the global minimum of a function is located by multiple nodes, and hence can be extrapolated as an ideal solution to multi-robot target searching in unknown environments. 
The objective is to find a global best particle based on a distance based heuristic and accordingly assign trajectories to other bots in the swarm based on the coordinates of the global best particle's position based on the following equation.
![equation](http://www.sciweavers.org/tex2img.php?eq=v_i%5E%7Bt%2B1%7D%3Dwv_i%5Et%2Bc_1r_1%28pB_i%5Et-x_i%5Et%29%2Bc_2r_2%28gB_i%5Et-x_i%5Et%29&bc=Transparent&fc=White&im=jpg&fs=12&ff=arev&edit=0)

## Implementation
This package utilizes multiple particle (robot) nodes communicating with a single master (ROS Master) server witht the help of ROS parameters to find the global best particle and to assign respective trajectories to the bots based on the above equation.
