# Project Challenge

The Project Challenge asked for a simulation that efficiently moved from a start location to an end location on a grid while avoiding obstacles. I utilized an RRT* planner and python's matplotlib library for visual aid.

## Overview

The simulation consists of a robot and an environment. The environment is populated with obstacles that may or may not be initially seen by the robot. The robot has a "sight distance" such that it can only detect obstacles within a certain range of it's own location. Once it sees the obstacle, it remembers where the obstacle is on the map. This is an extreme simplification of a SLAM algorithm for obstacle detection that would be implemented in a physical system. The robot, assuming the only obstacles are the ones it has discovered, utilizes an RRT* path planner to find an optimal path to it's goal (a region represented by a green box). Once a path is found, it moves through the environment to the goal. If a new obstacle is detected and the new obstacle is in the way of the robot, it will pause and perform another search for a path to the goal. This is repeated until the robot has reached the goal region.

## Assumptions

* Obstacles are represented as "expanded" objects (allowing the robot to be a point with no radius)
* The environment does not change (bounds and dimensions are static)
* Obstacles are entered in the correct order (such that they are convex objects, though this is not necessary)
* Each obstacle contains at least two points (a line/boundary)
* Neither the start location nor the entirety of the goal location are within obstacles

## Motivations/Design Choices

RRT* was chosen for it's optimality. It is a sampling-based algorithm that works well when a map is known a priori. Because I was restricted to basic libraries, I created my own classes for Regions and Obstacles and my own function for collision detection. This would have been easier with a library like Shapely, but felt too far beyond the basic libraries. As a result, the "goal detection" is simply whether a new path collides with the boundaries of the goal region. I chose to have obstacles to be hidden until the robot is close enough to balance having a known map a priori and detecting new obstacles during the actual navigation. 

## Next Steps

Ideally, the robot would not pause when it sees a new object and get rid of all the path planning it has already done. Additionally, RRT* is only optimal when it runs continuously. As a result, it would have been fantastic to keep the path planning running while the robot moves as opposed to the two tasks being mutually exclusive. This could allow for the path to be optimized while the robot moves and for the path to change in the face of a new obstacle with minimal disruption.

## Files

* searchNode - Node Class
* region - Region and Obstacle Classes
* environment - Environment Class (allows for parsing of YAML files)
* collision_check - Function to check if two lines intersect
* simulation - Simulation Class and Visual Animation

## To Use

Keep all files in the same folder and run the simulation.py file from the root. 

## Built With

* [numpy](http://www.numpy.org/)
* [matplotlib](https://matplotlib.org/)
* YAML - To import obstacles quickly
* Random

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments/Sources

* http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
* [Sampling-based Algorithms for Optimal Motion Planning](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=0ahUKEwiQ4eKB8KDYAhXI5oMKHbRpD_wQFggtMAA&url=https%3A%2F%2Farxiv.org%2Fabs%2F1105.1186&usg=AOvVaw0R0asdosqqUrFXzpDvv1IS)