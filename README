Author:
Menglong Zhu, menglong AT cis upenn edu

DESCRIPTION:
Path planning package. Two approaches are implemented in this package
1. uniform grid over map
2. quadtree decomposition over the map
Planning algorithm is A* search

video of quadtree decomposition:
http://www.youtube.com/watch?v=jqjlsIAQDT0

visualization use ROS markers in rviz

COMPILATION:
Cmake file is provided.
$ cmake .
$ make

Only the visualization part depends on ROS rviz package. Other part of the code are stand alone. If you don't have ROS installed, simply comment out the visualization part of the code in run_planner.cpp.

HOW TO RUN:
./run_planner <map path> 
which generate intermidiate text file for output.

NOTE:
This implementaion assume obstacles are all circles
