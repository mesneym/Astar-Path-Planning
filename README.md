# A*-algorithm implementation for Rigid and Point Robot
### Description
The aim of this project is to allow a point or rigid robot to move in an environment from some specified start location to a goal location through some optimal path accounted for by the use of A\* algorithm. A brief diagram of depicting the problem is shown below
![](./Data/problem.png)


### Point Vs Rigid Robot
A point robot is a theoretical construct and has no dimensions. To somewhat account for this, we create a 2D rigid robot which  depicts  a circle with some given radius. To make the robot safe whilst navigating, we specify a parameter(clearance) which indicates 
a safety distance, a measure of how far a robot can be close to an obstacle without compromising its safety.

![](./Data/pointvsrigid.png)
<p align="center">
  <img width="300" height="300" src="./Data/clearance.png">
</p>


### Dependencies 
1. python -version 3
2. pygame


### Libraries used
Numpy

### Run Code
Enter the following to run the dijkstra for point robot.

```
cd [to 'codes' directory]
python3 astar_point_rigid.py
```

### Input Instruction:
As soon as you run the program, the following prompt occurs in the command window:
```
1 -> point robot 
 2 -> rigid robot
 Enter number :
2
Enter cleareance
1
Enter radius
1
Enter step size of the robot
1
Enter start location s1 - (X-coordinate of start node)
50
Enter start location s2 - (Y-coordinate of start node)
30
Enter goal location g1 - (X-coordinate of goal node)
150
Enter goal location g2 - (Y-coordinate of goal node)
150
```



### Sample output for rigid robot:
After running the python file
```
Cost took to reach the goal is: 395.64675298172733
Backtracking...
Total time taken 128.9800910949707
```


### Sample video output
![Alt Text](./Data/video1.gif)

For a step size of 5 we have the following
![Alt Text](./Data/video2.gif)




