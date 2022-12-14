# MultiAgent_AStar_WarehouseRobot
A C++ implantation for Multi-Agent A* Algorithm for Multi Agent Path planning.

# Overview
This is the capstone project for the Udacity C++ Nanodegree Program. In this project, I implemented a multi-agent A* path planner and an example application of the algorithm in an order fulfillment center. The puzzle is as follows;

In a warehouse, A fleet of robots has to fulfill a pool of tasks, the problem is solved when all tasks are fulfilled.
The robots should not collide with each other nor should they collide with an obstacle in the warehouse while navigating to their task. According to [1] "A collision occurs if two agents occupy the same location at the same timestep (called a vertex conflict) or traverse the same edge in opposite directions at the same timestep (called a swapping conflict)"

### Assumptions have been made to simplify the problem

- The space (the map of the warehouse) is discretized into equal size cells
- Time is discretized into time steps. At each time step, a robot can move to a neighboring cell on the map
- All robots should move at the same speed


### Dependencies for Running Locally
cmake >= 2.8
All OSes: click here for installation instructions
make >= 4.1 (Linux, Mac)
Linux: make is installed by default on most Linux distros
OpenCV >= 4.1
The OpenCV 4.1.0 source code can be found here
gcc/g++ >= 5.4
Linux: gcc / g++ is installed by default on most Linux distros

### Compile and build the project
mkdir build
cd build 
cmake ..
make

### Collision Tests
To validate if the algorithm is avoiding collisions two tests have been developed.To run the tests

 ./test/collisionTest 
 
and the console will show that the tests are passing

[==========] 2 tests from 1 test suite ran. (16328 ms total)
[  PASSED  ] 2 tests.

The first test examines what is called a swapping conflict, which is when two robots swap their locations. Or as expressed mathematically in [2]

Pa(t) = Pb (t+1) AND Pb(t) = Pa (t+1)

where a and b are distinct agents.

This collision case can be simulated if we comment out line 68 in Planner.hpp

//&& !neighbor->isReserverd(currentCell->getTimeStamp())

build the project and run the tests

./test/collisionTest 

and the console will show that the test is failing

[==========] 2 tests from 1 test suite ran. (14300 ms total)
[  PASSED  ] 1 test.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] CollisionTest.SwappingCellsTest

Now uncomment line 68 in Planner.hpp

&& !neighbor->isReserverd(currentCell->getTimeStamp())

build the project and run the tests


The second test examines what is called a vertex conflict, which is when two robots occupy the same location. Or as expressed mathematically in [2]

Pa(t) = Pb (t)

This collision case can be simulated if we comment out line 67 in Planner.hpp

//&& !neighbor->isReserverd(NexttimeStamp)

build the project and run the tests

and the console will show that the second test is failing

 [==========] 2 tests from 1 test suite ran. (14278 ms total)
 [  PASSED  ] 1 test.
 [  FAILED  ] 1 test, listed below:
 [  FAILED  ] CollisionTest.OccupyingSameCell
 
Now uncomment line 68 in Planner.hpp

 && !neighbor->isReserverd(NexttimeStamp)
 
build the project and run the tests


## Warehouse Demo
Now that we have validated the Multi-Agent A* planner, we can utilize it in an order fulfillment center scenario, where we have a queue of tasks and a queue of robots, and the demo shows how the robots can fulfill all the tasks without colliding with each other or with the shelves in the warehouse. run the demo

 cd build
 ./MAA-Star 
You should be able to see this on the screen below (this gif is 3X the actual speed, the robots in the demo are moving at a velocity of 1 cell/sec)

### code structure
 CellData.hpp/CellData.cpp CellData is a struct that represents the cells of the grid of the warehouse, and it stores the information about the cell that the planner needs such as the index, Cartesian position, and the value of the cell {emptey,occupied,delivery,pickup}
Map.hpp The Map Class is composed of a vector of Objects of type CellData
 Warehouse.hpp/Warehouse.cpp The Warehouse is a Class that is used to generates the map.
Robot.hpp The Robots (the colored circles) are objects of type Robot (class), each object owns a shared pointer to the cellData object that the robot is parking at. In addition, it owns a queue of shared pointers of DataCell objects that represents the planned path.
Planner.hpp The multiAgentPlanner is an object of type planner (class). It needs a robot instance, task and pointer to the Map to plan a path for the robot.
GenericQueue.hpp The robots in the demo are circulating between the planning thread and the executing thread until all tasks in the queue are fulfilled. The planning thread blocks until a robot is available in the queue of the available robots, this queue is of type GenericQueue.
 Graphics.hpp/Graphics.cpp The viewer is an object of type Graphics (class). This viewer owns a shared pointer to the map and to all the instances of Robot Class. The viewer runs in parallel in the simulation thread.
