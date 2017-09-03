# Path Planning
This project is to design a path planner to navigate a car going through highway traffic. During the navigation process, the planner generates trajectories for the car to follow. Safety and efficiency is taken into account when generating such trajectories, that is, trying to maintain the speed and avoiding hitting other cars.

The planner is implemented in `src/main.cpp`(line 264-445). There are two steps to generate a path in the current implementation:

1. Decide which lane to go and in what speed
2. Genreate a trajectory that does not violate the acceleration and jerk limits

In the following we will discuss the details of the above steps and propose possible improvements for the current implementation.

### Choosing the lane and the speed
To decide which lane to take in each cycle, we define some cost functions (line 165-180). Each lane has its own cost value that starts with 0 in each cycle. We caculate the predicted positions of our own car and other cars, which are car_end_s and check_car_end_s in the Frenet coordinate. Then we add penalties to the cost value (line 289-316) if it is in one of the following situations:

* (line 291-301) A car is in the same lane in the front and the _predicted_ Frenet s distance in is less than 30m. The penalty take velocity and distance into account so that it encourage our car to change lane when the car in the front drive slower than our car does. 
* (line 305-316) There are other cars on the lane next to our car, and the _current_ Frenet s distance is within 15m. This discourages the car to change lane when other cars next to the current lane are too close.
* (line 318-326) In addition, if there is a car in the front, and the _current_ Frenet s distance is within 10m, the velocity is decreased to avoid hitting.

After calculating penalties for each lane, the lane with the smallest penalty is chosen. However, to prevent hitting other cars and exceeding the acceleration and jerk limits, the car can only change one lane at a time and it must consume 80 waypoints before it changes lane (line 330-346).

### Generating a trajectory
Here we follow the method intorduced in the project walkthrough. First of all, we collect the last two positions from the previous trajectory (line 359-385), and then we add three 30m spaced (in the Frenet coordinate) points ahead of the last position from the previous trajectory. These points are then trasformed into the vehicle's coordinate and are fittd by spline interpolation (line 387-409). Transforming the points into the vehicles coordinate help ensure that each x correponds to only one y. After the interpolation, we can generate points and then transform them back to the map coordinate. We send in 50 waypoints to the simulator in each cycle, so in this case we take the list of waypoints returned by the simulator and fill it up to 50 points (line 411-446). This completes the trajectory generation for each cycle.

### Summary

## Instruction   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

