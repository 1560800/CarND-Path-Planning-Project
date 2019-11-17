
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
[![Dataset 1](/data/path_plannning.PNG)](https://youtu.be/oQK8Zmnv2Kg)

[YouTube Link](https://youtu.be/oQK8Zmnv2Kg)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

# Project Rubric and Details of Implementation
## Project Rubric
The goal of this project is to drive the car safely on the highway with other vehicles running at different speeds. 
Self-driving cars should be able to:
1. Follow speed limit
2. Must be comfortable while driving (do not exceed maximum jerk, acceleration)
3. You need to change lanes when the vehicle ahead is driving slowly, and it is safe to change lanes
4. Should stay in that lane
5. Avoid collisions
## Reflection on Implementation
#### 1. The car will travel according to the speed limit.
* To achieve this, the maximum speed of the car was limited to 49.5MPH (speed limit is 50MPH).

#### 2. Maximum acceleration and jerk will not be exceeded.
* Solved the problem of maximum acceleration / jerk by increasing or decreasing the vehicle speed using a small value of 0.224 mph. 
* This gives a maximum acceleration of 5 / s / s.

#### 3. The car can change lane
* The speed is reduced by 0.224 mph to avoid collisions, but at the same time, we started to see the surrounding cars. 
* If it is safe to change lanes (no other cars in the 30 m range), we are the only ones to change lanes. Logic prioritized the left lane over the right lane (because the left lane is a high-speed lane).

#### 4. The car will stay in the lane except for the time between changing lanes.
* The car will stay almost in that lane until it needs to be changed (if a slower car is ahead).

#### 5. There is no collision in the car.
* Sensor fusion data was used to detect surrounding vehicles. 
* If there is a car within 30m, our car will take safety measures such as reducing the speed when other cars are ahead, and will not change the lane if there are cars in other lanes.

#### 6. Consideration regarding the path generation method
To generate the path, I used a spline library and library, as shown in the Udacity class and provided anchor points and requested a “Y value” for each “X” to get the (X, Y) coordinates of the path.
Note: UDACITY video class code was used as the base for the project. And is based largely on that

* I used the remaining previous points and added future pass points for a smooth transition.
* The car's “S” value was used to get future waypoints in the 30m, 60m, and 90m range. These points were used as spline anchor points.
* These global coordinate anchor points have been converted to car coordinates for easy calculation.
* I planed the next 30 m (1.5 seconds from now) path plan. The plan wasn't too short or too long as it may not reflect the correct situation around the car.
* First, I used speed to divide the distance into N points.
* I adjusted Y for every X using a spline.
* The X and Y coordinates of these cars have been converted to (x, y) on the map.

## Conclusions and challenges

This project was successful because it cleared all Rubric's requirements.

Future issues are as follows.
The lane change algorithm only considers the distance between the vehicles closest to each lane.
In creating an optimal path, it is necessary to recognize not only the nearest vehicle but also the positional relationship with multiple vehicles ahead and behind, and find the optimal path using a cost function.
