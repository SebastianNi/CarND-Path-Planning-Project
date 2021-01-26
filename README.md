# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

The result of the path planner can be seen in this [video](https://youtu.be/FCqII9czg8I).

<img src="path_planning.png" width="900" alt="Combined Image" />

---

### Simulator. You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Rubric Points and Model Documentation

### The car is able to drive at least 4.32 miles without incident
The car drives around the whole track by itself just using the path planning model. The result can be watched in this [video](https://drive.google.com/open?id=0B0agIiDyIPj1QnVCeDNETWRXREU).

### The car drives according to the speed limit
The reference velocity in this model is set to 49.5 mph. The velocity is limited to the reference velocity but can be decreased when the car get stuck in traffic without a safe option to change to a faster lane. The function GetOptimalVelocity handles the speed of the car and takes care of the speed limit.

### Max Acceleration and Jerk are not Exceeded
The function GetOptimalVelocity takes care of the acceleration and the deceleration of the car. It accelerates or decelerates the car by maximal 0.45 mph per timestep. A spline function, which passes through waypoints with distances of 30 meters between them, makes sure the maximal jerk it not exceeded.

### Car does not have collisions
The car changes the lane if a car appears in front of it. If it does not find a safe way to change the lane, it starts to slow down to a little lower velocity then the car in front of it. Since the car in front of our car might slow down at any time, it is good to gain some safe distance.

### The car stays in its lane, except for the time between changing lanes
The map has 181 waypoints which lead along the middle of the road. The only interesting lanes for our car are situated in the right side of the waypoints and each lane is 4 meters wide. The car follows the waypoints of the map around the track with an offset of either 2m, 6m or 10m to the right. The math is done by converting the Cartesian (x,y) coordinates to Frenet (s,d) coordinates. The s value describes the way along the center of the road and the d value describes the offset to the roads center.

### The car is able to change lanes
The car tries to drive in the lane with the lowest traffic. It will start to search for the optimal lane as soon as another car in the same lane appears less than 100 meters before our car. The car will stay in its current lane if the other lanes do not have less traffic. If a lane with less traffic exists, however, the car will check if the lane next to its position is safe to drive. If that is the case, it will switch the lane. A lane is considered safe if no other car is driving about ten meters or less behind or about 30 meters or less before our car's position.

### Reflection on how to generate paths
The path generation starts with defining a reference point to start (or go on) with the path. This reference point is either the cars position (if no previous path exists) or the end of the previous path. Together with the predecessor of the reference point, the angle for the current heading of the path is calculated. Three additional points are added to the trajectory with 30 meters distance between each of them. Then the points are shifted and rotated so that the reference point lies in the center of the coordinate system and the tangent through the reference point leads along the x axis. After that, a spline which leads through all five points is calculated to interpolate all necessary points for the trajectory. The path should consist of 50 points. All previous path points are kept and added to the new path. The rest of the path is filled up by new points from the spline. In the last step, the path points are shifted and rotated back to the original coordinate system.

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
