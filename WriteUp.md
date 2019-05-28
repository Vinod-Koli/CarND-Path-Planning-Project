
# CarND Highway Driving Project

## 1. Goal

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## 2. Rubic Points

## Code & Compilation

The code is written in C++ and uses spline library to interpolate the points on the trajectory. The code compiles without any errors.

No changes were required to make in `CMakeLists.txt`.


## Trajectories

#### The car is able to drive at least 4.32 miles without incident
The car was run for more than 5 miles in the simulator without any errors.

#### The car drives according to the speed limit
The car drives at maximum approximate speed of 49.5 mph. This is taken care at line number 73 in `main.cpp`.

#### Max Acceleration and Jerk are not Exceeded
The maximum acceleration is set to 5 m/s^2, also the jerk is below the specified value of 10 m/s^3.

#### Car does not have collisions
The car is able to avoid collisions with the other cars. The car tries to change lane whenever there is a car ahead of it moving with slower speed. If there is no room for lane change then the car slows down to safe speed.

#### The car stays in its lane, except for the time between changing lanes
The car stays in its lane unless there is need to change lane and if it is safe.

#### The car is able to change lanes
The car can change between the lanes without colliding with other vehicles whenever needed.

## 3. Reflection

The model for generating trajectories is based on the techniques provided in the classroom. The model can be divided into following parts

## Sensor Fusion Data Analysis adn Prediction

The data provided by sensor fusion is analysed to detect any cars in the near vicinity of the vehicle. This is implemented at line number 116 to 150 in `main.cpp`. If any vechicle is within 30m from the car then this car is considered for the decision making. The `s` and `d` co-ordinates of the vehicles are very useful in this step.

## Behaviour Planning
The behaviour of the vehicle is planned based on the presence of vehicles in wihtin next 30m of road in the current and the adjacent lanes. Code snippet used for planning behaviour is shown below.

Lane planning
```
          bool too_close = false;
          if(car_ahead == true)
          {        
            if((car_left == false) && (my_lane != 2))
            {
              // Change lane to the left
              my_lane += 1;
            }
            else if((car_right == false) && (my_lane != 0))
            {
              // change lane to the right
              my_lane -= 1;
            }
            else
            {
              // if no room to change lane then slow down the vehicle
              too_close = true;
            }
          }
```
Speed adjustment
```
          if(too_close)
          {
              ref_speed -= 0.224; 
          }
          else if(ref_speed < 49.5)
          {
              ref_speed += 0.224;
          }
```

## Create Trajectory Path

To create a smooth path 2 points from the previous trajectory are also considered. First 3 points are set at a distance 30m, 60m and 90m from the car. These points are set based on the Lane and Speed desicions made in the behaviour planning step.

The spline library is then used to interpolate the points between the widely spaced 30m, 60m and 90m points based on the speed.

The trajectory planning has been implemented at line numbers from 253 to 282 in `main.cpp` file.

