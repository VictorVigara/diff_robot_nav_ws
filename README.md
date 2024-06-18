# Custom differential robot navigation 
Workspace containing the packages to perform autonomous navigation given waypoints in an indoor known environment. 

# Structure

## custom_diff_robot
Package containing the description of a custom differential robot. It also contains the launch files to spawn the robot into gazebo, as well as the launch files to perform autonomous navigation: 
- gmapping: To create the indoor map (Mapping)
- amcl : Takes laser scan and transform mmessages and outputs pose estimates (Localization)
- move_base: Parto f the ros navigation stack that given tf, odom,  map and laser scan cacluates the local and global plan and outputs velocity commands to reach the goals. 

## ira_lase_tools
Package  used to merge front and rear scans needed for amcl. 

## goal action
Custom action package that contains a goal action: 

```
#goal definition
geometry_msgs/Pose[] waypoints  # List of waypoint
---
#result definition
std_msgs/String result  # OK, ERROR, ...
---
#feedback
float32 progress    # Percentage of waypoints reached
float32 distance_next_waypoint  # Distane to next waypoint
```

It also contains: 
- goal_server.py: Receives a list of waypoints from a client and send the waypoints to the move base server. 
- goal_client.py: Send the waypointsto the goal_server and receives the feedback of the navigation. 

# USAGE

## Launch robot model into gazebo
```
roslaunch custom_diff_robot robot_model.launch
```

## Launch navigation stack
```
roslaunch custom_diff_robot navigation.launch
```

## Launch goal action server

```
 goal_action goal_server.py
```

## Launch goal action client
```
rosrun goal_action goal_client.py
```
