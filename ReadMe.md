# Smart Wheelchair
# Dependencies
```
ros-melodic
rtabmap-ros
turtlebot3
```

# Installation
Clone the repository
```
$ mkdir catkin_ws && cd catkin_ws
$ mkdir src && cd src
$ git clone https://github.com/Nishantgoyal918/smart_wheelchair

```
Clone dependencies
```
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ git clone https://github.com/patilnabhi/nuric_wheelchair_model_02
```
Make
```
$ cd ..
$ catkin_make
```

# Running Turtlebot3 Simulation on Corridor map
Copy the ```turtlebot3_corridor.launch``` file from ```catkin_ws/src/smart_wheelchair/extras/turtlebot3_corridor.launch``` to ```catkin_ws/src/smart_wheelchair/turtlebot3_gazebo/launch/```

## Launching Turtlebot3 Gazebo Simulation
Now,
```
$ cd catkin_ws
$ source devel/setup.bash
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_corridor.launch
```

In new terminal,
```
$ cd catkin_ws
$ source devel/setup.bash
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

## Launching RTAB-Mapping
In new terminal,
```
$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Odom/ResetCountdown 1 --Rtabmap/StartNewMapOnLoopClosure true" rgb_topic:=/camera/rgb/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/rgb/camera_info approx_sync:=true rtabmapviz:=false
```

## Launching Navigation Stack
In new terminal,
```
$ catkin_ws
$ source devel/setup.bash
$ roslaunch nav_stack_pkg move_base.launch
```