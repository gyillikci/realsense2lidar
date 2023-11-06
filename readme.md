# Description
This ROS2 package transforms an incoming Realsense2 Depth camera pointcloud2(XYZRGB) to Lidar pointcloud2(XYZI) and then publishes the pointcloud2(XYZI) format to a given topic name.       
    
# Bug report
None known

# Build the package      
Ubuntu 20.04 and ROS2 Humble is required for this package.      


Clone this repository in your ros2 workspace (most commonly `$/home/$USER/ros2_ws`)

Source ROS2 in your terminal   

```
source /opt/ros/humble/setup.bash
```

Build the package with colcon.
```
cd /home/$USER/ros2_ws && 
colcon build --packages-select realsense2lidar --event-handlers console_direct+ &&
cd ..
```

# Run the node 

In your ros2 sourced terminal session call:    
    
``` 
ros2 run realsense2lidar realsense2lidar_node 
``` 
    
Topic parameters can be modified as follows:  

``` 
ros2 run realsense2lidar realsense2lidar_node --ros-args --remap topic_pointcloud_in:=/camera/depth/color/points --remap topic_pointcloud_in:=realsense2lidar/point_cloud_realsense2lidar
``` 
     
# compile 

Run all test defined in the package by following command 
 
```
cd /home/$USER/ros2_ws && 
colcon build
```     


# Reference
https://github.com/GitRepJo/pcl_example