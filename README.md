
#Dependencies:

1. https://github.com/ethz-asl/libpointmatcher


# How to run 
1. Make sure you have the dependencies installed.
2. launch the nodes for point cloud processing:
```
roslaunch mkp_pcd mkp_pcd.launch 
```
3. run the main file: 
```
rosrun mkp_pcd mkp_pcd_ultrasound_main
```
4. publish 777 to topic /jjj once to move the robot and collect point cloud. 
