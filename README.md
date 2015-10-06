# SickScan3D  

## Introduction
The rospackage is based on sicktoolbox_wrapper. When running, there are 4 nodes: 

- **sicklms**
communicate with a sick laser, publishing a laser_scan topic ("scan", 2D distance information).     
It publishs the topic: `scan (sensor_msgs/LaserScan)`  
*NOTE:This node is provided by sicktoolbox_wrapper ros package which means you should install it in advance*.
```
sudo apt-get install ros-indigo-sicktoolbox-wrapper
```

- **motorcontrol**
control the rotation motor, and estimate the motor angle using EKF.  
It publishs a motor angle topic :`motorangle`-- the angle corresponding to current laser_scan message.

- **scan3d**
generate 3D point clouds according to the messages provided by `scan` and `motorangle`.  
It publishs a pointcloud topic: `cloud`-- a frame of 3D point clouds.

- **merge_cloud**  
accumulatively merge pointcloud frames to reconstruct the 3D environment.  
It publishs a merged pointcloud topic:`cloud_out`--accumulated point clouds.  

## Installation & Launch   
1.	Install sicktoolbox_wrapper by running.  
2.	Compile the sickscan3d source codes.  
3.	Adapt the parameters in launch files, especially the port names for the sick laser and the EPOS motor driver.  
4.	Run a launch file, e.g., test.launch.  
5.	The published topics can be visualized through rviz.  

## NOTE & TODO  
1.	The sicklms I used was a revised version (sicklms1). Compared to the original version, I modified the default settings for "port" to "/dev/ttyUSB1". At that time, I did not find a effective way to set the port name in the launch file.  

2.	The control of the EPOS motor driver is based on an open source library ("epos.cpp") which controls the motor driver through RS232 serial communication.  

3.	The EPOS driver works at the "Profile Position Mode". The detail description can be found in the file "EPOS-Application-Note-Device-Programming-En.pdf". Before running the sickscan3d rospackage, the parameters for the "Profile Position Mode" can be adjusted using the official EPOS installation and configuration software for Windows. The tutorial can be found in the file "300583_Getting_Started_En.pdf".  

4.	The parameters for EKF are still very rough, more work is necessary to optimize the motor angle estimation, which is quite important for 3D sensing with a higher rotation velocity.  

5.	The current version of merging method is still very rough. It just accumulates a couple of pointcloud frames without further processing. In each turn of rotation direction, the pointcloud will be reset, i.e., the 3D reconstruction is done according to all frames happen in each clockwise or counterclockwise rotation.  

6.	I remember that the parameters set in the launch file cannot be transported to the programs. I am not sure whether I had solved this problem in this version. If possible, you'd better test it.


Tuesday, 06. October 2015 09:49PM 
by [@ZHANG fangyi](mailto:gzzhangfangyi@gmail.com)




