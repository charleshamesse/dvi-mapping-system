# Depth-Visual-Inertial (DVI) Mapping System
## A Robust and Precise 3D Odometry and Mapping System for DVI Sensors

**Sep 26 2024: we open source our code and our revised paper is submitted to IEEE RA-L.**

We propose the Depth-Visual-Inertial (DVI) mapping system: a robust multi-sensor fusion framework for dense 3D mapping using time-of-flight cameras equipped with RGB and IMU sensors. Inspired by recent developments in real-time LiDAR-based odometry and mapping, our system uses an error-state iterative Kalman filter for state estimation: it processes the inertial sensor's data for state propagation, followed by a state update first using visual-inertial odometry, then depth-based odometry. This sensor fusion scheme makes our system robust to degenerate scenarios (e.g. lack of visual or geometrical features, fast rotations) and to noisy sensor data, like those that can be obtained with off-the-shelf time-of-flight sensors. 

For evaluation, we propose the new [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset), featuring data from multiple DVI sensors recorded in challenging conditions reflecting search-and-rescue operations.

This video shows the proposed system running on all evaluation sequences of the Bunker DVI Dataset:

<a href="https://www.youtube.com/embed/GzjHYDx21o0" target="_blank"><img src="http://img.youtube.com/vi/Nr6ZI32Nbs8/0.jpg" 
alt="cla" width="640" height="480" border="10" /></a>

This code runs on **Linux**, and is fully integrated with **ROS**. It has been tested with ROS Noetic. In the future, we plan to implement a ROS2 compatible version.

**Authors:** Charles Hamesse, Michiel Vlaminck, Hiep Luong, Rob Haelterman

**Related Papers and Code**

Our article is currently under review by IEEE RA-L. 

Our proposed dataset is available here: [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset)

For the VIO compoenent, this code reuses parts of VINS-Mono:
* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018),  [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Robotics, [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert), [GitHub repo](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)


For the DOM component, this code uses parts of VoxelMap:
* **Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry**, Chongjian Yuan, Wei xu, Xiyuan Liu, Xiaoping Hong, Fu Zhang, IEEE Robotics and Automation Letters 2022, [pdf](https://arxiv.org/abs/2109.07082), [GitHub repo](https://github.com/hku-mars/VoxelMap)


## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
This code was tested with Ubuntu 20.04 and ROS Noetic. 

[ROS Installation](http://wiki.ros.org/ROS/Installation)

Additional ROS packages:
```
sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 20.04, ROS Noetic, OpenCV 3.3.1, Eigen 3.3.3) 

## 2. Build DVI Mapping System on ROS
Clone the repository and catkin build:
```
    cd ~/catkin_ws/src
    git clone  
    cd ../../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Run
For examples with the MS Kinect for Azure sensor, open two terminals and run:
```
roslaunch dvi_ms k4a.launch
rosbag play your_dataset.bag
```

## 4. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

This is research code and is provided as is, without warranty of functioning for your own use cases. We are still working on a large refactoring and improving various features. For any technical issues, please contact Charles Hamesse <charles.hamesse@mil.be>.

