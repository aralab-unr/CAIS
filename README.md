# IMM_LIO

**IMM-LIO (Interaction Multiple Models LiDAR-Inertial Odometry) is a reliable and effective LiDAR-inertial odometry package that supports multiple filters for estimation. It mitigates LiDAR motion distortion by leveraging high-rate IMU data through a tightly-coupled approach using the IMM filter. Three models—constant velocity, acceleration, and turning rate—are employed in this work.**
<p align='center'>
    <img src="./doc/New_college.gif" alt="drawing" width="800"/>
</p>

# Dependencies
The framework has been tested with ROS Noetic and Ubuntu 20.04. The following configuration, along with the required dependencies, has been verified for compatibility:

- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (```roscpp```, ```std_msgs```, ```sensor_msgs```, ```geometry_msgs```)
- C++ 14
- [PCL >= 1.8](https://pointclouds.org/downloads/)
- [Eigen >= 3.3.4](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [ZED SDK >= 3.5](https://www.stereolabs.com/developers)
- [CUDA](https://developer.nvidia.com/cuda-downloads) (Recommend to use CUDA toolkit >= 11 for Ubuntu 20.04)
