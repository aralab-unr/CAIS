# CAIS: Culvert Autonomous Inspection Robotic System

**Culverts, essential components of drainage systems, require regular inspection to ensure their optimal functionality. However, culvert inspections pose numerous challenges, including accessibility, defect localization, and reliance on superficial visual assessments. To address these challenges, we propose a novel  Culvert Autonomous Inspection Robotic System (CAIS) equipped with advanced sensing and evaluation capabilities. Our solution integrates deep learning methodologies, lighting systems, and non-destructive evaluation (NDE) techniques to enable accurate defect localization and comprehensive condition assessment. We present a pioneering Partially Observable Markov Decision Process (POMDP) framework to resolve uncertainty in autonomous inspection, especially in confined and unstructured environments like culverts or tunnels. The framework outputs detailed 3D maps highlighting visual defects and NDE condition assessments, demonstrating consistent and reliable performance in various indoor and outdoor scenarios. Additionally, we provide an open-source implementation of our framework on GitHub, contributing to the advancement of autonomous inspection technology and fostering collaboration within the research community.**
<p align='center'>
    <img src="./pic/robot.jpg" alt="drawing" width="800"/>
</p>

# Dependencies
The framework has been tested with ROS Noetic and Ubuntu 20.04. The following configuration, along with the required dependencies, has been verified for compatibility:

- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (```roscpp```, ```std_msgs```, ```sensor_msgs```, ```geometry_msgs```)
- [ZED SDK >= 3.5](https://www.stereolabs.com/developers)
- [CUDA](https://developer.nvidia.com/cuda-downloads) (Recommend to use CUDA toolkit >= 11 for Ubuntu 20.04)


## Visual and Condition Assessment
Indoor            |  Outdoor
:-------------------------:|:-------------------------:
![](https://github.com/aralab-unr/CAIS/blob/main/pic/fusion.jpg)  |  ![](https://github.com/aralab-unr/CAIS/blob/main/pic/fusion_outdoor.jpg)
