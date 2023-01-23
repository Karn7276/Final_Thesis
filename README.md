# Object Detection and Localization using Camera-Lidar sensor fusion
The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. 
From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your 
next robotics project. And it's all open source.
YOLOv5 is the world's mostly used vision AI, representing Ultralytics open-source research into future vision AI methods, incorporating lessons learned and best practices evolved over thousands of hours of research and development.
ROS2 node for SLAMTEC LIDAR
Supported SLAMTEC LIDAR
-------------------
| Lidar Model    | 
| ---------------------- | 
|RPLIDAR A1              | 
|RPLIDAR A2              | 
|RPLIDAR A3              | 
|RPLIDAR S1              |
|RPLIDAR S2              | 
|SLAMTEC LPX T1          | 

Please familiarize yourself with ROS2, before use! See https://docs.ros.org/en/foxy/Tutorials.html
Also, do the tutorials for YOLOv5 customed data object detection training and interefence from 
https://github.com/ultralytics/yolov5
Docker images and docker container knowledge is also required. Please check this site for basic docker knowledge https://docs.docker.com/
Visit following Website for more details about SLAMTEC LIDAR:

SLAMTEC LIDAR roswiki: http://wiki.ros.org/sllidar， http://wiki.ros.org/rplidar

SLAMTEC LIDAR HomePage:   http://www.slamtec.com/en/Lidar

SLAMTEC LIDAR SDK: https://github.com/Slamtec/rplidar_sdk

SLAMTEC LIDAR Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

## Description

You acquired this code:

```
ROS2_final_Thesis
└───build
└───install
└───log
└───src
    └───bbox_ex_msgs
    └───cv_basics
        └───cv_basics
            │   │   Calibration.py
            │   │   filtered_scan.py
            │   │   camera_publisher        
        │   │   package.xml
        │   │   setup.cfg
        │   │   setup.py
    └───sllidar_ros2
        └───launch
        └───rviz
        └───scripts
        └───sdk
        └───src
        │   │   package.xml
        │   │   setup.py
    └───YOLOv5_ROS
        └───yolov5_ros
            └───launch
            └───resource
            └───yolov5_ros
                └───bbox_ex_msgs
                └───build
                └───config
                └───data
                └───install
                └───log
                └───models
                └───__pycache
                └───utils
                │   │   main.py
                │   │   yolov5_best.pt
            │   │   package.xml
            │   │   setup.cfg
            │   │   setup.py
│   │   setup.sh
│   │   README.md
```

and you are currently reading ```README.md```. The ```setup.sh``` will add [GIT Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to the overall ROS2_final_thesis Git project. Each submodule represents a ROS2 Foxy Package, which will include all necessary code to run a ROS Nodes that includes a sensors on the RC-Car.  

## Getting Started

### Prerequisites
We are using Jetson Nano Development kit for deployment of this project. On jetson nano, NVIDIA offical official jetpack is installed from https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit. Please follow all instructions to install jetpack on your jetson nano Development kit. As ROS2 Foxy has requirement of Ubuntu 20.04 but official jetpack comes with ubuntu 18.04, so we have to use jetson docker container to deploy ROS2 Foxy on this device.
Please check https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit. Please follow the instructions to build the docker for ros2 foxy with pytorch. This docker image will have modules such has ROS2 Foxy and pytorch with CUDA enabled.
Please run the docker container with this built docker image. Inside the docker container, you should
clone the this repository which is at https://github.com/Karn7276/ROS2_final_Thesis.
Open the setup.sh file and install remaining requirements for this project. 
this setup.sh file will build all ROS2 packages required for this project.
### Dependencies

There are no further dependencies you need to install manually. Follow the instructions
in [Installing](###installing).


## Installing
```
ROS2_final_thesis
└───build
└───install
└───log
└───src
    └───bbox_ex_msgs
    └───cv_basics
    └───sllidar_ros2
    └───YOLOv5_ROS
│   │   setup.sh
│   │   README.md
```
when inside the direcory ```ROS2_final_thesis``` you need to run ```setup.sh``` as root, i.e. 

```
 sudo sh ./setup.sh
```
which will install everything you need to run all sensors. 

## Help

Contact the [Authors](##authors), but please make sure to make an effort before asking for help.

## Authors

Contributor names and contact info

Karan Dhage | [dhage@rhrk.uni-kl.de]


## Version History

* Version 0.01
    * I really hope it works.

## License

I have not yet started on licences.

## Acknowledgments

Thanks to [awesome-readme](https://github.com/matiassingers/awesome-readme) for the ReadMe-Template.
