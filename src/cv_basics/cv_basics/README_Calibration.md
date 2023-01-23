# ROS2 Node for [Calibration of stereo camera and rp1 Lidar](https://github.com/Karn7276/ROS2_final_Thesis)

You acquired this code

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
And you are currently reading ```Calibration_README.md```. Before you continue, start with reading ```README.md```. 
Once you have finished reading ```README.md``` and followed its instructions you can continue reading here. 

Great! You should now have a directory ```ROS2_final_Thesis``` which contains code from 
[here](https://github.com/Karn7276/ROS2_final_Thesis).

## Getting Started

The ```setup.sh``` already built the ROS2-Package inside ```./ROS2_final_Thesis```. Connect the Stereo-Camera and rp1 Lidar to your Jetson nano or PC using a USB-C-to-USB cable. 
It's advised to use a USB3-port on your computer. Please run the ros2 node for starting and publishing camera data by starting a ROS2 node 
```
ros2 run cv_basics img_publisher
```
[pkg: cv_basics executable: img_publisher] 

and another ros2 node for starting rp1 lidar sensor and publishing lidar data as
```
ros2 launch sllidar_ros sllidar_view_ros.py
```
pkg: sllidar_ros.
Please read all related ros2 node READ.md file for more understanding.

```
cd ./ROS2_final_Thesis
source ./install/setup.bash
```

to run the ROS2-Node. Check if it worked by opening a new terminal and calling

```
source /opt/ros/foxy/setup.bash
ros2 topic list
```

The topic list should contain these 2 topics which are necessary for our calibration

```
/video_frames
/scan
```
and then run this calibration node
Before starting calibrtation ros2 node we need to have a 3D printed pyramid which I and my teammate designed and printed. This object will be placed in front of camera and lidar setup for calibration. First check this object is in the frame of camera by starting and visualising captured video frames. Secondly we have to place this object in the horizontal 2D line of lidar sensor so we can get only on point from the peak of the 3D pyramid. When we set up this frame which can satisfy both conditions mentioned above.

Once everything is setup, we can move forward with the calibration. This is the first version of our calibration node so we have to make measurements manually but I am working on the second version which can directly take frame coordinates once we set up the frame and pyramid. 
Please run this ros2 command to start calibration 

```
ros2 run cv_basics Calibration 
```
Calibration node subscribes to /video_frame and /scan topics from camera and lidar sensor nodes respectively. Video_frames are displayed using opencv library where with riz we can visualise /scan data. Firstly, we filter /scan data approximately for front angles such as [30° to 330°]
once we can see the one maximum peak from 3D printed pyramid. From displayed image we can get the frame coordinates of the pyramid peak by sliding the curser on the peak. By using numpy array we can get the minimum distance and corrensponding to that value will be the orientation angle of laser from filtered data[/filtere_scan] of lidar sensor. we have not the x1 value from image cordinate and corresponding angle of laser to that point as A1.
We do above process once again but this time 3D pyramid will be placed to another position on the horizontal line which can satisfy initial 2 conditions. With this, we have X2 and corresponding A2 angle. by using coorrelation method, we have convertion factor as X1-X2/A1-A2. 
Simple math will get us the all conversions for corelating any image cordinate to laser angle. This angle will give us the exact distance of this object from lidar sensor data.  
## Help

Contact the [Authors](#Authors), but please make sure to make an effort before asking for help. For references, check

- https://github.com/Karn7276/ROS2_final_Thesis

## Authors

Karan Sanjay Dhage| [dhage@rhrk.uni-kl.de]

## Version History

* Version 0.01
    * I really hope it works.

## License

I have not yet started on licences.

## Acknowledgments

Thanks to [awesome-readme](https://github.com/matiassingers/awesome-readme) for the ReadMe-Template.
