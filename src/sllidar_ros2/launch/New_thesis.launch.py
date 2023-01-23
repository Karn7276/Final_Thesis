import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import time
from launch.actions import TimerAction
def generate_launch_description():
    ld = launch.LaunchDescription()
    include_launch = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource('/tmp/.X11-unix/ROS2_final_Thesis/src/sllidar_ros2/launch/sllidar_launch.py'))
    camera = Node(package= 'cv_basics', executable= 'img_publisher')

    #Tensor_rt = Node(package='yolov5_ros',executable='New_trt')
    ld.add_action(include_launch)
    ld.add_action(camera)
    #ld.add_action(Tensor_rt)

    return ld
