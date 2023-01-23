import launch
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    
    
    Tensor_rt = Node(package= 'yolov5_ros', executable = 'New_trt', name = 'Trt')
    ld.add_action(Tensor_rt)

    return ld
