from launch import LaunchDescription
from launch_ros.actions import Node
import time
def generate_launch_description():
    serial_port = '/dev/ttyUSB0'
    serial_baudrate = '115200' #for A1/A2 is 115200
    frame_id = 'laser'
    inverted = 'false'
    angle_compensate = 'true'
    ld = LaunchDescription()
    Camera = Node( package='cv_basics',namespace='camera',executable='img_publisher',name='video_frames')
    #Lidar = Node(package='sllidar_ros2',node_executable='sllidar_node',node_name='sllidar_node',parameters=[{'serial_port': serial_port},{'serial_baudrate': serial_baudrate},{'frame_id': frame_id},{'inverted': inverted},{'angle_compensate': angle_compensate}],output='screen')
    Tensor_rt = Node(package='yolov5_ros',namespace='yolo',executable='New_trt',name='yolo_node'),

    ld.add_action(Camera)
    #ld.add_action(Lidar)
    time.sleep(2)
    ld.add_action(Tensor_rt)
    return ld
