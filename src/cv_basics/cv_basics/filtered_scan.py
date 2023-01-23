from email.headerregistry import DateHeader
import rclpy
from rclpy.node import Node
import math
from math import *
import numpy as np
import copy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge 
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox
ranges_filter = []
intensities_filter = []
filterScan= LaserScan()
class MinimalSubscriber(Node):

    def __init__(self):
        global filterScan 
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(LaserScan,'/scan',self.callback_scan,10)

        self.subscription  # prevent unused variable warning
        self.subscription2 = self.create_subscription(Image,'trt_image',self.callback_scan2,10)
        self.subscription2  # prevent unused variable warning
        #self.subscription3 = self.create_subscription(BoundingBoxes,'yolov5/bounding_boxes',self.callback_scan3,50)
        #self.subscription3  # prevent unused variable warning
        self.publisher_ = self.create_publisher(LaserScan, '/filtere_data', 50)
        #self.br = CvBridge()
    def callback_scan(self, data):
        global ranges_filter, intensities_filter, filterScan

        len(data.ranges) #360
        len(data.intensities) #360

        ranges_filter = copy.copy(data.ranges)
        intensities_filter = copy.copy(data.intensities)

        #convert them to list
        ranges_filter = list(ranges_filter)
        intensities_filter = list(intensities_filter)
        '''global min1 
        min1 = ranges_filter[30] 
        for i in range(-30,30):    
            #Compare elements of array with min    
            if(ranges_filter[360+i] < min1):    
                min1 = ranges_filter[360+i]
                global min_inst 
                min_inst= intensities_filter[360+i]
        #filtering those angles that I do not want them (based on the question)'''
        '''for x in range(0, 360):
            if x != min1:
                ranges_filter[x] = 0
                intensities_filter[x] = 0
                data.ranges[x] = 0
                data.intensities[x] = 0
            else:
                continue'''
        for x in range(25,346):
            range_filter[x]=0
            intensities_filter[x] = 0
            data.ranges[x] = 0
            data.intensities[x] = 0
        self.publisher_.publish(data)
        
        #self.get_logger().info('I ranges: "%s"' % filterScan)
        #self.get_logger().info('I ranges: "%s"' % data)
        #self.get_logger().info('I intensities: "%s"' % filterScan.intensities)
    def callback_scan2(self, data):
        current_frame = self.br.imgmsg_to_cv2(data) 
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
    def callback_scan3(self, data):
        box = copy.copy(data.bounding_boxes)
        if box:
            x_dis_min = box[0].xmin
            x_dis_max = box[0].xmax
            x_cent = ((x_dis_max-x_dis_min)/2)+ x_dis_min
            if x_cent < middle_point:
                dept = ranges_filter[int(x_cent*conversion_factor)]
            else:
                dept = ranges_filter[int((x_cent*conversion_factor)-385)]
            self.get_logger().info('dept: "%s"' % dept)
            self.get_logger().info('image coordinate: "%s"' % x_cent)


        #self.get_logger().info('bounding data: "%s"' % data)
        #self.get_logger().info('min point: "%s"' % min1)
        #self.get_logger().info('min intensity: "%s"' % min_inst)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
