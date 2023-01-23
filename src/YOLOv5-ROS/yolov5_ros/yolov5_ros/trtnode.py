'''
This is the node for lidar and camera data sensor funsion,
where we subscribe from camera node for camera image frames and subscribe Lidar node for scan data.
We are using tensorrt model for traffic sign detection and 
based on calibration_config file we are fusing lidar data to camera detection model.
'''

import ctypes
import os
import shutil
import random
import sys
import threading
import time
import cv2
import copy
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
import math
from math import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#from bbox_ex_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from cv_bridge import CvBridge



# Defining all constants from calibration_config file
CONF_THRESH = 0.25
IOU_THRESHOLD = 0.4
MIDDLE_POINT = 337
CONVERSION_FACTOR = 0.06
STARTING_PT = 55
END_PT = 670

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
        line_thickness or round(0.001 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )


class YoLov5TRT(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path):
        # Create a Context on this device,
        self.ctx = cuda.Device(0).make_context()
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []

        for binding in engine:
            print('bingding:', binding, engine.get_binding_shape(binding))
            size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            # Append the device buffer to device bindings.
            bindings.append(int(cuda_mem))
            # Append to the appropriate list.
            if engine.binding_is_input(binding):
                self.input_w = engine.get_binding_shape(binding)[-1]
                self.input_h = engine.get_binding_shape(binding)[-2]
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        # Store
        self.stream = stream
        self.context = context
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings
        self.batch_size = engine.max_batch_size

    def infer(self, raw_image_generator, categories, scan_data):
        """
        description: Removes detections with lower object confidence score than 'conf_thres' and performs
        Non-Maximum Suppression to further filter detections.
        param:
            raw_image_generator: Input image frame by frame
            categories: List of labels for each detected class
            scan_data: Lidar scan data coming from lidar node
        return:
            result_img: Final image with detection box and distance 
        """
        # Make self the active context, pushing it on top of the context stack.
        self.ctx.push()
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        data = scan_data
        # Do image preprocess
        batch_image_raw = []
        batch_origin_h = []
        batch_origin_w = []
        batch_input_image = np.empty(shape=[1, 3, self.input_h, self.input_w])

        #print(raw_image_generator.shape)
        input_image, image_raw, origin_h, origin_w = self.preprocess_image(raw_image_generator)
        batch_image_raw.append(image_raw)
        batch_origin_h.append(origin_h)
        batch_origin_w.append(origin_w)
        np.copyto(batch_input_image, input_image)
        batch_input_image = np.ascontiguousarray(batch_input_image)

        # Copy input image to host buffer
        np.copyto(host_inputs[0], batch_input_image.ravel())
        start = time.time()
        # Transfer input data  to the GPU.
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        # Run inference.
        context.execute_async(batch_size=1, bindings=bindings, stream_handle=stream.handle)
        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        # Synchronize the stream
        stream.synchronize()
        end = time.time()
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()
        # Here we use the first row of output in that batch_size = 1
        output = host_outputs[0]
        #filter scan data from lidar
        ranges_filter = copy.copy(data.ranges)

        #convert them to list
        ranges_filter = list(ranges_filter)
        #print(len(ranges_filter))
        #anything which not in the field of view if camera should be set to 0
        for x in range(60,660):
            ranges_filter[x]=0
        # Do postprocess
        for i in range(self.batch_size):
            result_boxes, result_scores, result_classid = self.post_process(
                output[i * 6001: (i + 1) * 6001], batch_origin_h[i], batch_origin_w[i]
            )
            # Draw rectangles and labels on the original image with distance from lidar
            for j in range(len(result_boxes)):
                box = result_boxes[j]
                #print(box)
                # Dept calculation from calibration
                center = ((box[0]-box[2])/2)+box[0]
                x1 = box[0]-10
                x2 = box[2] +10
                cf = 337
                if center <= cf:
                    dept = ranges_filter[int(abs(x2-cf)/4):int(abs(x1-cf)/4)]
                    if dept:
                        dept = min(dept)
                    else:
                        dept = 0
                    
                    #print(int(abs(center-cf)/6))
                else:
                    #dept = ranges_filter[-(int((abs(center-cf)/3)))]
                    test = -int(abs(x2-cf)/4)
                    test2 = -int(abs(x1-cf)/4)
                    #print(ranges_filter[test:], test)
                    dept = ranges_filter[test:test2]
                    if dept:
                        dept = min(dept)
                    else:
                        dept = 0
                
                plot_one_box(
                    box,
                    image_raw,
                    label="{}:{:.2f},d.{:.2f}".format(
                        categories[int(result_classid[j])], result_scores[j]
                    ,dept),
                )
        return image_raw, end - start

    def destroy(self):
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()

    def preprocess_image(self, raw_bgr_image):
        """
        description: Convert BGR image to RGB,
                     resize and pad it to target size, normalize to [0,1],
                     transform to NCHW format.
        param:
            input_image_path: str, image path
        return:
            image:  the processed image
            image_raw: the original image
            h: original height
            w: original width
        """
        image_raw = raw_bgr_image
        #print(image_raw.shape)
        h, w, c = image_raw.shape
        image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        # Calculate widht and height and paddings
        r_w = self.input_w / w
        r_h = self.input_h / h
        if r_h > r_w:
            tw = self.input_w
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((self.input_h - th) / 2)
            ty2 = self.input_h - th - ty1
        else:
            tw = int(r_h * w)
            th = self.input_h
            tx1 = int((self.input_w - tw) / 2)
            tx2 = self.input_w - tw - tx1
            ty1 = ty2 = 0
        # Resize the image with long side while maintaining ratio
        image = cv2.resize(image, (tw, th))
        # Pad the short side with (128,128,128)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, None, (128, 128, 128)
        )
        image = image.astype(np.float32)
        # Normalize to [0,1]
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.ascontiguousarray(image)
        return image, image_raw, h, w

    def xywh2xyxy(self, origin_h, origin_w, x):
        """
        description:    Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
        param:
            origin_h:   height of original image
            origin_w:   width of original image
            x:          A boxes numpy, each row is a box [center_x, center_y, w, h]
        return:
            y:          A boxes numpy, each row is a box [x1, y1, x2, y2]
        """
        y = np.zeros_like(x)
        r_w = self.input_w / origin_w
        r_h = self.input_h / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h

            y /= r_h

        return y

    def post_process(self, output, origin_h, origin_w):
        """
        description: postprocess the prediction
        param:
            output:     A numpy likes [num_boxes,cx,cy,w,h,conf,cls_id, cx,cy,w,h,conf,cls_id, ...] 
            origin_h:   height of original image
            origin_w:   width of original image
        return:
            result_boxes: finally boxes, a boxes numpy, each row is a box [x1, y1, x2, y2]
            result_scores: finally scores, a numpy, each element is the score correspoing to box
            result_classid: finally classid, a numpy, each element is the classid correspoing to box
        """
        # Get the num of boxes detected
        num = int(output[0])
        # Reshape to a two dimentional ndarray
        pred = np.reshape(output[1:], (-1, 6))[:num, :]
        # Do nms
        boxes = self.non_max_suppression(pred, origin_h, origin_w, conf_thres=CONF_THRESH, nms_thres=IOU_THRESHOLD)
        result_boxes = boxes[:, :4] if len(boxes) else np.array([])
        result_scores = boxes[:, 4] if len(boxes) else np.array([])
        result_classid = boxes[:, 5] if len(boxes) else np.array([])
        return result_boxes, result_scores, result_classid

    def bbox_iou(self, box1, box2, x1y1x2y2=True):
        """
        description: compute the IoU of two bounding boxes
        param:
            box1: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))
            box2: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))            
            x1y1x2y2: select the coordinate format
        return:
            iou: computed iou
        """
        if not x1y1x2y2:
            # Transform from center and width to exact coordinates
            b1_x1, b1_x2 = box1[:, 0] - box1[:, 2] / 2, box1[:, 0] + box1[:, 2] / 2
            b1_y1, b1_y2 = box1[:, 1] - box1[:, 3] / 2, box1[:, 1] + box1[:, 3] / 2
            b2_x1, b2_x2 = box2[:, 0] - box2[:, 2] / 2, box2[:, 0] + box2[:, 2] / 2
            b2_y1, b2_y2 = box2[:, 1] - box2[:, 3] / 2, box2[:, 1] + box2[:, 3] / 2
        else:
            # Get the coordinates of bounding boxes
            b1_x1, b1_y1, b1_x2, b1_y2 = box1[:, 0], box1[:, 1], box1[:, 2], box1[:, 3]
            b2_x1, b2_y1, b2_x2, b2_y2 = box2[:, 0], box2[:, 1], box2[:, 2], box2[:, 3]

        # Get the coordinates of the intersection rectangle
        inter_rect_x1 = np.maximum(b1_x1, b2_x1)
        inter_rect_y1 = np.maximum(b1_y1, b2_y1)
        inter_rect_x2 = np.minimum(b1_x2, b2_x2)
        inter_rect_y2 = np.minimum(b1_y2, b2_y2)
        # Intersection area
        inter_area = np.clip(inter_rect_x2 - inter_rect_x1 + 1, 0, None) * \
                     np.clip(inter_rect_y2 - inter_rect_y1 + 1, 0, None)
        # Union Area
        b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1)
        b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1)

        iou = inter_area / (b1_area + b2_area - inter_area + 1e-16)

        return iou

    def non_max_suppression(self, prediction, origin_h, origin_w, conf_thres=0.5, nms_thres=0.4):
        """
        description: Removes detections with lower object confidence score than 'conf_thres' and performs
        Non-Maximum Suppression to further filter detections.
        param:
            prediction: detections, (x1, y1, x2, y2, conf, cls_id)
            origin_h: original image height
            origin_w: original image width
            conf_thres: a confidence threshold to filter detections
            nms_thres: a iou threshold to filter detections
        return:
            boxes: output after nms with the shape (x1, y1, x2, y2, conf, cls_id)
        """
        # Get the boxes that score > CONF_THRESH
        boxes = prediction[prediction[:, 4] >= conf_thres]
        # Trandform bbox from [center_x, center_y, w, h] to [x1, y1, x2, y2]
        boxes[:, :4] = self.xywh2xyxy(origin_h, origin_w, boxes[:, :4])
        # clip the coordinates
        boxes[:, 0] = np.clip(boxes[:, 0], 0, origin_w -1)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, origin_w -1)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, origin_h -1)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, origin_h -1)
        # Object confidence
        confs = boxes[:, 4]
        # Sort by the confs
        boxes = boxes[np.argsort(-confs)]
        # Perform non-maximum suppression
        keep_boxes = []
        while boxes.shape[0]:
            large_overlap = self.bbox_iou(np.expand_dims(boxes[0, :4], 0), boxes[:, :4]) > nms_thres
            label_match = boxes[0, -1] == boxes[:, -1]
            # Indices of boxes with lower confidence scores, large IOUs and matching labels
            invalid = large_overlap & label_match
            keep_boxes += [boxes[0]]
            boxes = boxes[~invalid]
        boxes = np.stack(keep_boxes, 0) if len(keep_boxes) else np.array([])
        return boxes


class inferThread():
    def __init__(self, yolov5_wrapper, categories):
        #threading.Thread.__init__(self)
        self.yolov5_wrapper = yolov5_wrapper
        self.cap = cv2.VideoCapture(0)
        self.categories = categories
        self.result = None

    def run(self):
        while(True):
            ret, frame = self.cap.read()
            #img = cv2.resize(frame[:, 240:1680],(640,480))
            self.result, use_time = self.yolov5_wrapper.infer(frame, self.categories)
            
            # show the detection in realtime window 
            #cv2.imshow('result', self.result)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break
        return self.result


class YoloTrt(Node):
    def __init__(self):
        super().__init__('trt_yolo')
        # Create publisher to publish final image with detection label and distance
        self.pub_image = self.create_publisher(Image, 'trt_image', 1)
        timer_period = 0.010  # seconds
        self.timer = timer_period
        #time.sleep(5)
        # Subscribers to get lidar data and camera frames from lidar and camera nodes respectively
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.callback_scan,1)
        self.sub_scan
        time.sleep(0.2)    #to get lidar data first iteration and then start detection
        self.sub_img = self.create_subscription(Image, 'video_frames',self.timer_callback,1)
        self.sub_img
        # Paths for tensorrt engine and plugins
        PLUGIN_LIBRARY = "/tmp/.X11-unix/ROS2_final_Thesis/src/YOLOv5-ROS/yolov5_ros/yolov5_ros/build/libmyplugins.so"
        engine_file_path = "/tmp/.X11-unix/ROS2_final_Thesis/src/YOLOv5-ROS/yolov5_ros/yolov5_ros/build/new_s_best.engine"

        if len(sys.argv) > 1:
            engine_file_path = sys.argv[1]
        if len(sys.argv) > 2:
            PLUGIN_LIBRARY = sys.argv[2]

        ctypes.CDLL(PLUGIN_LIBRARY)

        # load labels for traffic signs

        #self.categories = ["Speed limit (20km/h)"," Speed limit (30km/h)","Speed limit (50km/h)", "Speed limit (60km/h)","Speed limit (70km/h)","Speed limit (80km/h)","Double curve","Bumpy road","Slippery road","Road narrows on the right", "Road work","Traffic signals","Pedestrians", "Children crossing","Bicycles crossing","Beware of ice/snow","Wild animals crossing","End speed + passing limits","Turn right ahead","Turn left ahead","Ahead only","Go straight or right","Go straight or left","Keep right","Keep left","Roundabout mandatory","End of no passing","End no passing veh > 3.5 tons"]
        self.categories = 'Speed limit (20km/h)', ' Speed limit (30km/h)', 'Speed limit (50km/h)', 'Speed limit (60km/h)', 'Speed limit (70km/h)', 'Speed limit (80km/h)', 'End of speed limit (80km/h)', 'Speed limit (100km/h)', 'Speed limit (120km/h)', 'No passing', 'No passing veh over 3.5 tons', 'Right-of-way at intersection', 'Priority road', 'Yield', 'Stop', 'No vehicles', 'Veh > 3.5 tons prohibited', 'No entry', 'General caution', 'Dangerous curve left', 'Dangerous curve right', 'Double curve', 'Bumpy road', 'Slippery road', 'Road narrows on the right', 'Road work', 'Traffic signals', 'Pedestrians', 'Children crossing', 'Bicycles crossing', 'Beware of ice/snow', 'Wild animals crossing', 'End speed + passing limits', 'Turn right ahead', 'Turn left ahead', 'Ahead only', 'Go straight or right', 'Go straight or left', 'Keep right', 'Keep left', 'Roundabout mandatory', 'End of no passing', 'End no passing veh > 3.5 tons'

        if os.path.exists('output/'):
            shutil.rmtree('output/')
        os.makedirs('output/')
        # a YoLov5TRT instance
        self.yolov5_wrapper = YoLov5TRT(engine_file_path)
        self.br = CvBridge()
    def callback_scan(self,data):
        """
        description: Call back function for subscriber of lidar node
        param:
            data: Scan data from lidar node
        return:
            None
        """
        self.scan_data = data
        #print(data.ranges)

    def timer_callback(self, image:Image):
        """
        description: Call back function for subscriber of camera node
        param:
            image: camera image frame from camera node
        return:
            None
        """
        #self.cap = cv2.VideoCapture(4)
        categories = self.categories
        # Initialise tensorrt model and show final result with distance
        try:
            # for showing direct image captured from camera index
            #ret,initial_img = self.cap.read()
            #cv2.imshow('initial', initial_img)
            #cv2.waitKey(1)
            #image_raw = cv2.flip(image_raw,1)

            #Subcribed to camera node which is taking
            initial_img = self.br.imgmsg_to_cv2(image)
            result, use_time = self.yolov5_wrapper.infer(initial_img, categories, self.scan_data)
            self.pub_image.publish(self.br.cv2_to_imgmsg(result))
            result = cv2.line(result,(0,284),(640,284),(0,255,0),3)
            cv2.imshow('final', result)
            cv2.waitKey(1)

        except KeyboardInterrupt:
            self.yolov5_wrapper.destroy()
            exit(0)
        
def main(args=None):
    """
    description: Main function for Running ros2 node and initiazing tensorrt model
        param:
            None
        return:
            None
    """
    rclpy.init(args=args)
    yolov5_node = YoloTrt()
    rclpy.spin(yolov5_node)
    yolov5_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
            

