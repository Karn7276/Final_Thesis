
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from sensor_msgs.msg import Image, LaserScan, PointCloud2,PointField
#import laser_geometry as lg
#import laser_geometry.laser_geometry as lg
#from sensor_msgs import point_cloud2 as pc2
import math
import sys
import struct
import ctypes
import time
#import roslib.message
_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    print('yes')
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field. datatype not in _DATATYPES:
            print('skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length =_DATATYPES[field.datatype]
            fmt+= field. count * datatype_fmt
            offset += field. count * datatype_length
    return fmt


def create_cloud(header, fields, points):
    """ 
    Create a L{sensor_msgs.msg.PointCloud2} message.
    @param header: The point cloud header.
    @type header: L(std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type fields: iterable of L(sensor_nsgs.nsg.PaintField)
    @param points: The point cloud points.
    @type points: list of iterables, i.e. one iterable for each point, with the
                    elements of each iterable being the values of the fields for
                    that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype: L{sensor_msgs.msg.PointCloud2}
    """
    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))
    buff = ctypes.create_string_buffer(cloud_struct.size* len(points))
    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step
     
    return PointCloud2 (header=header,
                        height=1,
                        width=len(points), 
                        is_dense = False, 
                        is_bigendian=False,
                        fields=fields,
                        point_step=cloud_struct.size,
                        row_step=cloud_struct.size * len(points),
                        data=buff.raw)



class LaserProjection:
    """
    A class to Project Laser Scan

    This calls will project laser scans into point clouds. It caches
    unit vectors between runs (provided the angular resolution of
    your scanner is not changing) to avoid excess computation.

    By default all range values less thatn the scanner min_range,
    greater than the scanner max_range are removed from the generated
    point cloud, as these are assumed to be invalid.

    If it is important to preserve a mapping between the index of
    range values and points in the cloud, the recommended approach is to
    pre-filter your laser scan message to meet the requirement that all
    ranges are between min and max_range.

    The generate PointClouds have a number of channels which can be enabled
    through the use of ChannelOption.
    - ChannelOption.INTENSITY - Create a channel named "intensities" with the
    intensity of the return for each point.
    - ChannelOption.INDEX     - Create a channel named "index" containing the
    index from the original array for each point.
    - ChannelOption.DISTANCE  - Create a channel named "distance" containing
    the distance from the laser to each point.
    - ChannelOption.TIMESTAMP - Create a channel named "stamps" containing the
    specific timestamp at which each point was measured.
    """

    LASER_SCAN_INVALID   = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE      = 0x00 # Enable no channels
        INTENSITY = 0x01 # Enable "intensities" channel
        INDEX     = 0x02 # Enable "index"       channel
        DISTANCE  = 0x04 # Enable "distances"   channel
        TIMESTAMP = 0x08 # Enable "stamps"      channel
        VIEWPOINT = 0x10 # Enable "viewpoint"   channel
        DEFAULT   = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0

        self.__cos_sin_map = np.array([[]])

    def projectLaser(self, scan_in,
            range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        """
        Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2.

        Project a single laser scan from a linear array into a 3D
        point cloud. The generated cloud will be in the same frame
        as the original laser scan.

        Keyword arguments:
        scan_in -- The input laser scan.
        range_cutoff -- An additional range cutoff which can be
            applied which is more limiting than max_range in the scan
            (default -1.0).
        channel_options -- An OR'd set of channels to include.
        """
        return self.__projectLaser(scan_in, range_cutoff, channel_options)

    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)

        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != scan_in.angle_min or
            self.__angle_max != scan_in.angle_max):
            #rospy.logdebug("No precomputed map given. Computing one.")

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max
            
            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        # Set the output cloud accordingly
        cloud_out = PointCloud2()

        fields = [PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance =  idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
            len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        for i in range(N):
            ri = scan_in.ranges[i]
            if ri < range_cutoff and ri >= scan_in.range_min:
                point = output[:, i].tolist()
                point.append(0)

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

        cloud_out = create_cloud(scan_in.header, fields, points)

        return cloud_out



bridge = CvBridge()
lp = LaserProjection()
lens = 'pinhole'
def get_z(T_cam_world, T_world_pc, K):
    R = T_cam_world[:3,:3]
    t = T_cam_world[:3,3]
    proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
    print(T_world_pc.shape[0])
    xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
    xy_hom = np.dot(proj_mat, xyz_hom.T).T
    z = xy_hom[:, -1]
    z = np.asarray(z).squeeze()
    return z




#rospy.init_node('reprojection')
laser_point_radius = 3
time_diff = 1


with open('/home/eit-lab/data/ros2_ws/src/new_calibration/new_calibration/calibration_result.txt', 'r') as f:
    data = f.read().split()
    qx = float(data[0])
    qy = float(data[1])
    qz = float(data[2])
    qw = float(data[3])
    tx = float(data[4])
    ty = float(data[5])
    tz = float(data[6])
q = Quaternion(qw,qx,qy,qz).transformation_matrix
q[0,3] = tx
q[1,3] = ty
q[2,3] = tz
print("Extrinsic parameter - camera to laser")
print(q)
tvec = q[:3,3]
rot_mat = q[:3,:3]
rvec, _ = cv2.Rodrigues(rot_mat)
K=  np.matrix([[6.0169303990753042e+02, 0.0, 3.1205962562768883e+02],[0.0,
       6.0220750194861989e+02, 2.4302282153249340e+02], [0., 0., 1. ]])
D = np.matrix([-7.1348559600513881e-02, 2.3163627007712124e+00,
       -5.0189955504786306e-03, -8.3655319586255113e-03,
       -9.2397991166854787e+00 ])
print("Camera parameters")
print("Lens = %s" % 'pinhole')
print("K =")
print(K)
print("D =")
print(D)

#pub = rospy.Publisher("/reprojection", Image, queue_size=1)
#scan_sub = message_filters.Subscriber(scan_topic, LaserScan, queue_size=1)
#image_sub = message_filters.Subscriber(image_topic, Image, queue_size=1)


class final_show(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.sub = self.create_subscription(LaserScan, 'scan', self.call_scanback, 10)
        #self.subscription = message_filters.Subscriber("scan",LaserScan)                          # Lidar sensor node subscription 
        #self.subscription  
        time.sleep(0.2)  # prevent unused variable warning
        self.sub2 = self.create_subscription(Image, 'video_frames', self.callback, 10)
        #self.subscription2 = message_filters.Subscriber('video_frames',Image)
        #ts = message_filters.ApproximateTimeSynchronizer([self.sub, self.sub2], 10, time_diff)
        #ts.registerCallback(callback) 
        
        
    def call_scanback(self, scan):
        self.scan = scan
        
    def callback(self, image):
        #rclpy.loginfo("image timestamp: %d ns" % image.header.stamp.to_nsec())
        #rclpy.loginfo("scan timestamp: %d ns" % self.scan.header.stamp.to_nsec())
        #diff = abs(image.header.stamp.to_nsec() - self.scan.header.stamp.to_nsec())
        #rclpy.loginfo("diff: %d ns" % diff)
        img = bridge.imgmsg_to_cv2(image)
        cloud = lp.projectLaser(self.scan)
        #print(cloud)
        points = self.laser_read_points(cloud)
        list_1 = []
        cnt = 0
        for i in points:
            cnt +=1
            list_1.append([i[0], i[1],i[2]])
            if cnt ==100:
                break
        #print(list_1)
        #objPoints = np.array(map(lambda points:[points[0],points[1],points[2]], points))
        objPoints = np.array(list_1)
        #for i in objPoints:
        #    print(i)
        Z = get_z(q, objPoints, K)
        objPoints = objPoints[Z > 0]
        if lens == 'pinhole':
            img_points, _ = cv2.projectPoints(objPoints, rvec, tvec, K, D)
            
        elif lens == 'fisheye':
            objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
            img_points, _ = cv2.fisheye.projectPoints(objPoints, rvec, tvec, K, D)
        img_points = np.squeeze(img_points)
        for i in range(len(img_points)):
            try:
                print(int(img_points[i][0]))
                print(img_points[i])
                print(list_1[i])
                cv2.circle(img, (int(img_points[i][0]),int(img_points[i][1])), 3, (0,255,0), 1)
            except OverflowError:
                continue
        cv2.imshow('calibrated', img)
        cv2.waitKey(1)
        #pub.publish(bridge.cv2_to_imgmsg(img))
    
    def laser_read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        print('yes1')
        fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from
        print(width, height)
        for v in range(height):
            offset = row_step * v
            #print(v)
            offset= row_step *v
            for u in range(width):
                yield unpack_from(data, offset)
                #print(u)
                offset += point_step 

    def extract(self, point):
        return [point[0], point[1], point[2]]
        
def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = final_show()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()

        
        
