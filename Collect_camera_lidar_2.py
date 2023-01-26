import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import cv2
import yaml

u = None
v = None

x = None
y = None

alpha = 1.0
beta = 0

clicked = False
fig = None

sub_img = np.array([])
K=  np.matrix([[6.0828185767968671e+02, 0, 3.3842269191061536e+02],[0,6.0903767639365606e+02, 2.4789566223578996e+02],[0,0,1]])
D = np.matrix([1.7220089819731990e-01, -2.5647187248578546e-01,7.1051611387101888e-03, 4.1046774380740303e-03])
lens = 'pinhole' 
bridge = CvBridge()

class matplot:

    def __init__(self):
        global alpha, beta
        alpha = 1.0
        beta = 0
        self.create_figure()
            
    def create_figure(self):
        global fig
        if sub_img.size == 0:
            print('No image obtained.')
            return
        self.img = sub_img
        self.img_ori = sub_img
        self.fig = plt.figure()
        self.fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
        ax = self.fig.add_subplot(111)
        self.image = ax.imshow(self.img)
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.fig.canvas.mpl_connect('key_press_event', self.onkey)
        fig = self.fig
        plt.show()
        
    def onclick(self, event):
        global u, v
        if event.xdata != None and event.ydata != None and event.button == 3:
            u = int(event.xdata)
            v = int(event.ydata)
            img_temp = np.copy(self.img)
            cv2.circle(img_temp, (u, v), 1, (255,0,0), 1)
            self.image.set_data(img_temp)
            plt.draw()

    def onkey(self, event):
        global u, v, clicked, alpha, beta
        if event.key == 'enter':
            if u == None and v == None:
                print('No point selected. Try again.')
            else:
                print('Image coordinate: (%d, %d), Laser coordinate: (%f, %f)' % (u, v, x, y))
                plt.close()
                with open('data.txt', 'a') as f:
                    f.write('%f %f %d %d\n' % (x, y, u, v))
                u, v = None, None
        elif event.key in ['up','down','pageup','pagedown']:
            if event.key == 'up':
                alpha += 0.1
            if event.key == 'down':
                alpha -= 0.1
            if event.key == 'pageup':
                beta += 10
            if event.key == 'pagedown':
                beta -= 10
            self.img = cv2.addWeighted(self.img_ori, alpha, self.img_ori, 0, beta)
            self.image.set_data(self.img)
            self.fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
            plt.draw()

def point_cb(msg):
    global x, y, clicked, fig
    x = msg.pose.position.x
    y = msg.pose.position.y
    if clicked and plt.get_fignums():
        fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
        plt.draw()
    else:
        pass
    clicked = True

def image_cb(msg):
    global sub_img
    sub_img_distorted = bridge.imgmsg_to_cv2(msg)
    sub_img_distorted = cv2.cvtColor(sub_img_distorted, cv2.COLOR_BGR2RGB)
    lens= 'pinhole'
    if lens == 'pinhole':
        sub_img = cv2.undistort(sub_img_distorted, K, D, newCameraMatrix = K)
    elif lens == 'fisheye':
        sub_img = cv2.fisheye.undistortImage(sub_img_distorted, K, D, Knew = K)

def Shutdown():
    plt.close()
    
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')      #'/move_base_simple/goal'
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            point_cb,
            1)                                                                   # Lidar sensor node subscription 
        self.subscription  
        #time.sleep(0.2)                                                       # prevent unused variable warning
        self.subscription2 = self.create_subscription(
            Image,
            'video_frames',
            image_cb,
            1)                                                                   # Camera node subscription   
        self.subscription2   
        
def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    while not rclpy.is_shutdown():
        if clicked:
            matplot()
            clicked = False
            

if __name__ == '__main__':
    main()
