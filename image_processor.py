import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String

class Imageprocessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        self.br = CvBridge()
        self.red_low_h = 0
        self.red_low_s = 43
        self.red_low_v = 46
        self.red_high_h = 6
        self.red_high_s = 255
        self.red_high_v = 255
        self.img_pub = self.create_publisher(Image, '/vehicle_1/camera/image_output', 10)
        self.move_red_pub = self.create_publisher(String,'/vehicle_1/camera/moving_info',10)
        self.camera_state = 'stop'
        self.color_state = 'red'

    def start(self):
        state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)

        controller_image_process_sub = self.create_subscription(String, '/controller_image_process', self.controller_image_process_msg_callback, 10)

    def controller_image_process_msg_callback(self, msg):
        if msg.data == 'start image process':
            self.camera_state = 'start'

        elif msg.data == 'landing':
            self.color_state = 'yellow'
        
        else:
            pass

    def image_callback(self,msg):
        
        img = self.br.imgmsg_to_cv2(msg,'bgr8')
        order_send= String()
        #img_output=self.img_processor(img)
        gs_img = cv2.GaussianBlur(img, (5, 5), 0)
        hsv_image= cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)
        erode_msg=cv2.erode(hsv_image, None, iterations=4)
        mask1 = cv2.inRange(erode_msg,(self.red_low_h,self.red_low_s,self.red_low_v),(self.red_high_h,self.red_high_s,self.red_high_v))

        #dilation = cv2.dilate(erode_msg, None, iterations=4)
        cnts = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        img_output = cv2.drawContours(img, cnts, -1, (0, 0, 0), 2)
        self.img_pub.publish(self.br.cv2_to_imgmsg(img_output,'bgr8'))
#         contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if self.camera_state == 'start':
            if len(cnts) > 0:
        # cv2.boundingRect()返回轮廓矩阵的坐标值，四个值为x, y, w, h， 其中x, y为左上角坐标，w,h为矩阵的宽和高
                boxes = [cv2.boundingRect(c) for c in cnts]
                for box in boxes:
                    x, y, w, h = box
                    self.get_logger().info('x={}, y={},w={},h={}'.format(x,y,w,h))
                    if ((x<420) and (y<150) and ((x+w)>180) and ((x+w)<420) and ((y+h)>150)):
                        self.get_logger().info('up')
                        order_send.data='up'
                        self.move_red_pub.publish(order_send)
                    else:
                        if ((y>150) and (x<180) and ((x+w)>180)):
                            self.get_logger().info('left')
                            order_send.data='left'
                            self.move_red_pub.publish(order_send)
                        else:
                            if ((x>180) and ((y+h)>350) and (x<420) and (y<350)):
                                self.get_logger().info('down')
                                order_send.data='dowm'
                                self.move_red_pub.publish(order_send)
                            else:
                                if ((y<350) and (x<420) and ((y+h)<350)):
                                    self.get_logger().info('right')
                                    order_send.data='right'
                                    self.move_red_pub.publish(order_send)
        else:
            pass
#         #绘制矩形框对轮廓进行定位
#                 img_output = cv2.rectangle(img, (x, y), (x+w, y+h), (153, 153, 0), 2)
#         #if img_output:

    def img_processor(self, img):
        gs_img = cv2.GaussianBlur(img, (5, 5), 0)
        hsv_image= cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv_image,(self.red_low_h,self.red_low_s,self.red_low_v),(self.red_high_h,self.red_high_s,self.red_high_v))
        erode_msg=cv2.erode(mask1, None, iterations=4)
        
        dilation = cv2.dilate(erode_msg, None, iterations=4)
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
    # cv2.boundingRect()返回轮廓矩阵的坐标值，四个值为x, y, w, h， 其中x, y为左上角坐标，w,h为矩阵的宽和高
            boxes = [cv2.boundingRect(c) for c in contours]
            for box in boxes:
                x, y, w, h = box
        #绘制矩形框对轮廓进行定位
                origin_pic = cv2.rectangle(img, (x, y), (x+w, y+h), (153, 153, 0), 2)
            self.get_logger().info('get')
            return origin_pic
        else: 
            return img


def main(args=None):
    rclpy.init(args=args)
    image_node= Imageprocessor()
    image_node.start()
    rclpy.spin(image_node)
    

if __name__ == '__main__':
    main()


