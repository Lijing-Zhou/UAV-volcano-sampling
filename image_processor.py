"""
Very simple image processor based on example from
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        self.br = CvBridge()
        self.red_low_H = 0
        self.red_low_S = 60
        self.red_low_V = 60
        self.red_high_H = 6
        self.red_high_S = 255
        self.red_high_V = 255

        self.yellow_low_H = 26
        self.yellow_low_S = 60
        self.yellow_low_V = 60
        self.yellow_high_H = 34
        self.yellow_high_S = 255
        self.yellow_high_V = 255

        self.blue_low_H = 100
        self.blue_low_S = 43
        self.blue_low_V = 46
        self.blue_high_H = 120
        self.blue_high_S = 255
        self.blue_high_V = 255

        self.img_pub = self.create_publisher(Image, '/vehicle_1/camear/image_output', 10)        

    def start(self):
        # set up subscriber for image
        state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)

    # on receiving image, convert and log information
    def image_callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        # can do OpenCV stuff on img now
        # shp = img.shape # just get the size
        # self.get_logger().info('Got an image of {} x {}'.format(shp[0],shp[1]))

        # color = 'red'
        gs_img = cv2.GaussianBlur(img, (5, 5), 0)
        hsv_img = cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)
        erode_img = cv2.erode(hsv_img, None, iterations=2)
        inRange_img = cv2.inRange(erode_img, (self.red_low_H, self.red_low_S, self.red_low_V)
                                , (self.red_high_H, self.red_high_S, self.red_high_V))
        cnts = cv2.findContours(inRange_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        img_processed = cv2.drawContours(img, cnts, -1, (0, 0, 0), 2)
        # inRange_img = self.image_preprocess(img, color)
        # img_output = self.draw_border(inRange_img, img)
        self.img_pub.publish(self.br.cv2_to_imgmsg(img_processed, 'bgr8'))
        self.get_logger().info('image publish successfully')

    def image_preprocess(self, img, color):
        if img == None:
            self.get_logger().info('No image input')
            return None
        
        else:
            # GussianBlur: make the image more blurred, the pixels are smoother
            gs_img = cv2.GaussianBlur(img, (5, 5), 0)
            # convert BGR to HSV
            hsv_img = cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)
            # d
            erode_img = cv2.erode(hsv_img, None, iterations=2)
            # Remove all parts oustide the color threshold range and convert into a binary image
            if color == 'red':   
                inRange_img = cv2.inRange(erode_img, (self.red_low_H, self.red_low_S, self.red_low_V)
                                        , (self.red_high_H, self.red_high_S, self.red_high_V))
            elif color == 'yellow':
                inRange_img = cv2.inRange(erode_img, (self.yellow_low_H, self.yellow_low_S, self.yellow_low_V)
                                        , (self.yellow_high_H, self.yellow_high_S, self.yellow_high_V))        
            self.get_logger().info('image_preprocess successfully')
            return inRange_img

    def draw_border(self, inRange_img, img):
        # only find the peripheral contour and the inflection point information
        cnts = cv2.findContours(inRange_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        img = cv2.drawContours(img, cnts, -1, (255, 255, 255), 2)

        return img


def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    rclpy.spin(image_node)


if __name__ == '__main__':
    main()