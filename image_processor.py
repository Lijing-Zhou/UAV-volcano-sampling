from cmath import cos, sin
from dis import dis
from pickle import NONE
from re import A
from select import select
# from sys import ps2                                             # type: ignore
import rclpy                                                    # type: ignore
from rclpy.node import Node                                     # type: ignore
from sensor_msgs.msg import Image, Imu, NavSatFix               # type: ignore
import cv2                                                      # type: ignore
from cv_bridge import CvBridge                                  # type: ignore
from std_msgs.msg import String                                 # type: ignore
from std_msgs.msg import Float32                                # type: ignore
from geometry_msgs.msg import Twist,TwistStamped,PoseStamped    # type: ignore
import numpy as np                                              # type: ignore
import math

class Point(object):
    def __init__(self,xParam = 0.0,yParam = 0.0):
        self.x = xParam
        self.y = yParam
        self.dist = 100
        self.invalid = False

    def __str__(self):
        return "(%.2f, %.2f)"% (self.x ,self.y)


    def sum(self,x, y):
        self.x += x
        self.y += y

    def reset(self, x, y):
        self.x = x
        self.y = y
        self.dist = 100
        self.invalid = False

    def cal_distance(self, p2):
        self.dist = math.sqrt(math.pow(self.x - p2.x, 2) + math.pow(self.y - p2.y, 2))
        return self.dist

    def to_int(self):
        self.x = int(self.x)
        self.y = int(self.y)

    def rotate(self, angle):
        theta = math.radians(angle)

        x1 = self.x * cos(theta) - self.y * sin(theta)
        y1 = self.x * sin(theta) + self.y * cos(theta)

        self.x = x1.real
        self.y = y1.real


class Direction(object):
    DIR_NOTRH = 'N'
    DIR_SOUTH = 'S'
    DIR_EAST = 'E'
    DIR_WEST = 'W'
    DIR_NE = 'NE'
    DIR_NW = 'NW'
    DIR_SE ='SE'
    DIR_SW = 'SW'
    # 降落
    DIR_DOWN = 'DOWN'
    # 表示没有计算出方向
    DIR_NONE = 'NONE'
    # 表示红色状态可以结束
    DIR_DONE = 'DONE'

class Location(object):
    def __init__(self,lat = 0.0,lng = 0.0):
        self.lat = lat
        self.lng = lng
    
    def cal_dist(self, location):
        ra = 6378140  # radius of equator: meter
        rb = 6356755  # radius of polar: meter
        flatten = (ra - rb) / ra  # Partial rate of the earth
        # change angle to radians
        radLatA = math.radians(self.lat)
        radLonA = math.radians(self.lng)
        radLatB = math.radians(location.lat)
        radLonB = math.radians(location.lng)
 
        pA = math.atan(rb / ra * math.tan(radLatA))
        pB = math.atan(rb / ra * math.tan(radLatB))
        x = math.acos(math.sin(pA) * math.sin(pB) + math.cos(pA) * math.cos(pB) * math.cos(radLonA - radLonB))
        c1 = (math.sin(x) - x) * (math.sin(pA) + math.sin(pB))**2 / math.cos(x / 2)**2
        c2 = (math.sin(x) + x) * (math.sin(pA) - math.sin(pB))**2 / math.sin(x / 2)**2
        dr = flatten / 8 * (c1 - c2)
        distance = ra * (x + dr)
        return distance.real
    
    def get_distance(self, loca):
         # 经纬度转换成弧度
        lat0 = math.radians(self.lat)
        lat1 = math.radians(loca.lat)
        lng0 = math.radians(self.lng)
        lng1 = math.radians(loca.lng)

        dlng = math.fabs(lng0 - lng1)
        dlat = math.fabs(lat0 - lat1)
        h = self.hav(dlat) + cos(lat0) * cos(lat1) * self.hav(dlng)
        distance = 2 * 6371000 * math.asin(math.sqrt(h.real))
        return distance.real

    def hav(self,theta):
        s = sin(theta / 2)
        return s * s

    # 计算两个坐标沿逆时针方向转到正东方向的夹角
    def cal_angle(self, loca):
        radLatA = math.radians(self.lat)
        radLonA = math.radians(self.lng)
        radLatB = math.radians(loca.lat)
        radLonB = math.radians(loca.lng)
        dLon = radLonB - radLonA
        y = math.sin(dLon) * math.cos(radLatB)
        x = math.cos(radLatA) * math.sin(radLatB) - math.sin(radLatA) * math.cos(radLatB) * math.cos(dLon)
        brng = math.degrees(math.atan2(y, x))
        brng = (brng + 360) % 360 - 90
        return brng

class Mode():
    # 红色模式
    MODE_RED = 1
    # 黄色模式
    MODE_YELLOW = 2
    # 默认模式，即不工作
    MODE_NONE = 0
    
class Imageprocessor(Node):
    def __init__(self):

        super().__init__('image_processor')

        # 需要讨论
        self.TOPIC_NAME_FINITE_STATE = '/controller_image_process'

        # 本地模块开关
        self.switch_img_proc = False
    
        ### 状态机的启动信息为RED或YELLOW，之后图像识别自动进入相应的模式，关闭信息为STOP，图像识别会自动关闭
        ### 本地开关switch是有最优先级的，在图像识别处理时，开关为关，则不会工作
    
        # 需要讨论
        self.ORDER_RED = 'RED'

        # 需要讨论
        self.ORDER_YELLOW = 'YELLOW'

        #需要讨论
        self.ORDER_START = 'START'

        # 需要讨论
        self.ORDER_STOP = 'STOP'

        # 需要讨论
        self.TOPIC_NAME_MOVE_DIRECTION = '/image_process_controller'

        # self use
        self.TOPIC_NAME_IMAGE_OUTPUT = '/vehicle_1/camera/image_output'

        self.TOPIC_NAME_VELOCITY_ANGULAR = '/vehicle_1/mavros/local_position/velocity_local'

        self.TOPIC_NAME_LOCAL_POSITION = '/vehicle_1/mavros/local_position/pose'

        self.TOPIC_NAME_GLOBAL_POSITION = '/vehicle_1/mavros/global_position/global'

        self.TOPIC_NAME_COMPASS = '/vehicle_1/mavros/global_position/compass_hdg'

        self.TOPIC_NAME_IMU = '/vehicle_1/mavros/imu/data'

        # 移动方向列表
        self.list_move_dir = Direction()

        # 模式列表
        self.list_mode = Mode()

        # 计算出的移动方向
        self.move_dir = self.list_move_dir.DIR_NONE

        # 图像识别模式
        self.myMode = Mode()
        self.myMode = self.list_mode.MODE_NONE
        
        # 图像识别计时器
        self.img_proc_timer = 0

        # 存储当前速度
        self.cur_vel = Twist()

        # 存储当前的局部坐标系的位置
        self.local_pos = PoseStamped()
        
        # 无人机在相机中的投影半径
        self.radius_shadow = 10
        # 无人机的实际半径
        self.radius_drone = 0.5

        # 无人机在摄像头图图像中的位置
        self.drone_point = Point(320, 240)

        # 相机摄影实际大小
        self.img_width = 0
        self.img_height = 0

        # 无人机移动8个方向的目标位置
        #--------------------> x
        #| p0   p1  p2
        #| p3   机  p4
        #| p5   p6  p7
        #y
        self.list_dest = [  Point(320, 240),    Point(320, 240),    Point(320, 240),
                            Point(320, 240),                        Point(320, 240),
                            Point(320, 240),    Point(320, 240),    Point(320, 240)]
        
        # 无人机自身在摄像机视野中的位置
        self.loca_camera = Point(320, 240)

        # 红色区域的矩阵坐标集合
        self.list_area_red = []
        # 黄色区域的矩阵坐标集合
        self.list_area_yellow = []

        #redHsv
        self.br = CvBridge()
        self.lower_red = np.array([0, 43, 46])
        self.upper_red = np.array([6, 255, 255])

        self.lower_yellow = np.array([26, 43, 46])
        self.upper_yellow = np.array([34, 255, 255])

        # 无人机经纬度坐标
        self.global_loca = Location(0,0)
        # 目的地坐标
        # self.dest_loca = Location(51.4219206, -2.6687700)
        self.dest_loca = Location(51.4219206, -2.6687700)

        # 目的地在相机图像中的坐标
        self.dest_point = Point(0, 0)

        # 内圆半径
        self.inner_circle_r = 35
        # 外圆半径
        self.excircle_r = 50

        # 内园在相机视野中的半径
        self.inner_circle_r_img = 0
        # 外圆在相机视野中的半径
        self.excircle_r_img = 0

        # 创建摄像头移动的client
        self.camera_pub = self.create_publisher(Float32, '/vehicle_1/gimbal_tilt_cmd', 10)
        self.get_logger().info('Image Processor: Create a publisher for camera movement')

        # 创建一个发布移动方向的topic
        self.move_dir_pub = self.create_publisher(String, self.TOPIC_NAME_MOVE_DIRECTION, 10)
        self.get_logger().info('Image Processor: Published a topic for dir msg')

        #创建一个发布输出图像的topic
        self.img_pub = self.create_publisher(Image, self.TOPIC_NAME_IMAGE_OUTPUT, 10)
        self.get_logger().info('Image Processor: Published a topic for img output')

    
    def start(self):
        #接受摄像头原始图像
        state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)

        # 订阅状态机的节点
        self.state_sub = self.create_subscription(String, self.TOPIC_NAME_FINITE_STATE, self.state_callback, 10)
        self.get_logger().info('Image Processor: Subscriped to finite_state')
        
        # 订阅速度节点
        self.speed_sub = self.create_subscription(TwistStamped, self.TOPIC_NAME_VELOCITY_ANGULAR, self.speed_callback,10)
        self.get_logger().info('Image Processor: Subscriped to velocity_state')

        # 订阅局部位置节点
        self.local_pos_sub = self.create_subscription(PoseStamped, self.TOPIC_NAME_LOCAL_POSITION, self.local_pos_callback,10)
        self.get_logger().info('Image Processor: Subscriped to local_position')

        # 订阅全局位置节点
        self.global_pos_sub = self.create_subscription(NavSatFix, self.TOPIC_NAME_GLOBAL_POSITION, self.global_pos_callback,10)
        self.get_logger().info('Image Processor: Subscriped to global_position')

        # 订阅IMU节点，由于获取朝向
        self.local_pos_sub = self.create_subscription(Imu, self.TOPIC_NAME_IMU, self.imu_callback,10)
        self.get_logger().info('Image Processor: Subscriped to imu')

        # 1秒执行一次图像识别
        self.timer = self.create_timer(1, self.timer_callback)

        # 默认将摄像头调到水平
        self.turn_camera(0)
        self.get_logger().info('Image_Processor_Topic: init camera angle')

# callback functions=============================================================================================
    # 读取速度
    def speed_callback(self,msg):
        self.cur_vel = msg.twist
        #self.get_logger().debug('Image_Processor_Topic: Current velocity is--linear: x{} y{} z{}, angluar: x{} y{} z{}'.format(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z))
        # self.get_logger().info('Image Processor Topic: Is drone still: {}'.format(self.is_drone_still()))
    
    # 接受状态机topic的信息，根据内容，决定是否开启图像识别以及识别模式
    def state_callback(self, msg):
        # log信息内容
        self.get_logger().debug('Image_Processor_Topic: state_callback with msg {}'.format(msg.data))

        # 状态机发消息让图像识别进入红色模式
        if msg.data == self.ORDER_RED:
            # 进入红色模式
            self.myMode = self.list_mode.MODE_RED
            
            # 开启图像识别开关
            # 开关未开
            if not self.switch_img_proc:
                self.switch_img_proc = True
                self.get_logger().info('Image_Processor_Topic: Now active image processor and into red mode')
                # 将摄像头调整到垂直向下
                self.turn_camera(1.57)
            # 开关已经开了
            else:
                pass
            
        # 状态机发消息让图像识别进入黄色模式
        elif msg.data == self.ORDER_YELLOW:
            # 设置黄色模式
            self.myMode = self.list_mode.MODE_YELLOW

            if not self.switch_img_proc:
                self.switch_img_proc = True
                self.get_logger().info('Image_Processor_Topic: Now active image processor and into yellow mode')
                
                # 将摄像头调整到垂直向下
                self.turn_camera(1.57)

            # 开关已经开了
            else:
                pass

        # 状态机发消息让图像识别停止工作
        elif msg.data == self.ORDER_STOP:
            
            # 设置为默认模式，即不工作
            self.myMode = self.list_mode.MODE_NONE

            # 关闭图像识别开关
            self.switch_img_proc = False
            self.get_logger().info('Image_Processor_Topic: Now deactive image processor')
            
            # 将摄像头角度调整到默认
            self.turn_camera(0.7)
    
    def local_pos_callback(self, msg):
        self.local_pos = msg
        # self.get_logger().debug('Image Processor Topic: Local position is {} {} {}'.format(self.local_pos.pose.position.x, self.local_pos.pose.position.y, self.local_pos.pose.position.z))

    def global_pos_callback(self, msg):
        self.global_loca.lat = msg.latitude
        self.global_loca.lng = msg.longitude
        # self.get_logger().debug('Image_Processor_Topic: global pos is {} {}'.format(self.global_loca.lat, self.global_loca.lng))
    
    def imu_callback(self, msg):
        # 无人机与正东方向的偏角，正为东偏南，负为东偏北， -1 到 1
        self.orientation = msg.orientation.z

    def timer_callback(self):
        new_dir = self.image_process()

        if new_dir == self.move_dir:
            self.img_proc_timer += 1
        else:
            self.img_proc_timer = 0

        self.move_dir = new_dir
        if self.switch_img_proc:
            msg = self.cal_publish_move_route(self.move_dir)
            self.publish_move_direction(str(msg))

# callback functions end========================================================================================
    
    def cal_publish_move_route(self, dir):
        msg = ''
        point = None
        dis = 2.5
        if dir == self.list_move_dir.DIR_DONE:
            return 'DONE'
        elif dir == self.list_move_dir.DIR_DOWN:
            return 'LAND'
        elif dir == self.list_move_dir.DIR_NONE:
            return 'NONE'
        elif dir == self.list_move_dir.DIR_NW:
            point = Point(-dis, dis)
        elif dir == self.list_move_dir.DIR_NOTRH:
            point = Point(0, dis)
        elif dir == self.list_move_dir.DIR_NE:
            point = Point(dis, dis)
        elif dir == self.list_move_dir.DIR_WEST:
            point = Point(-dis, 0)
        elif dir == self.list_move_dir.DIR_EAST:
            point = Point(dis, 0)
        elif dir == self.list_move_dir.DIR_SW:
            point = Point(-dis, -dis)
        elif dir == self.list_move_dir.DIR_SOUTH:
            point = Point(0, -dis)
        elif dir == self.list_move_dir.DIR_SE:
            point = Point(dis, -dis)
        
        if point != None:
            angle = -self.orientation
            point.rotate(angle)
            point.to_int()
            msg = str(point.y) + ',' + str(-point.x)
            return msg
        else:
            return 'NONE'
    # 转动摄像头
    def turn_camera(self, angle):
        camera_angle_req = Float32()
        camera_angle_req.data = float(angle)
        self.camera_pub.publish(camera_angle_req)
        # self.get_logger().debug('Image_Processor_Topic: Set camera angle to {}'.format(camera_angle_req.data))

    # 发布移动方向信息
    def publish_move_direction(self, direction):
        if not self.switch_img_proc:
            return
        self.dir_msg = String()
        self.dir_msg.data = str(direction)
        self.move_dir_pub.publish(self.dir_msg)
        # self.get_logger().debug('Image Processor: Drone Should Move To {}'.format(self.dir_msg.data)) 
  
    # 判断无人机是否静止
    def is_drone_still(self):
        threshold_still = 0.03
        if self.cur_vel.linear.x > threshold_still or self.cur_vel.linear.x < -threshold_still:
            return False
        elif self.cur_vel.linear.y > threshold_still or self.cur_vel.linear.y < -threshold_still:
            return False
        elif self.cur_vel.linear.z > threshold_still or self.cur_vel.linear.z < -threshold_still:
            return False
        elif self.cur_vel.angular.x > threshold_still or self.cur_vel.angular.x < -threshold_still:
            return False
        elif self.cur_vel.angular.y > threshold_still or self.cur_vel.angular.y < -threshold_still:
            return False
        elif self.cur_vel.angular.z > threshold_still or self.cur_vel.angular.z < -threshold_still:
            return False
        else:
            return True

    # todo 
    # 图像识别的反馈模块
    # 返回一个计算出的方向
    def image_process(self):
        self.move_dir = self.list_move_dir.DIR_NONE

        if not self.switch_img_proc:
            self.move_dir = self.list_move_dir.DIR_NONE
            
        elif not self.is_drone_still():
            self.get_logger().info('Image Processor: Drone is not still, quit')
            self.move_dir = self.list_move_dir.DIR_NONE
            
        else:
            self.get_logger().info('Image Processor: Calculating move direction==============================')
            
            # 将摄像头调整到垂直向下
            self.turn_camera(1.57)

            if self.myMode == self.list_mode.MODE_RED:
                self.get_logger().info('Image Processor: red mode')
                self.move_dir = self.cal_route_mode_red()
                
            elif self.myMode == self.list_mode.MODE_YELLOW:
                self.get_logger().info('Image Processor: yellow mode')
                self.move_dir = self.cal_route_mode_yellow()
                
            else:
                self.get_logger().info('Image Processor: error mode')
                self.move_dir = self.list_move_dir.DIR_NONE
                

        # if self.img_proc_timer > 60:
        #     self.move_dir = self.list_move_dir.DIR_NONE
        #     return self.move_dir

        
        return self.move_dir

    
    def cal_route_mode_red(self):
        dist = self.loca_camera.cal_distance(self.dest_point)

        if self.is_done_with_red() and dist > self.inner_circle_r_img and dist < self.excircle_r_img:
            self.move_dir = self.list_move_dir.DIR_DONE
            return self.move_dir

        self.calculate_all_move_destination()
        min = 100000
        target_min = None
        max = 0
        target_max = None

        # 获取每个可行目标点距离圆心的距离
        # 找到距离圆心最近的点，并且该点不会进入内园
        for i in range(0, len(self.list_dest)):
            if self.list_dest[i].invalid:
                continue
            temp = self.list_dest[i].cal_distance(self.dest_point)

            if temp < min and temp >= self.inner_circle_r_img:
                min = temp
                target_min = i
            if temp > max:
                max = temp
                target_max =i

        if target_min == None:
            if max < self.inner_circle_r_img:
                self.move_dir = self.get_dir_with_id(target_max)
            else:
                self.move_dir = self.list_move_dir.DIR_NONE
            return self.move_dir
        else:
            self.move_dir = self.get_dir_with_id(target_min)
            return self.move_dir
        
    def cal_route_mode_yellow(self):
        dist = self.loca_camera.cal_distance(self.dest_point)

        # 可以降落
        if self.is_safe_to_land() and dist > self.inner_circle_r_img and dist < self.excircle_r_img:
            self.move_dir = self.list_move_dir.DIR_DOWN
            return self.move_dir
        
        self.calculate_all_move_destination()
        min = 100000
        target_min = None
        max = 0
        target_max = None

        # 获取每个可行目标点距离圆心的距离
        # 找到距离圆心最近的点，并且该点不会进入内园
        for i in range(0, len(self.list_dest)):
            if self.list_dest[i].invalid:
                continue
            temp = self.list_dest[i].cal_distance(self.dest_point)

            if temp < min and temp >= self.inner_circle_r_img:
                min = temp
                target_min = i
            if temp > max:
                max = temp
                target_max =i

        if target_min == None:
            if max < self.inner_circle_r_img:
                self.move_dir = self.get_dir_with_id(target_max)
            else:
                self.move_dir = self.list_move_dir.DIR_NONE
            return self.move_dir
        else:
            self.move_dir = self.get_dir_with_id(target_min)
            return self.move_dir

    def get_dir_with_id(self, id):
        if id == 0:
            return self.list_move_dir.DIR_NW
        elif id== 1:
            return self.list_move_dir.DIR_NOTRH
        elif id== 2:
            return self.list_move_dir.DIR_NE
        elif id== 3:
            return self.list_move_dir.DIR_WEST
        elif id== 4:
            return self.list_move_dir.DIR_EAST
        elif id== 5:
            return self.list_move_dir.DIR_SW
        elif id== 6:
            return self.list_move_dir.DIR_SOUTH
        elif id== 7:
            return self.list_move_dir.DIR_SE
        else:
            return self.list_move_dir.DIR_NONE

    def publish_img(self, msg):
        self.img_pub.publish(self.br.cv2_to_imgmsg(msg,'bgr8'))

    # 图像处理，标记边界
    def image_callback(self,msg):
        img = self.br.imgmsg_to_cv2(msg,'bgr8')
        img_output = None
        self.list_area_red.clear()
        self.list_area_yellow.clear()

        img_output = self.draw_contours(img, img_output, 'RED')
        img_output = self.draw_contours(img, img_output, 'YELLOW')

        if(self.calculate_all_move_destination()):
            
            # self.get_logger().info('Image Processor: dest:{}  {}'.format(self.dest_point.x, self.dest_point.y))

            img_output = self.draw_possible_dest(img_output)
            img_output = self.draw_dest(img_output)

        self.img_pub.publish(self.br.cv2_to_imgmsg(img_output,'bgr8'))   
       
                
    def draw_contours(self, raw, output, mode):
        # 高斯滤波
        gs_img = cv2.GaussianBlur(raw, (5, 5), 0)
        # 将其转换为HSV色彩空间图喜爱嗯
        hsv_image= cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)
        # 侵蚀，突出边界
        erode_msg=cv2.erode(hsv_image, None, iterations=4)

        if mode == 'RED':
            # 设置阈值，突出背景部分，为2值图
            mask = cv2.inRange(erode_msg,self.lower_red,self.upper_red)
        else:
            # 设置阈值，突出背景部分，为2值图
            mask = cv2.inRange(erode_msg,self.lower_yellow,self.upper_yellow)

        # 检测轮廓，RETER——EXTERNAL只检测外轮廓，CHAIN_APPROX_SIMPLE只保留基础轮廓位置，如矩形只有四个顶点
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # 绘制边界， -1表示绘制所有轮廓，轮廓颜色为黑
        output = cv2.drawContours(raw, cnts, -1, (0, 0, 0), 2)

        if len(cnts) > 0:
            for i in cnts:
                rect = cv2.minAreaRect(i)
                box = cv2.boxPoints(rect)
                if mode == 'RED':
                    self.list_area_red.append(box.tolist())
                else:
                    self.list_area_yellow.append(box.tolist())
                box = np.int0(box)
                output = cv2.drawContours(output, [box], 0, (255, 0, 0), 2)
                
        return output
         
    def draw_possible_dest(self, output):
        # 绘制无人机阴影
        output = cv2.circle(output, (int(320), int(240)), int(self.radius_shadow), (255, 255, 255), 4)

        # 绘制目标点
        for point in self.list_dest:
            # 可行的点设为蓝色
            if not point.invalid:
                output = cv2.circle(output, (int(point.x), int(point.y)), 6, (255, 0, 0), -1)
            # 不可行的点设为黑色
            else:
                output = cv2.circle(output, (int(point.x), int(point.y)), 6, (0, 0, 0), -1)
        # self.get_logger().info('Image Processor: draw destination')
        return output

    def draw_dest(self, output):
        # # 绘制内环
        output = cv2.circle(output, (int(self.dest_point.x), int(self.dest_point.y)), int(self.inner_circle_r_img), (255, 255, 255), 4)
        # # 绘制外环
        output = cv2.circle(output, (int(self.dest_point.x), int(self.dest_point.y)), int(self.excircle_r_img), (255, 255, 255), 4)

        dest_radius = 5
        
        if(self.dest_point.x < 640 and self.dest_point.y < 640 and self.dest_point.x > 0 and self.dest_point.y > 0):
            output = cv2.circle(output, (int(self.dest_point.x), int(self.dest_point.y)), int(dest_radius), (255, 255, 255), 4)
        else:
            x = self.dest_point.x
            y = self.dest_point.y 
            x = 640 if x >= 640 else x
            x = 0 if x <= 0 else x
            y = 480 if y >= 480 else y
            y = 0 if y <=0 else y
            output = cv2.circle(output, (int(x), int(y)), int(dest_radius), (255, 255, 255), 4)
        return output

    # 计算出8个移动方向的目标点位置
    # 找出目标点中不在违规区域的点
    def calculate_all_move_destination(self):
        if self.myMode == self.list_mode.MODE_NONE:
            return False

        # 每次计算需要重新初始化所有的目标点，所有目标点初始时必须是invalid为False
        for point in self.list_dest:
            point.reset(320, 240)
        
        # 无人机高度
        h = self.local_pos.pose.position.z
        # self.get_logger().info('Image Processor: current height is {}'.format(h))
        if h <=0 or self.myMode == self.list_mode.MODE_NONE:
            return False

        # 无人机移动距离,单位m
        move_len = 2.5

        # 无人机摄像头视野的实际高度
        self.img_height = h / 0.53 

        # 无人机摄像头视野的实际宽度
        self.img_width = 4 / float(3) * self.img_height 
        # 移动目标点在摄像头图像x轴的偏移量
        offset_x = float(640) / self.img_width * move_len
        # 移动目标点在摄像头图像y轴的偏移量
        offset_y = float(480) / self.img_height * move_len

        
        self.cal_drone_shadow(self.img_width)
        self.cal_dest_pos_in_camera(self.img_width, self.img_height)
        self.cal_inner_circle_radius(self.img_width)
        self.cal_excircle_radius(self.img_width)

        self.list_dest[0].sum(-offset_x,  -offset_y)
        self.list_dest[1].sum(0,          -offset_y)
        self.list_dest[2].sum(offset_x,   -offset_y)
        self.list_dest[3].sum(-offset_x,  0)
        self.list_dest[4].sum(offset_x,   0)
        self.list_dest[5].sum(-offset_x,  offset_y)
        self.list_dest[6].sum(0,          offset_y)
        self.list_dest[7].sum(offset_x,   offset_y)

        for p in self.list_dest:
            p.to_int()
        self.is_dest_in_invalid_area()

        return True
    
    # 计算相机图像坐标系下目标点的位置
    def cal_dest_pos_in_camera(self, width, height):
        dist = self.global_loca.get_distance(self.dest_loca)
        angle = self.global_loca.cal_angle(self.dest_loca)

        # self.get_logger().info('Image Processor: dist{}  angle{}'.format(dist, angle))

        angle_a = - self.orientation * 180
        theta = angle + angle_a

        offset_x = dist * sin(math.radians(theta)) /width * 640
        offest_y = - dist * cos(math.radians(theta)) / height * 480
       
        self.dest_point.reset(320 + offset_x.real, 240 + offest_y.real)
        self.dest_point.to_int()

    def cal_inner_circle_radius(self, width):
        self.inner_circle_r_img = self.inner_circle_r / width * 640

        self.inner_circle_r_img = int(self.inner_circle_r_img)
        
    def cal_excircle_radius(self, width):
        self.excircle_r_img = self.excircle_r / width * 640

        self.excircle_r_img = int(self.excircle_r_img)
        
    def cal_drone_shadow(self, width):
        self.radius_shadow = self.radius_drone / width * 640

    # 判断目标点是否在红色或黄色区域中
    def is_dest_in_invalid_area(self):
        
        # 红色模式或者黄色模式，都要计算红色区域
        if self.myMode == self.list_mode.MODE_RED or self.myMode == self.list_mode.MODE_YELLOW:
            # 针对每个目标点
            for i in range(0, len(self.list_dest)):
                for area in self.list_area_red:
                    
                    if self.is_point_in_rect(self.list_dest[i], area):
                        self.list_dest[i].invalid = True
                        break
        if self.myMode == self.list_mode.MODE_YELLOW:
            for i in range(0, len(self.list_dest)):
                if self.list_dest[i].invalid:
                    continue
                for area in self.list_area_yellow:
                    if self.is_point_in_rect(self.list_dest[i], area):
                        self.list_dest[i].invalid = True
                        break
       
    def is_done_with_red(self):
        for red in self.list_area_red:
            if self.is_point_in_rect(self.loca_camera, red):
                return False
        return True
    def is_safe_to_land(self):
        for red in self.list_area_red:
            if self.is_point_in_rect(self.loca_camera, red):
                return False
        for yellow in self.list_area_yellow:
            if self.is_point_in_rect(self.loca_camera, yellow):
                return False
        return True

    def is_point_in_rect(self, p, list):
        p0 = Point(0, 0)
        p1 = Point(0, 0)
        p2 = Point(0, 0)
        p3 = Point(0, 0)

        p0.x = float(list[0][0])
        p0.y = float(list[0][1])
        p1.x = float(list[1][0])
        p1.y = float(list[1][1])
        p2.x = float(list[2][0])
        p2.y = float(list[2][1])
        p3.x = float(list[3][0])
        p3.y = float(list[3][1])

        p0.to_int()
        p1.to_int()
        p2.to_int()
        p3.to_int()
        
        if self.get_cross(p1, p2, p) * self.get_cross(p3, p0, p) >=0 and self.get_cross(p2, p3, p) * self.get_cross(p0, p1, p) >=0:
            return True

    def get_cross(self, p1, p2, p):
        return (float(p2.x) - p1.x) * (float(p.y) - p1.y) - (float(p.x) - p1.x) * (float(p2.y) - p1.y)




def main(args=None):
    rclpy.init(args=args)
    image_node= Imageprocessor()
    image_node.start()
    rclpy.spin(image_node)
    

if __name__ == '__main__':
    main()

