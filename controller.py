import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist

from std_msgs.msg import Int32MultiArray

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong

from math import radians, cos, sin, asin, sqrt

class DroneState(object):

    # identify values
    STATE_INIT = 'Init'
    STATE_CHECKING = 'Checking'
    STATE_ARMING = 'Arming'
    STATE_WAIT_FOR_USER = 'Wait_for_user'
    STATE_CLIMBING = 'Climbing'
    STATE_TAKEOFF = 'Takeoff'
    STATE_AUTO_MOVE_TO_DESTINATION = 'Auto move to destination'
    STATE_FIND_SAFE_LAND = 'Finding safe land'
    STATE_LANDING = 'Landing'
    STATE_RETURN = 'Return_home'
    STATE_EXIT = 'Exit'
    STATE_RISK_MANAGE = 'Risk manage'
    STATE_STOP = 'Stop'

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('controller')
        self.state = DroneState()
        self.control_state = 'init'
        self.user_state = 'auto'   # manual

        self.last_status = None     # store for last received status message
        self.last_pos = None       # store for last received position message
        self.init_alt = None       # store for global altitude at start
        self.last_alt_rel = None   # store for last altitude relative to start
        # create service clients for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
        # ... for mode changes ...
        self.mode_cli = self.create_client(SetMode, '/vehicle_1/mavros/set_mode')
        # ... for arming ...
        self.arm_cli = self.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
        # ... and for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')

        self.landing_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/land')
        # create publisher for setpoint
        self.target_pub = self.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()
        # initial state for finite state machine
        # self.control_state = 'init'
        # timer for time spent in each state
        self.state_timer = 0
        # multi goal position, original: 51.4233628, -2.671761
        # self.goal_position = [[51.4234178, -2.6715506], [51.4219206, -2.6687700]]
        self.original_position = [51.4234178, -2.6715506]
        # self.target_position = [51.4219206, -2.6687700]
        self.target_position = [51.4214509, -2.6693021]
        self.goal_position = []
        self.target_dis = 65

        # create publisher for control velocity
        self.velocity_pub = self.create_publisher(Twist, '/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.velocity = Twist()

        self.controller_path_planning_pub = self.create_publisher(String, '/controller_path_planning', 10)
        self.controller_path_planning_msg = String()

        self.controller_interface_pub = self.create_publisher(String, '/controller_interface', 10)
        self.controller_interface_msg = String()

        self.controller_interface_annulus_pub = self.create_publisher(String, '/controller_interface/annulus', 10)
        self.controller_interface_annulus_msg = String()

        self.controller_risk_pub = self.create_publisher(String, '/controller_risk', 10)
        self.controller_risk_msg = String()

        self.controller_risk_init_alt_pub = self.create_publisher(String, '/controller_risk/init_alt', 10)
        self.controller_risk_init_alt_msg = String()

        self.controller_image_process_pub = self.create_publisher(String, '/controller_image_process', 10)
        self.controller_image_process_msg = String()

        self.setting_alt = None

        self.check_state = 'arm'

        self.annulus_state = 'out_of_annulus' # 'in_annulus'

        self.task_state = 'task_not_completed' # task_completed

        self.velocity_control_list = []

        self.risk_alarm = 'unknow'  # risk状态初始化为未知

        #为了测试状态机
        self.controller_foxyglove_pub = self.create_publisher(String, '/controller_foxyglove', 10)

        # 为测试
        self.flag = 0

    def start(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)
        # create subscriber for risk management
        risk_controller_sub = self.create_subscription(String, '/vehicle_1/risk_alarm_state', self.risk_alarm_callback ,10)

        interface_controller_alt_sub = self.create_subscription(String, '/interface_controller/alt', self.interface_controller_alt_callback, 10)

        interface_controller_sub = self.create_subscription(String, '/interface_controller', self.interface_controller_msg_callback, 10)

        path_planning_controller_sub = self.create_subscription(String, '/path_planning_controller', self.path_planning_controller_msg_callback, 10)

        moving_info_sub = self.create_subscription(String, '/vehicle_1/camera/moving_info', self.moving_info_callback, 10)

        image_process_controller_sub = self.create_subscription(String, '/image_process_controller', self.image_process_controller_msg_callback, 10)

        pos_t_sub = self.create_subscription(Int32MultiArray, '/pos_test', self.pos_t_callback, 10)

        # 仅为测试
        foxyglove_controller_sub = self.create_subscription(String, '/foxyglove_controller', self.foxyglove_controller_msg_callback, 10)

        # create a ROS2 timer to run the control actions
        self.timer = self.create_timer(1.0, self.timer_callback)

    # 仅测试
    def foxyglove_controller_msg_callback(self, msg):
        self.control_state = msg.data

    def image_process_controller_msg_callback(self, msg):
        if msg.data == 'LAND':
            self.control_state = 'landing'
            self.state_timer = 0
            self.task_state = 'task_completed'
        elif msg.data == 'NONE':
            pass
        elif msg.data == 'DONE':
            self.controller_image_process_msg.data = 'YELLOW'
            self.controller_image_process_pub.publish(self.controller_image_process_msg)
        else:
            # 假设数据为'1,0',前面为x=,后面为y=,也就是'x=1,y=0'
            velocity_msg = msg.data.split(',')
            velocity_x = float(velocity_msg[0])
            velocity_y = float(velocity_msg[1])
            velocity_xy = [velocity_x, velocity_y]
            self.velocity_control_list.append(velocity_xy)
            self.velocity_control(velocity_x, velocity_y, 0.0, 0.0, 0.0, 0.0)

    def moving_info_callback(self, msg):
        pass

    def pos_t_callback(self, msg):
        pass

    def path_planning_controller_msg_callback(self, msg):
        if msg.data == 'return home path planning finished':
            self.check_state = 'goal_position_check'
            self.control_state = 'checking'
            self.state_timer = 0
        else:
            # init_x = -2.67155
            # init_y = 51.42341
            pos_msg = msg.data.split(",")
            pos_x = (float)(pos_msg[0])
            pos_y = (float)(pos_msg[1])
            # if abs(pos_x - init_x) > 0.001 or abs(pos_y - init_y) > 0.001:
            pos_xy = [pos_y, pos_x]
            self.goal_position.append(pos_xy)

    def interface_controller_msg_callback(self, msg):
        pass

    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_status = msg
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                        msg.longitude,
                                                                        self.last_alt_rel))

    def risk_alarm_callback(self, msg):
        if self.risk_alarm != msg.data:
            self.risk_alarm = msg.data
            if msg.data == '-1':
                self.check_state = 'alt'
                self.control_state = 'checking'
                self.state_timer = 0
            elif msg.data == '1':
                self.get_logger().info('Flyover border')
                # self.control_state = 'stop'
                # self.state_timer = 0
            elif msg.data == '2':
                self.get_logger().info('Low battery')
                # self.task_state == 'task_completed'
                # self.control_state = 'return_home'
                # self.state_timer = 0

    def interface_controller_alt_callback(self, msg):
        if self.risk_alarm == '-1': # 如果之后改了risk的逻辑，这里也要改
            # self.setting_alt = float(msg.data)
            # 为测试
            self.setting_alt = 20.0
            self.control_state = 'arming'
            self.change_mode("GUIDED")
            self.state_timer = 0

    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        self.get_logger().info('Requested msg {} every {} us'.format(msg_id,msg_interval))

    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        self.get_logger().info('Request sent for {} mode.'.format(new_mode))

    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        self.get_logger().info('Arm request sent')

    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        self.get_logger().info('Requested takeoff to {}m'.format(target_alt))

    def landing(self, target_alt):
        landing_req = CommandTOL.Request()
        landing_req.altitude = target_alt
        future = self.landing_cli.call_async(landing_req)
        self.get_logger().info('Requested landing to {}m'.format(target_alt))

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        self.target_pub.publish(self.last_target)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 

    def velocity_control(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.velocity.linear.x = float(linear_x)
        self.velocity.linear.y = float(linear_y)
        self.velocity.linear.z = float(linear_z)
        self.velocity.angular.x = float(angular_x)
        self.velocity.angular.y = float(angular_y)
        self.velocity.angular.z = float(angular_z)
        self.velocity_pub.publish(self.velocity)        

    def geo_distance(self, lon1, lat1, lon2, lat2):
        lon1, lat1, lon2, lat2 = map(radians, [float(lon1), float(lat1), float(lon2), float(lat2)]) # 经纬度转换成弧度
        dlon=lon2-lon1
        dlat=lat2-lat1
        a=sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        distance=2*asin(sqrt(a))*6371*1000*1000 # 地球平均半径，6371km
        distance=round(distance/1000,3)
        return distance

    def state_transition(self):
        if self.control_state == 'init':
            if self.last_status:
                if self.last_status.system_status==3:
                    self.get_logger().info('Drone initialized')
                    # send command to request regular position updates
                    self.request_data_stream(33, 1000000) # global

                    self.request_data_stream(32, 1000000)

                    self.request_data_stream(147, 10000)

                    self.request_data_stream(26, 1000000)

                    self.request_data_stream(31, 1000000)

                    self.request_data_stream(105, 1000000)
                    # self.request_data_stream(135, 1000000)


                    # change mode to GUIDED
                    self.change_mode("GUIDED")

                    self.controller_interface_msg.data = 'init finished'
                    self.controller_interface_pub.publish(self.controller_interface_msg)
                    return('checking')
                else:
                    return('init') 
            else:
                return('init')

        elif self.control_state == 'checking':
            if self.check_state == 'arm':
                self.controller_risk_msg.data = 'arm check'
                self.controller_risk_pub.publish(self.controller_risk_msg)
                return('wait_for_user')
            
            elif self.check_state == 'alt':
                self.controller_interface_msg.data = 'alt check'
                self.controller_interface_pub.publish(self.controller_interface_msg)
                return('wait_for_user')

            elif self.check_state == 'annulus_check':
                if self.annulus_state == "out_of_annulus":
                    self.check_state = 'goal_position_check'
                    return('checking')
                else:
                    return('fly_out_of_annulus')

            elif self.check_state == 'goal_position_check':
                if len(self.goal_position) == 0:
                    self.get_logger().info('No goal position')
                    return('hover')
                else:
                    current_goal_position = self.goal_position[0]
                    self.flyto(current_goal_position[0], current_goal_position[1], self.init_alt + self.setting_alt - 50.0)
                    return('on_way')  
            elif self.state_timer > 60: # 目前没想到有什么情况会checking超时
                return('exit')

        elif self.control_state == 'hover':
            # d_lat = self.last_pos.latitude - self.original_position[0]     
            # d_lon = self.last_pos.longitude - self.original_position[1]
            # if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
            #     return('landing')
            if self.task_state == 'task_completed':
                return('landing')
            else:
                self.controller_image_process_msg.data = 'RED'
                self.controller_image_process_pub.publish(self.controller_image_process_msg)
                return('image_process')            

        elif self.control_state == 'wait_for_user':
            if self.state_timer > 6000: #仅演示，实际为3min即180
                d_lat = self.last_pos.latitude - self.original_position[0]     
                d_lon = self.last_pos.longitude - self.original_position[1]
                if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
                    if self.last_alt_rel:
                        if self.last_alt_rel > 1.0:
                            return('landing')
                        else:
                            return('exit')
                    else: # last_alt_rel为None，说吗arming都没进，直接exit
                        return('exit')
                else:
                    if self.annulus_state == 'in_annulus':
                        if self.last_alt_rel < 1.0:  #这里需要判断是否在地上
                            self.setting_alt = 20.0
                            return('arming')
                        else:
                            return('fly_out_of_annulus')
                    else:
                        if self.last_alt_rel < 1.0:
                            self.setting_alt = 20.0
                            return('arming')
                        else:
                            return('return_home')
            else:
                return 'wait_for_user'

        elif self.control_state == 'arming':
            if self.last_status.armed:
                self.get_logger().info('Arming successful')
                if self.last_pos:
                    self.last_alt_rel = 0.0
                    self.init_alt = self.last_pos.altitude
                    self.controller_risk_init_alt_msg.data = str(self.init_alt)
                    self.controller_risk_init_alt_pub.publish(self.controller_risk_init_alt_msg)
                return('takeoff')
                # armed - grab init alt for relative working
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to arm')
                return('exit')
            else:
                self.arm_request()
                return('arming')

        # publisher-->topic-->subscriber
        # host -- serice --  

        elif self.control_state == 'takeoff':
            # send takeoff command
            if self.setting_alt:
                self.takeoff(self.setting_alt)
                return('climbing') # !!!path planning checking
            elif self.state_timer > 60:
                return('exit')
            else:
                self.get_logger().info('no alt data')
                return('exit')

        elif self.control_state == 'climbing':
            if self.last_alt_rel > self.setting_alt - 1.0:
                self.get_logger().info('Close enough to flight altitude')
                if self.flag == 0:
                    self.check_state = 'annulus_check'
                    return('checking')
                # 为测试
                else:
                    return('return_home')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to reach altitude')
                return('landing')
            else:
                self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
                return('climbing')

        elif self.control_state == 'on_way':
            if self.task_state == 'task_not_completed':
                dis = self.geo_distance(self.last_pos.longitude, self.last_pos.latitude, self.target_position[1], self.target_position[0])
                if dis > self.target_dis - 1.0 and dis < self.target_dis + 1.0:
                    self.velocity_control(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    self.goal_position = []
                    return('hover')

            d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
                self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                del self.goal_position[0]
                self.check_state = 'goal_position_check'
                return('checking')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to reach target')
                return('return_home') # !!! 可以改成让用户决定
            else:
                self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
                return('on_way')

        elif self.control_state == 'landing':
            if self.state_timer > 60:
                return('exit')
            else:
                self.landing(0.0)
                return('landing_down')

        elif self.control_state == 'landing_down':
            if self.last_alt_rel < 1:
                self.get_logger().info('Close enough to ground')
                if self.last_status.armed == False:
                    # 为测试
                    self.flag = 1
                    self.task_state = 'task_completed'
                    if self.annulus_state == 'in_annulus':
                        self.controller_interface_annulus_msg.data = 'in_annulus landing finished'
                        self.controller_interface_annulus_pub.publish(self.controller_interface_annulus_msg)
                        return('wait_for_user')
                    else:
                        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        return('wait_for_user')# 为测试改为的wait for user 
                        # return('exit')
                else:
                    return('landing_down')
            elif self.state_timer > 60:
                # timeout
                self.get_logger().error('Failed to land')
                return('landing')
            else:
                self.get_logger().info('Landing, altitude {}m'.format(self.last_alt_rel))
                return('landing_down')

        elif self.control_state == 'image_process':
            dis = self.geo_distance(self.last_pos.longitude, self.last_pos.latitude, self.target_position[1], self.target_position[0])
            if dis > 35.0 + 3.0 and dis < 50.0 - 3.0:
                self.annulus_state = 'in_annulus'
                # self.controller_image_process_msg.data = 'in annulus'
                # self.controller_image_process_pub.publish(self.controller_image_process_msg)
            elif self.state_timer > 600:
                if self.annulus_state == 'in_annulus':
                    return('fly_out_of_annulus')
                else:
                    return('return_home')

            return('image_process')

        elif self.control_state == 'fly_out_of_annulus':
            if len(self.velocity_control_list) > 0:
                self.velocity_control(self.velocity_control_list[-1][0], self.velocity_control_list[-1][1], 0.0, 0.0, 0.0, 0.0)
                del self.velocity_control_list[-1]
                return('fly_out_of_annulus')
            else:
                return('return_home')

        elif self.control_state == 'return_home':
            self.goal_position = []
            # ！！！ 需要auto mode去让用户不能打断
            # self.controller_path_planning_msg.data = 'return_home,{},{},{},{}'.format(self.last_pos.latitude, self.last_pos.longitude,
            #                                                                 self.original_position[0], self.original_position[1])
            self.controller_path_planning_msg.data = 'return_home,{},{}'.format(self.last_pos.latitude, self.last_pos.longitude)
            self.controller_path_planning_pub.publish(self.controller_path_planning_msg)
            return('wait_for_path_planning')

        elif self.control_state == 'wait_for_path_planning':
            if self.state_timer > 120: #给2分钟执行path planning, 以及传输数据
                return('exit') # 应该是转换为用户选择
            else:
                return('wait_for_path_planning')

        elif self.control_state == 'exit':
            # nothing else to do
            return('exit')

    def timer_callback(self):
        new_state = self.state_transition()
        if new_state == self.control_state:
            self.state_timer = self.state_timer + 1
        else:
            self.state_timer = 0
        self.control_state = new_state
        self.get_logger().info('Controller state: {} for {} steps'.format(self.control_state, self.state_timer))

        msg = String()
        msg.data = '{} for {}'.format(self.control_state, self.state_timer)
        self.controller_foxyglove_pub.publish(msg)

def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.start()
    rclpy.spin(controller_node)


if __name__ == '__main__':
    main()