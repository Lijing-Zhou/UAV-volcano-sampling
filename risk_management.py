import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, BatteryState
from .risk_ass import RiskAssessment

class RiskInterface(Node):

    def __init__(self):
        super().__init__("risk_management")

        # create publisher for message
        self.risk_msg_pub = self.create_publisher(String, '/vehicle_1/risk_msg_output', 10)
        self.risk_alarm_pub = self.create_publisher(String, '/vehicle_1/risk_alarm_state', 10)
        self.risk_ass_state = RiskAssessment()
        self.risk_pub_msg = String()
        self.risk_state_msg = String()

        # position data
        self.last_pos = None
        self.init_alt = None
        self.last_alt_rel = None
        
        # battery data
        self.battery_power = None
        """
        risk alarm state:
                    -1: safe
                    1: border check failure 
                    2: battery check failure
                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    之后改成不要全部用-1, 而是border check success
        """

    def start(self):
        controller_risk_msg_sub = self.create_subscription(String, '/controller_risk', self.risk_msg_callback, 10)

        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)

        battery_sub = self.create_subscription(BatteryState, '/vehicle_1/mavros/battery', self.battery_callback, 10)

        controller_risk_init_alt_sub = self.create_subscription(String, '/controller_risk/init_alt', self.controller_risk_init_alt_msg_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def controller_risk_init_alt_msg_callback(self, msg):
        self.init_alt = float(msg.data)

    def risk_msg_callback(self, msg):
        if msg.data == 'arm check':
            self.get_logger().info('receiving the risk msg')
            # self.risk_pub_msg = self.risk_ass.arm_check()
            self.risk_pub_msg.data = 'Please check if the UAV can be armed.'
            self.risk_msg_pub.publish(self.risk_pub_msg)

        if msg.data == 'arm checked':
            self.risk_state_msg.data = '-1'
            self.risk_pub_msg.data = 'UAV is armed. Please set flying altitude. ==>'
            self.risk_msg_pub.publish(self.risk_pub_msg)
            self.risk_alarm_pub.publish(self.risk_state_msg)

    def timer_callback(self):
        if self.last_pos and self.last_alt_rel:
            # state = self.risk_ass_state.border_check(self.last_pos.latitude, self.last_pos.longitude, self.last_alt_rel)
            # self.get_logger().info('lat={} N, lon={} E, alt_rel={} m, border_check_state={}'.format(self.last_pos.latitude, self.last_pos.longitude, self.last_alt_rel, state))
            if self.risk_ass_state.border_check(self.last_pos.latitude, self.last_pos.longitude, self.last_alt_rel):
                if self.risk_state_msg.data == '1':
                    self.risk_state_msg.data = '-1'
                    self.risk_alarm_pub.publish(self.risk_state_msg)  # 为了延续之前的测试习惯，目前只想risk 有警告再发数据
            else:
                self.risk_state_msg.data = '1'
                self.risk_alarm_pub.publish(self.risk_state_msg)  # 为了延续之前的测试习惯，目前只想risk 有警告再发数据
        # else:
            # self.get_logger().info('position data error')

        if self.battery_power:
            if self.risk_ass_state.battery_check(self.battery_power):
                if self.risk_state_msg.data != '-1':
                    self.risk_state_msg.data = '-1'
                    # self.risk_alarm_pub.publish(self.risk_state_msg)  # 为了延续之前的测试习惯，目前只想risk 有警告再发数据
            else:
                self.risk_state_msg.data = '2'
                self.risk_alarm_pub.publish(self.risk_state_msg)  # 为了延续之前的测试习惯，目前只想risk 有警告再发数据
        # else:
            # self.get_logger().info('battery data error')

    def position_callback(self,msg):    
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt        
        self.last_pos = msg

    def battery_callback(self, msg):
        self.battery_power = msg.percentage

def main(args=None):
    rclpy.init(args=args)

    risk_management_node = RiskInterface()
    risk_management_node.start()
    rclpy.spin(risk_management_node)

if __name__ == '__main__':
    main()
