import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix, BatteryState
from .risk_ass import RiskAssessment

class RiskInterface(Node):

    def __init__(self):
        super().__init__("risk_management")

        self.cmd_cli = self.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
        # create publisher for message
        self.risk_msg_pub = self.create_publisher(String, '/vehicle_1/risk_msg_output', 10)
        self.risk_alarm_pub = self.create_publisher(String, '/vehicle_1/risk_alarm_state', 10)
        self.risk_ass = RiskAssessment()
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
        """

    def start(self):
        risk_msg_sub = self.create_subscription(String, '/vehicle_1/risk_msg_input', self.risk_msg_callback, 10)

        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)

        battery_sub = self.create_subscription(BatteryState, '/vehicle_1/mavros/battery', self.battery_callback, 10)

        self.request_data_stream(33, 1000000)

        self.request_data_stream(147, 10000)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def risk_msg_callback(self, msg):
        if msg.data == 'init finished':
            if self.last_pos:
                self.init_alt = self.last_pos.altitude
                self.last_alt_rel = 0.0
            else:
                self.get_logger().info('risk init pos error')

        if msg.data == 'arm check':
            self.get_logger().info('receiving the risk msg')
            # self.risk_pub_msg = self.risk_ass.arm_check()
            self.risk_pub_msg.data = 'Please check if the UAV can be armed'
            self.risk_msg_pub.publish(self.risk_pub_msg)

        if msg.data == 'arm checked':
            self.risk_state_msg.data = '-1'
            self.risk_alarm_pub.publish(self.risk_state_msg)

    def timer_callback(self):
        pass
        # if self.last_pos and self.last_alt_rel:
        #     if self.risk_ass.border_check(self.last_pos.latitude, self.last_pos.longitude, self.last_alt_rel):
        #         self.risk_state_msg.data = '-1'
        #     else:
        #         self.risk_state_msg.data = '1'
        # else:
        #     self.get_logger().info('position data error')

        # if self.battery_power:
        #     if self.risk_ass.battery_check(self.battery_power):
        #         self.risk_state_msg.data = '-1'
        #     else:
        #         self.risk_state_msg.data = '2'
        # else:
        #     self.get_logger().info('battery data error')
        
    def position_callback(self,msg):
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt        
        self.last_pos = msg

    def battery_callback(self, msg):
        self.battery_power = msg.percentage

    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        self.get_logger().info('Requested msg {} every {} us'.format(msg_id,msg_interval))    

def main(args=None):
    rclpy.init(args=args)

    risk_management_node = RiskInterface()
    risk_management_node.start()
    rclpy.spin(risk_management_node)

if __name__ == '__main__':
    main()