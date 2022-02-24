import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState

class RiskInterface(Node):

    def __init__(self):
        super().__init__("risk_management")
        # create publisher for message
        self.risk_msg_pub = self.create_publisher(String, '/vehicle_1/risk_msg_output', 10)
        self.risk_alarm_pub = self.create_publisher(String, '/vehicle_1/risk_alarm_state', 10)
        self.risk_ass = RiskManage()
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
            self.risk_pub_msg = self.risk_ass.arm_check()
            self.risk_msg_pub.publish(self.risk_pub_msg)
        
        if msg.data == 'arm checked':
            self.risk_state_msg.data = '-1'
            self.risk_alarm_pub.publish(self.risk_state_msg)

    def timer_callback(self):
        if self.last_pos and self.last_alt_rel:
            if self.risk_ass.border_check(self.last_pos.latitude, self.last_pos.longitude, self.last_alt_rel):
                self.risk_state_msg.data = '-1'
            else:
                self.risk_state_msg.data = '1'
        else:
            self.get_logger().info('position data error')

        if self.battery_power:
            if self.risk_ass.battery_check(self.battery_power):
                self.risk_state_msg.data = '-1'
            else:
                self.risk_state_msg.data = '2'
        else:
            self.get_logger().info('battery data error')

        

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

class RiskAssessment():

    def __init__(self):
        self.alt_border = 50

    # 3D border check
    def border_check(self, lat, lon, alt):
        """
        # index = 5
        # A, B, C ,D counter-clockwise
        self.flight_region = [[51.4234260, -2.6717208], [51.4212462, -2.6701340], 
                                [51.4224401, -2.6656878], [51.4246918, -2.6670602]]
        # A, B, C, D clockwise
        self.no_fly_zone = [[51.4224669, -2.6720597], [51.4235482, -2.6673723], 
                            [51.4228490, -2.6670013], [51.4217450, -2.6714016]]
        """
        # !!!it maybe could use safety_area

        # flight region border check
        lat_flight_ab = round((-137371 * lon * 100000 + 47753256419832) / 1000000000000, 7)
        if lat < lat_flight_ab:
            return False
        lat_flight_bc = round((26852 * lat * 100000 + 52138230581680) / 1000000000000, 7)
        if lat < lat_flight_bc:
            return False
        lat_flight_cd = round((-164070 * lat * 100000 + 47048846126540) / 1000000000000, 7)
        if lat > lat_flight_cd:
            return False
        lat_flight_da = round((27160 * lat * 100000 + 52149065369280) / 1000000000000, 7)
        if lat > lat_flight_da:
            return False
        
        # no fly zone border check
        lat_no_fly_ab = round((23068 * lon * 100000 + 52038857631596) / 1000000000000, 7)
        lat_no_fly_bc = round((23068 * lon * 100000 + 52038857631596) / 1000000000000, 7)
        lat_no_fly_cd = round((23068 * lon * 100000 + 52038857631596) / 1000000000000, 7)
        # lat_no_fly_da = round((23068 * lon * 100000 + 52038857631596) / 1000000000000, 7)
        if lat < lat_no_fly_ab and lat > lat_no_fly_cd and lat < lat_no_fly_cd:
            return False

        if alt > 50:
            return False

        return True

    """
    safty check:
            arm_check
    """
    def arm_check(self):
        msg = String()
        msg.data = 'Please check if the UAV can be armed'
        return msg

    """
    sensor check:
            battery
    """
    # battery check 30%
    def battery_check(self, battery_msg):
        if battery_msg.percentage < 0.3:
            return False
        return True
    
    """
    complete_failure:
            GPS failure
            IMU failure
            Barometer failure
    """
    def complete_failure(self):
        # do nothing
        pass
