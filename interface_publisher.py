import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TwistStamped

class Interface(Node):

    def __init__(self):
        super().__init__("interface_publisher")
        
        # create client
        self.cmd_cli = self.create_client(CommandLong,'Vehicle_1/mavros/cmd/command')
        
        # create publisher for sending msg (alt setting) to interface
        self.alt_msg_pub = self.create_publisher(String, '/vehicle_1/alt_msg_output', 10)
   
        self.alt_pub_msg = String()


        # self.interface_alt_pub = self.create_publisher(Float32, '/vehicle_1/interface_alt_output', 10)
        # self.setting_altitude = 20.0

        # self.interface_controller_pub = self.create_publisher(String, '/interface_controller/alt', 10)

        # self.foxyglove_controller_pub = self.create_publisher(String, '/foxyglove_controller', 10)

    def start(self):
        alt_msg_sub = self.create_subscription(String, '/vehicle_1/alt_msg_input', self.alt_msg_callback,10)
        
        self.request_data_stream(33, 1000000)

        self.timer = self.create_timer(0.1, self.timer_callback)   
        
        # controller_sub = self.create_subscription(String, '/vehicle_1/control_state', self.controller_callback, 10)

        # interface_input_alt_sub = self.create_subscription(String, '/vehicle_1/interface_alt_input', self.interface_alt_input_callback,10)

        # interface_set_alt_sub = self.create_subscription(Float32, 'vehicle_1/interface_set_alt', self.interface_set_alt_callback, 10)

        # controller_foxyglove_sub = self.create_subscription(String, '/controller_foxyglove', self.controller_foxyglove_msg_callback, 10)
        
        
    def alt_msg_callback(self, msg):
        # Valid input altitude 10~50m
        if(float(msg.data)>=10 and float(msg.data)<=50.0):
            self.setting_altitude = float(msg.data)
            self.alt_pub_msg.data = 'Setting successfully. Altitude:' + msg.data + 'm'
        else:
            self.alt_pub_msg.data = 'Please reset valid altitude(10~50m).'
            
        self.alt_msg_pub.publish(self.alt_pub_msg)


#    def controller_foxyglove_msg_callback(self):
#        pass


#    def interface_alt_input_callback(self, msg):
#        pass
#        # if msg.data == 'init altitude':
#       #     self.interface_alt_pub.publish(self.setting_altitude)

#    def interface_set_alt_callback(self, msg):
#        pass
        # if msg:
        #     self.interface_alt_pub.publish(msg)
        
    def timer_callback(self):
        pass


    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        self.get_logger().info('Requested msg {} every {} us'.format(msg_id,msg_interval))



def main(args=None):
    rclpy.init(args=args)

    interface_node = Interface()
    interface_node.start()
    rclpy.spin(interface_node)

if __name__ == '__main__':
    main()
