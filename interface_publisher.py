import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TwistStamped

class Interface(Node):

    def __init__(self):
        super().__init__("interface_publisher")
        # create publisher for sending msg to interface to show
        self.interface_output_pub = self.create_publisher(String, '/vehicle_1/interface_controller', 10)
        self.interface_output_msg = String()

        self.interface_alt_pub = self.create_publisher(Float32, '/vehicle_1/interface_alt_output', 10)
        self.setting_altitude = 20.0

        self.interface_controller_pub = self.create_publisher(String, '/interface_controller/alt', 10)

        self.foxyglove_controller_pub = self.create_publisher(String, '/foxyglove_controller', 10)

    def start(self):
        controller_sub = self.create_subscription(String, '/vehicle_1/control_state', self.controller_callback, 10)

        interface_input_alt_sub = self.create_subscription(String, '/vehicle_1/interface_alt_input', self.interface_alt_input_callback,10)

        interface_set_alt_sub = self.create_subscription(Float32, 'vehicle_1/interface_set_alt', self.interface_set_alt_callback, 10)

        controller_foxyglove_sub = self.create_subscription(String, '/controller_foxyglove', self.controller_foxyglove_msg_callback, 10)

    def controller_foxyglove_msg_callback(self):
        pass

    def controller_callback(self, msg):
        pass
        # if msg.data == 'wait for user: alt':
        #     self.interface_output_msg.data = 'Please set altitude'
        #     self.interface_output_pub.publisher(self.interface_output_msg)

    def interface_alt_input_callback(self, msg):
        pass
        # if msg.data == 'init altitude':
        #     self.interface_alt_pub.publish(self.setting_altitude)

    def interface_set_alt_callback(self, msg):
        pass
        # if msg:
        #     self.interface_alt_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    interface_node = Interface()
    interface_node.start()
    rclpy.spin(interface_node)

if __name__ == '__main__':
    main()