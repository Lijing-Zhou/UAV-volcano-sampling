import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Interface(Node):

    def __init__(self):
        super().__init__("interface_publisher")
        # create publisher for message
        self.message_pub = self.create_publisher(String, '/hello/sub', 10)
        self.message_timer = 0

    def start(self):
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = '{}'.format(self.message_timer)
        self.message_pub.publish(msg)
        self.get_logger().info("send message successfully")
        self.message_timer += 1

def main(args=None):
    rclpy.init(args=args)

    interface_node = Interface()
    interface_node.start()
    rclpy.spin(interface_node)

if __name__ == '__main__':
    main()
