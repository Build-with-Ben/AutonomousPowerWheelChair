import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame

class SimpleTxNode(Node):
    def __init__(self):
        super().__init__('simple_tx_node')
        self.publisher = self.create_publisher(Frame, '/to_can_bus', 10)
        self.timer = self.create_timer(1.0, self.send_frame)  # 1 Hz to start
        self.counter = 0

    def send_frame(self):
        msg = Frame()
        msg.id = 0x123
        msg.is_extended = False
        msg.dlc = 8
        msg.data = [self.counter, 0, 0, 0, 0, 0, 0, 0]
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent frame id=0x123 counter={self.counter}')
        self.counter = (self.counter + 1) % 256

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()