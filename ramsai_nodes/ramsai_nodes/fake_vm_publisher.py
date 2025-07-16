import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class FakeVmPublisher(Node):
    def __init__(self):
        super().__init__('fake_vm_publisher')
        self.publisher_= self.create_publisher(Float64MultiArray,'/gpio_command_controller/commands',10)
        self.timer = self.create_timer(1.0, self.publish_fake_vm)
        self.counter = 0.0
        self.get_logger().info("FakeVmPublisher lancé .")

    

    def publish_fake_vm(self):
        vm = 150+50*math.sin(self.counter)
        self.counter += 0.3

        msg = Float64MultiArray()
        msg.data = [1.0,0.0,vm]
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publié : [1.0,0.0,{vm:.2f}]")

def main(args=None):
    rclpy.init(args=args)
    node = FakeVmPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
    
