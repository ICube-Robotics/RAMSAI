import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import socket

class ViscotecUDPReceiver(Node):
    def __init__(self):
        super().__init__('viscotec_velocity_observer')
        self.publisher_= self.create_publisher(Float64MultiArray,'/gpio_command_controller/commands',10)

        self.udp_port = 30005 # comme sur Java
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('', self.udp_port)) # Bind à toutes les IPs

        self.get_logger().info(f"[UDP] Ecoute sur port {self.udp_port}")
        self.create_timer(0.1,self.udp_listener)

    def udp_listener(self):
        self.sock.settimeout(0.01)
        try: 
            data, _ = self.sock.recvfrom(1024)
            msg_str = data.decode().strip()
            vm = float(msg_str)

            msg = Float64MultiArray()
            msg.data = [1.0, 0.0, vm]
            self.publisher_.publish(msg)
            self.get_logger().info(f"[UDP] Reçu vm={vm}, publié.")

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f"[UDP] Erreur : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ViscotecUDPReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
