import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import socket
import threading

class ViscotecVelocityObserver(Node):
    def __init__(self):
        super().__init__('viscotec_velocity_observer')
        self.publisher_= self.create_publisher(Float64MultiArray,'/gpio_command_controller/commands',10)

        #Lancer le serveur TCP dans un thread
        tcp_thread = threading.Thread(target=self.tcp_listener,daemon=True)
        tcp_thread.start()
        self.get_logger().info("viscotec_velocity_observer lancé et prêt (TCP) vers ROS2 lancé.")



    def tcp_listener(self):
        HOST = '192.170.10.5'
        PORT = 30000
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(1)

            self.get_logger().info(f"[TCP] En attente de connexion sur le {PORT}...")
            
            conn, addr = s.accept()
            self.get_logger().info(f"[TCP] Connecté à {addr}")


            while rclpy.ok():
                try:
                    data=conn.recv(1024).decode().strip()
                    if data:
                        try:
                            vm = float(data)
                            msg = Float64MultiArray()
                            msg.data = [1.0, 0.0, vm]
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"[TCP] Reçu vm={vm}, publié.")
                        except ValueError:
                            self.get_logger().warn(f"[TCP] Donnée invalide reçue: {data}")

                except Exception as e:
                    self.get_logger().error(f"[TCP] Erreur : {e}")
                    break

            conn.close
            s.close

def main(args=None):
    rclpy.init(args=args)
    node = ViscotecVelocityObserver()
    rclpy.spin(node)
    print("Wait for TCP thread to exit...")
    node.tcp_thread.join(timeout=1.0)
    print("All OK, bye!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
