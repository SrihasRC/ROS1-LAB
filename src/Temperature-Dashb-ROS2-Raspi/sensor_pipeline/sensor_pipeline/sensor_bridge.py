# sensor_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import threading

HOST = '0.0.0.0'
PORT = 5005

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_data', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((HOST, PORT))
        self.sock.listen(1)
        self.get_logger().info(f"Listening on {HOST}:{PORT}")

        thread = threading.Thread(target=self.accept_connections)
        thread.daemon = True
        thread.start()

    def accept_connections(self):
        conn, addr = self.sock.accept()
        self.get_logger().info(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            try:
                temp, hum = map(float, data.decode().split(','))
                msg = Float32MultiArray(data=[temp, hum])
                self.publisher_.publish(msg)
                self.get_logger().info(f"Temp: {temp} °C Hum: {hum} %")
            except Exception as e:
                self.get_logger().error(f"Failed to parse data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
