#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import numpy as np

class UELidarBridge(Node):
    def __init__(self):
        super().__init__('ue_lidar_bridge')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Объявляем параметры ROS 2
        self.declare_parameter('ue_ip', '192.168.0.159')
        self.declare_parameter('lidar_port', 12345)

        # Считываем значения из конфига
        self.ue_ip = self.get_parameter('ue_ip').get_parameter_value().string_value
        self.lidar_port = self.get_parameter('lidar_port').get_parameter_value().integer_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Увеличиваем системный буфер сокета, чтобы пакеты не терялись на уровне ядра
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        
        # Привязываем сокет к считанному порту
        self.sock.bind(('0.0.0.0', self.lidar_port))
        self.sock.setblocking(False)
        
        # Таймер на 100Гц для проверки сокета
        self.create_timer(0.01, self.receive_lidar) 
        self.get_logger().info(f"Lidar Bridge: Работаю. Порт: {self.lidar_port}. Ожидаю данные из Unreal...")

    def receive_lidar(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(8192) # Увеличим буфер на всякий случай
                if len(data) >= 720 * 4:
                    ranges = np.frombuffer(data, dtype=np.float32)[:720]

                    scan = LaserScan()
                    scan.header.stamp = self.get_clock().now().to_msg()
                    scan.header.frame_id = 'laser_frame'
                    scan.angle_min = 0.0
                    scan.angle_max = 2.0 * np.pi - (2.0 * np.pi / 720)
                    scan.angle_increment = (2.0 * np.pi) / 720
                    scan.range_min = 0.15
                    scan.range_max = 12.0
                    
                    # Отзеркаливание
                    scan.ranges = np.flip(ranges).tolist()
                    
                    self.publisher_.publish(scan)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f"Ошибка при получении данных: {e}")
                break

def main():
    rclpy.init()
    node = UELidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
