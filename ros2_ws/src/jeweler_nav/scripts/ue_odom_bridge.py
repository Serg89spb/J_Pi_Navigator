#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import tf2_ros
import math

class JewelerBridge(Node):
    def __init__(self):
        super().__init__('jeweler_bridge')
        
        # Настройки UDP
        self.udp_ip = "0.0.0.0"
        self.odom_port = 12346
        self.ue_ip = "192.168.0.159" # IP Unreal
        self.ue_port = 12347

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.odom_port))
        self.sock.setblocking(False)

        # Состояние
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        
        # Паблишеры
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Подписка на команды (возврат в UE)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        self.create_timer(0.01, self.read_udp)
        self.get_logger().info("Odom Bridge")

    def cmd_callback(self, msg):
        # Отправляем скорости НАЗАД в Unreal
        command = f"v {msg.linear.x:.2f} {msg.linear.y:.2f} {msg.angular.z:.2f}\n"
        try:
            self.sock.sendto(command.encode(), (self.ue_ip, self.ue_port))
        except Exception as e:
            self.get_logger().error(f"Send to UE failed: {e}")

    def read_udp(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                line = data.decode('utf-8').strip()
                
                # Парсим старый добрый формат "d dx dy dth"
                if line.startswith('d') or 'd' in line:
                    parts = line.split()
                    # Ищем где буква 'd', данные сразу после нее
                    try:
                        idx = parts.index('d')
                        dx = float(parts[idx+1])
                        dy = float(parts[idx+2])
                        dth = float(parts[idx+3])
                    except: continue

                    # Твоя рабочая формула интеграции
                    dx_ros = dx; dy_ros = -dy; dth_ros = -dth
                    self.th += dth_ros
                    self.x += dx_ros * math.cos(self.th) - dy_ros * math.sin(self.th)
                    self.y += dx_ros * math.sin(self.th) + dy_ros * math.cos(self.th)
                    
                    self.publish_odometry()
            except BlockingIOError:
                break

    def publish_odometry(self):
        now = self.get_clock().now().to_msg() # системное время
        q = self.euler_to_quaternion(0.0, 0.0, self.th)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        return Quaternion(w=cr*cp*cy+sr*sp*sy, x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy)

def main():
    rclpy.init()
    rclpy.spin(JewelerBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
