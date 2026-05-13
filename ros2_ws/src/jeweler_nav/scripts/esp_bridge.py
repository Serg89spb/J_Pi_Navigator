#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import math
import time

class JewelerBridge(Node):
    def __init__(self):
        super().__init__('jeweler_bridge')
        
        # Настройки Serial
        self.port = '/dev/ttyACM0'
        self.baud = 115200
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Не удалось открыть порт: {e}")

        # Состояние одометрии (начинаем всегда с нуля)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # ROS Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.reset_sub = self.create_subscription(Empty, 'reset_odom', self.reset_callback, 10)
        
        # TF Broadcaster (odom -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Таймер опроса Serial (50Гц для запаса, хотя ESP шлет 20Гц)
        self.create_timer(0.02, self.read_serial)
        self.get_logger().info("Jeweler Bridge запущен на дельтах!")

    def cmd_callback(self, msg):
        # Отправляем скорости в ESP
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z # Радианы оставляем как есть
        
        command = f"v {vx:.2f} {vy:.2f} {omega:.2f}\n"
        self.ser.write(command.encode())

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('d'): # Ждем пакет дельт: d <dx> <dy> <dth>
                try:
                    parts = line.split()
                    dx = float(parts[1])
                    dy = float(parts[2])
                    dth = -float(parts[3])

                    # 1. Интегрируем дельты в глобальные координаты
                    # Применяем матрицу поворота, чтобы локальные dx/dy стали мировыми
                    self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
                    self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
                    self.th += dth

                    self.publish_odometry()
                except (ValueError, IndexError):
                    pass

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0, 0, self.th)

        # 1. Публикуем TF (важно для визуализации в Foxglove)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 2. Публикуем сообщение Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def reset_callback(self, msg):
        self.get_logger().info("Сброс одометрии запрошен!")
        # Обнуляем локальные координаты на Pi
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        # Шлем команду в ESP (префикс 'r' для reset)
        self.ser.write(b"r\n")

def main():
    rclpy.init()
    node = JewelerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
