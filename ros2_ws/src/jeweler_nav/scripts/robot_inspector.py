#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import subprocess
import time
import math

class RobotInspector(Node):
    def __init__(self):
        super().__init__('robot_inspector')
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Empty, '/sys/run_test', self.start_test_handler, 10)
        self.create_subscription(Empty, '/sys/hard_reset', self.hard_reset_handler, 10)
        
        # ========== НАСТРОЙКИ ТЕСТА ==========
        self.LINEAR_SPEED = 0.2    # м/с
        self.ANGULAR_SPEED = 0.9   # рад/с
        self.MOVE_TIME = 2.0       # сек на движение
        self.STOP_TIME = 1.0       # сек на паузу
        self.RESULT_DISPLAY_TIME = 20.0 # СКОЛЬКО СЕКУНД ГЛЯДЕТЬ НА РЕЗУЛЬТАТ
        # =====================================

        self.sequence = [
            (0.0, 0.0, self.ANGULAR_SPEED, "🔄 ПОВОРОТ +"), (0.0, 0.0, 0.0, "⏸️ СТОП"),
            (0.0, 0.0, -self.ANGULAR_SPEED, "🔄 ПОВОРОТ -"), (0.0, 0.0, 0.0, "⏸️ СТОП"),
            (self.LINEAR_SPEED, 0.0, 0.0, "⬆️ ВПЕРЕД"),      (0.0, 0.0, 0.0, "⏸️ СТОП"),
            (-self.LINEAR_SPEED, 0.0, 0.0, "⬇️ НАЗАД"),     (0.0, 0.0, 0.0, "⏸️ СТОП"),
            (0.0, self.LINEAR_SPEED, 0.0, "⬅️ ВЛЕВО"),     (0.0, 0.0, 0.0, "⏸️ СТОП"),
            (0.0, -self.LINEAR_SPEED, 0.0, "➡️ ВПРАВО"),    (0.0, 0.0, 0.0, "⏸️ СТОП")
        ]

        self.cur_x, self.cur_y, self.cur_th = 0.0, 0.0, 0.0
        self.last_odom_t = 0.0
        self.is_testing = False
        self.result_show_until = 0.0 # Время, до которого показываем результат
        self.test_timer = None
        self.phase_idx = 0
        self.phase_step = 0

    def odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.cur_th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.last_odom_t = time.time()

    def health_check_callback(self):
        now = time.time()
        
        # Если идет тест - молчим
        if self.is_testing: return
        
        # Если время показа результата еще не вышло - показываем его
        if now < self.result_show_until:
            return 

        # Живой мониторинг (работает всё остальное время)
        odom_ok = "✅" if (now - self.last_odom_t < 2.0) else "❌"
        self.status_pub.publish(String(data=f"ODOM: {odom_ok} | MECANUM READY"))

    def start_test_handler(self, _):
        if self.is_testing: return
        self.is_testing = True
        self.result_show_until = 0.0
        self.phase_idx = 0
        self.phase_step = 0
        self.s_x, self.s_y, self.s_th = self.cur_x, self.cur_y, self.cur_th
        self.test_timer = self.create_timer(0.1, self.execute_sequence)

    def execute_sequence(self):
        if self.phase_idx >= len(self.sequence):
            self.finish_test()
            return

        vx, vy, w, label = self.sequence[self.phase_idx]
        self.status_pub.publish(String(data=f"⚙️ ТЕСТ: {label}"))
        
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, w
        self.cmd_pub.publish(msg)

        self.phase_step += 1
        limit = int(self.STOP_TIME * 10) if (vx == 0 and vy == 0 and w == 0) else int(self.MOVE_TIME * 10)
        
        if self.phase_step >= limit:
            self.phase_step = 0
            self.phase_idx += 1

    def finish_test(self):
        self.cmd_pub.publish(Twist())
        self.test_timer.destroy()
        
        dx, dy = abs(self.cur_x - self.s_x), abs(self.cur_y - self.s_y)
        dth = math.degrees(abs(self.cur_th - self.s_th))
        
        res = f"✅ OK! Err: X:{dx:.2f} Y:{dy:.2f} Th:{dth:.1f}°" if (dx < 0.12 and dy < 0.12) else f"⚠️ DRIFT! X:{dx:.2f} Y:{dy:.2f}"
        
        self.status_pub.publish(String(data=res))
        # Устанавливаем таймер показа результата
        self.result_show_until = time.time() + self.RESULT_DISPLAY_TIME
        self.is_testing = False

    def hard_reset_handler(self, _):
        self.status_pub.publish(String(data="♻️ ПЕРЕЗАГРУЗКА..."))
        subprocess.Popen(['sudo', 'systemctl', 'restart', 'robot.service'])


def main(args=None):
    rclpy.init(args=args)
    node = RobotInspector()
    node.status_pub.publish(String(data="ИНСПЕКТОР ГОТОВ"))
    node.create_timer(15.0, node.health_check_callback)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
