#!/usr/bin/env python3

# -*- coding: utf-8 -*-
# Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import socket
import numpy as np
import math
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class YoloLaserNode(Node):
    def __init__(self):
        super().__init__('yolo_laser_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Не ждем подтверждения доставки
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Храним только 1 последний кадр в очереди
        )

        self.publisher_ = self.create_publisher(LaserScan, '/camera_scan', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/camera/debug_image', qos_profile)
        self.bridge = CvBridge()
        
        # --- ФИЗИЧЕСКИЕ ПАРАМЕТРЫ ---
        self.H_LENS = 0.184           # Высота линзы (м)
        self.PITCH = math.radians(23.0) # Наклон камеры вниз
        self.OFFSET_X = 0.195         # Вынос камеры вперед от центра робота (м)
        self.VFOV = math.radians(41.0)
        self.HFOV = math.radians(66.0)
        self.MAX_RANGE = 3.0
        
        # Фокусное расстояние в пикселях
        self.f_pixel = 240 / (2 * math.tan(self.VFOV / 2))
        
        # Трекинг и FPS
        self.tracked_objects = []
        self.running = True
        self.current_frame = None
        self.last_debug_t = time.time()
        self.fps_start_time = time.time()
        self.frame_count = 0
        self.current_fps = 0

        # Получаем путь к папке, в которой лежит этот скрипт
        script_dir = os.path.dirname(os.path.realpath(__file__))
        # Формируем полный путь к папке с моделью
        model_path = os.path.join(script_dir, 'yolov8n_openvino_model')
        self.get_logger().info(">>> Загрузка модели OpenVINO...")
        self.model = YOLO(model_path, task='detect')
        
        # --- НАСТРОЙКИ UDP СОКЕТА ИЗ ПАРАМЕТРОВ ROS 2 ---
        self.declare_parameter('yolo_port', 12348)
        self.port = self.get_parameter('yolo_port').get_parameter_value().integer_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)
        
        # Системный буфер на 128Кб, чтобы JPEG не дропался
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)
        
        threading.Thread(target=self._capture_thread, daemon=True).start()
        threading.Thread(target=self._processing_thread, daemon=True).start()
        self.get_logger().info(f">>>> Система запущена (UDP порт {self.port}). Frame_id: base_link")

    def get_robot_coords(self, x_px, y_px):
        """ Преобразование пикселя в (dist, angle) относительно base_link """
        gamma = math.atan2(y_px - 120.0, self.f_pixel)
        denom = math.tan(self.PITCH + gamma)
        
        if denom <= 0.05: 
            return self.MAX_RANGE, 0.0
            
        dist_floor = self.H_LENS / denom
        
        # Координаты относительно КАМЕРЫ (X - вперед, Y - влево)
        x_cam = dist_floor
        y_cam = -(dist_floor * (x_px - 160.0)) / self.f_pixel
        
        # Координаты относительно ЦЕНТРА РОБОТА (base_link)
        x_base = x_cam + self.OFFSET_X
        y_base = y_cam # камера по центру Y
        
        # Перевод в полярную систему для LaserScan
        dist_final = math.sqrt(x_base**2 + y_base**2)
        angle_final = math.atan2(y_base, x_base)
        
        return dist_final, angle_final

    def _capture_thread(self):
        """ Захват JPEG кадров из UDP сокета """
        while self.running:
            try:
                data, addr = self.sock.recvfrom(65536)
                if not data: continue
                
                # Декодируем JPEG из байт
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    self.current_frame = frame
            except BlockingIOError:
                time.sleep(0.001)
                continue
            except Exception:
                continue

    def _processing_thread(self):
        while self.running:
            if self.current_frame is None:
                time.sleep(0.005)
                continue
            frame = self.current_frame
            self.current_frame = None 
            results = self.model.predict(frame, conf=0.25, verbose=False)
            self._process_results(results, frame)
            
            # FPS монитор
            self.frame_count += 1
            if time.time() - self.fps_start_time >= 1.0:
                self.current_fps = self.frame_count
                self.frame_count = 0
                self.fps_start_time = time.time()

    def _process_results(self, results, frame):
        num_readings = 60
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link' # Точки строятся от центра робота
        
        # Углы скана для base_link (шире, чем у камеры, т.к. камера впереди)
        scan.angle_min = -math.pi / 2 # -90 град
        scan.angle_max = math.pi / 2  # +90 град
        scan.angle_increment = math.pi / (num_readings - 1)
        scan.range_min, scan.range_max = 0.05, self.MAX_RANGE
        scan.ranges = [float('inf')] * num_readings

        debug_frame = frame.copy() if (time.time() - self.last_debug_t > 0.07) else None
        if debug_frame is not None:
            self.last_debug_t = time.time()
        
        new_detections = []
        for r in results:
            if r.boxes is None: continue
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()

            for i in range(len(boxes)):
                if confs[i] < 0.25: continue
                x1, y1, x2, y2 = boxes[i]

                 # --- ПРОВЕРКА ОБЩЕГО РАЗМЕРА ОБЪЕКТА (Area Filter) ---
                box_area = (x2 - x1) * (y2 - y1)
                frame_area = 320 * 240
            
                if box_area > (frame_area * 0.8):
                    continue
                
                # --- ГЕОМЕТРИЧЕСКИЙ ФИЛЬТР (Защита от фантомного пола) ---
                if y2 > 200: # Только если объект совсем под носом (у самого края кадра)
                    y_target = (y1 + y2) / 2.0
                else:
                    y_target = y2 # Для дальних берем нижнюю точку соприкосновени
                
                dist, angle = self.get_robot_coords((x1 + x2) / 2.0, y_target)
                
                if dist < 0.28: continue
                
                if dist < self.MAX_RANGE:
                    _, a_left = self.get_robot_coords(x1 + (x2-x1)*0.15, y_target)
                    _, a_right = self.get_robot_coords(x2 - (x2-x1)*0.15, y_target)

                    new_detections.append({
                        'dist': dist,
                        'angles': (a_left, a_right), 
                        'box': (int(x1), int(y1), int(x2), int(y2)),
                        'x_px_center': (x1 + x2) / 2.0,
                        'ttl': 12,
                        'active': True
                    })

        # Память и трекинг
        for det in new_detections:
            matched = False
            for track in self.tracked_objects:
                if abs(det['x_px_center'] - track['x_px_center']) < 70:
                    track.update(det)
                    matched = True
                    break
            if not matched: self.tracked_objects.append(det)

        still_alive = []
        for track in self.tracked_objects:
            a_start, a_end = track['angles']
            idx_s = int((min(a_start, a_end) - scan.angle_min) / scan.angle_increment)
            idx_e = int((max(a_start, a_end) - scan.angle_min) / scan.angle_increment)
            
            for idx in range(max(0, idx_s), min(num_readings, idx_e + 1)):
                if track['dist'] < scan.ranges[idx]:
                    scan.ranges[idx] = float(track['dist'])

            if debug_frame is not None:
                x1, y1, x2, y2 = track['box']
                color = (0, 255, 0) if track['active'] else (255, 140, 0)
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
                # Рисуем дистанцию прямо под рамкой объекта, чтобы видеть, чей это замер
                cv2.putText(debug_frame, f"{track['dist']:.2f}m", (x1, y2 + 15), 0, 0.4, color, 1)

            track['ttl'] -= 1
            track['active'] = False 
            if track['ttl'] > 0: still_alive.append(track)
        
        self.tracked_objects = still_alive
        self.publisher_.publish(scan)

        if debug_frame is not None:
            fps_text = f"FPS: {self.current_fps}"
            # Координаты 210, 230 — это примерно правый нижний угол для 320x240
            cv2.putText(debug_frame, fps_text, (220, 230), 0, 0.5, (255, 255, 255), 1)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.format = "jpeg"
            success, encoded_image = cv2.imencode('.jpg', debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if success:
                msg.data = encoded_image.tobytes()
                self.image_pub.publish(msg)

def main():
    rclpy.init()
    node = YoloLaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.running = False
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
