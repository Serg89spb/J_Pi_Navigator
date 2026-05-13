#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import subprocess
import numpy as np
import math
import time
import os
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

        self.get_logger().info(f">>> Загрузка модели OpenVINO из: {model_path}")
        self.model = YOLO(model_path, task='detect')
        
        cmd = ['rpicam-vid', '-t', '0', '--framerate', '20', '--width', '320', '--height', '240', '--inline', '--codec', 'mjpeg', '--flush', '--nopreview', '-o', '-']
        self.cam_process = subprocess.Popen(
            cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.DEVNULL, 
            bufsize=0 # ВАЖНО: отключаем буферизацию Python
        )
        
        threading.Thread(target=self._capture_thread, daemon=True).start()
        threading.Thread(target=self._processing_thread, daemon=True).start()
        self.get_logger().info(">>> Система запущена. Frame_id: base_link")

    def get_robot_coords(self, x_px, y_px):
        """ Преобразование пикселя в (dist, angle) относительно base_link """
        gamma = math.atan2(y_px - 120.0, self.f_pixel)
        denom = math.tan(self.PITCH + gamma)
        
        # Если знаменатель около нуля или отрицательный - точка выше горизонта
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
        import os
        import fcntl
        # Делаем чтение из stdout неблокирующим
        fd = self.cam_process.stdout.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        raw_buffer = b''
        while self.running:
            try:
                # Читаем абсолютно всё, что есть в буфере ОС на данный момент
                chunk = self.cam_process.stdout.read() 
                if not chunk: continue
                raw_buffer += chunk
                
                # Если накопилось слишком много (больше 100кб), значит мы отстаем
                # Ищем ПОСЛЕДНИЙ кадр в этой куче
                a = raw_buffer.rfind(b'\xff\xd8')
                b = raw_buffer.rfind(b'\xff\xd9')
                
                if a != -1 and b != -1 and b > a:
                    jpg_data = raw_buffer[a:b+2]
                    # Выкидываем ВЕСЬ старый буфер, оставляя только свежак
                    raw_buffer = b'' 
                    
                    frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        self.current_frame = frame
            except (IOError, TypeError):
                time.sleep(0.01) # Ждем, пока в трубе появятся новые данные
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
                    # self.get_logger().info(f"Skipping giant box: {box_area/frame_area:.1%}")
                    continue
                
                # --- ГЕОМЕТРИЧЕСКИЙ ФИЛЬТР (Защита от фантомного пола) ---
                # Если нижний край рамки слишком низко (наползает на капот робота),
                # берем центр объекта для замера дистанции, чтобы не считать пол.
                if y2 > 185:
                    y_target = (y1 + y2) / 2.0
                else:
                    # Для дальних объектов берем точку чуть выше нижней границы (на 10% высоты)
                    y_target = y2 - (y2 - y1) * 0.1
                
                # Считаем дистанцию и угол центра по новой "безопасной" вертикали
                dist, angle = self.get_robot_coords((x1 + x2) / 2.0, y_target)
                
                # Игнорируем всё в мертвой зоне камеры (ближе 28см). Там физически не может быть горшка.
                if dist < 0.28: continue
                
                if dist < self.MAX_RANGE:
                    # ПРАВКА: Углы краев теперь тоже считаем по y_target, а не по y2+4.0!
                    # Это синхронизирует дистанцию и ширину объекта на карте.
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
            # Заполняем LaserScan
            a_start, a_end = track['angles']
            # В ROS углы растут справа (+) налево (-), 
            # но atan2 дает инверсию, поэтому берем min/max
            idx_s = int((min(a_start, a_end) - scan.angle_min) / scan.angle_increment)
            idx_e = int((max(a_start, a_end) - scan.angle_min) / scan.angle_increment)
            
            for idx in range(max(0, idx_s), min(num_readings, idx_e + 1)):
                if track['dist'] < scan.ranges[idx]:
                    scan.ranges[idx] = float(track['dist'])

            if debug_frame is not None:
                x1, y1, x2, y2 = track['box']
                color = (0, 255, 0) if track['active'] else (255, 140, 0)
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(debug_frame, f"{track['dist']:.2f}m {self.current_fps}fps", (x1, y1-10), 0, 0.5, color, 2)

            track['ttl'] -= 1
            track['active'] = False 
            if track['ttl'] > 0: still_alive.append(track)
        
        self.tracked_objects = still_alive
        self.publisher_.publish(scan)

        if debug_frame is not None:
            save_path = os.path.expanduser('~/debug_yolo.jpg')
            cv2.imwrite(save_path, debug_frame)
        
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.format = "jpeg"
            # Сжимаем картинку в JPEG (качество 70-80 обычно за глаза)
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
        node.cam_process.terminate()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
