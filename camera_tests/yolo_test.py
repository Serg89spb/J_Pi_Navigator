#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import subprocess
import numpy as np
import os
import time

def main():
    # 1. Загрузка модели
    script_dir = os.path.dirname(os.path.realpath(__file__))
    model_path = os.path.abspath(os.path.join(script_dir, '..', 'yolov8n_openvino_model'))
    print(f"Загрузка модели из: {model_path}")
    
    # Явно указываем CPU и 1 поток для чистоты эксперимента
    os.environ["OPENVINO_NUM_THREADS"] = "1"
    model = YOLO(model_path, task='detect')

    # 2. Запуск камеры (те же параметры, что в основном коде)
    cmd = [
        'rpicam-vid', '-t', '0', '--framerate', '20', 
        '--width', '320', '--height', '240', 
        '--inline', '--codec', 'mjpeg', '--flush', '--nopreview', '-o', '-'
    ]
    
    cam_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    save_path = os.path.expanduser('~/test_yolo_check.jpg')
    
    print(f"Тест запущен. Результат пишется в {save_path}")
    print("Нажмите Ctrl+C для выхода.")

    raw_buffer = b''
    
    try:
        while True:
            # Читаем данные из трубы камеры
            chunk = cam_process.stdout.read(4096)
            if not chunk:
                continue
            raw_buffer += chunk

            # Ищем границы кадра JPEG
            a = raw_buffer.rfind(b'\xff\xd8')
            b = raw_buffer.rfind(b'\xff\xd9')

            if a != -1 and b != -1 and b > a:
                jpg_data = raw_buffer[a:b+2]
                raw_buffer = b'' # Очищаем буфер, чтобы не копить задержку

                # Декодируем кадр
                frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # 3. Инференс (только рамки)
                results = model.predict(frame, imgsz=320, conf=0.25, device='cpu', verbose=False)

                # 4. Рисуем только рамки и уверенность
                for r in results:
                    if r.boxes is not None:
                        boxes = r.boxes.xyxy.cpu().numpy()
                        confs = r.boxes.conf.cpu().numpy()

                        for i in range(len(boxes)):
                            x1, y1, x2, y2 = boxes[i].astype(int)
                            conf = confs[i]

                            # Зеленая рамка
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            
                            # Текст: уверенность справа внизу
                            label = f"{conf:.2f}"
                            # Вычисляем позицию, чтобы текст был внутри рамки
                            cv2.putText(frame, label, (x2 - 40, y2 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # 5. Запись в файл
                cv2.imwrite(save_path, frame)
                
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        cam_process.terminate()

if __name__ == '__main__':
    main()
