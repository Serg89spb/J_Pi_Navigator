# -*- coding: utf-8 -*-
# Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

import subprocess
import os
import time

def record_and_convert():
    # Настройки видео
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    temp_mjpeg = f"temp_{timestamp}.mjpeg"
    output_mp4 = f"robot_video_{timestamp}.mp4"
    
    # 1. Ждем команду на старт
    input(f"\n Нажми [ENTER], чтобы начать запись (сохраним в {output_mp4})...")
    
    # 2. Запуск записи (ставим большое время -t 0, чтобы писать пока не прервем)
    # --width 1280 --height 720 для нормального качества
    print(" >>> ЗАПИСЬ ПОШЛА! Для остановки нажми Ctrl+C...")
    
    cmd_record = [
        "rpicam-vid",
        "-t", "0", 
        "--codec", "mjpeg",
        "--width", "2560",
        "--height", "1440",
        "-o", temp_mjpeg,
        "--nopreview"
    ]
    
    try:
        # Запускаем и ждем ручного прерывания
        subprocess.run(cmd_record)
    except KeyboardInterrupt:
        print("\n >>> Запись остановлена пользователем.")

    # 3. Конвертация в MP4
    if os.path.exists(temp_mjpeg):
        print(" >>> Конвертирую в MP4 (без перекодировки)...")
        cmd_convert = [
            "ffmpeg", "-y", "-i", temp_mjpeg, 
            "-vcodec", "copy", output_mp4
        ]
        
        try:
            subprocess.run(cmd_convert, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            print(f" >>> ГОТОВО! Видео сохранено: {output_mp4}")
            
            # 4. Удаляем временный файл
            os.remove(temp_mjpeg)
            print(f" >>> Временный файл {temp_mjpeg} удален.")
        except Exception as e:
            print(f" Ошибка при конвертации: {e}")
    else:
        print(" Ошибка: Файл записи не найден.")

if __name__ == "__main__":
    try:
        while True:
            record_and_convert()
            ans = input("\n Записать еще раз? (y/n): ")
            if ans.lower() != 'y':
                break
    except KeyboardInterrupt:
        print("\nВыход из программы.")
