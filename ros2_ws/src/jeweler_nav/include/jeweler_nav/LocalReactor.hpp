#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include "jeweler_nav/GridMap.hpp"
#include "jeweler_nav/Variables.hpp"

namespace jeweler_nav {

class LocalReactor {

public:
    LocalReactor() = default;
    // НОВАЯ ФУНКЦИЯ: Расчет скоростей для Меканум-платформы
    geometry_msgs::msg::Twist computeMecanumVelocity(
        const geometry_msgs::msg::Point& pos,   // Текущее положение робота
        double yaw,                             // Текущий угол робота
        const PathPoint& target,                // Ближайшая "морковка" (цель)
        double desired_yaw,                     // Угол, куда должна смотреть камера
        bool force_stop                         // Флаг принудительной остановки
    ) {
        geometry_msgs::msg::Twist cmd;
        if (force_stop) return cmd;

        // Вектор до цели в глобальных координатах
        double dx = target.x - pos.x;
        double dy = target.y - pos.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 0.1) return cmd; // Приехали к точке

        // Перевод вектора цели в ЛОКАЛЬНЫЕ координаты робота (X - вперед, Y - лево)
        double local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
        double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);

        if (target.allow_strafing) {
            // --- РЕЖИМ СТРЕЙФА (Боковое движение) ---
            // Двигаемся в сторону точки (и по X и по Y)
            cmd.linear.x = std::clamp(local_x, -0.15, 0.15);
            cmd.linear.y = std::clamp(local_y, -0.15, 0.15);

            // Удерживаем корпус робота на желаемом угле (desired_yaw)
            double angle_err = std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));
            cmd.angular.z = std::clamp(static_cast<float>(angle_err * 1.5f), -0.2f, 0.2f);
        } 
        else {
            // --- ОБЫЧНЫЙ РЕЖИМ (Танковый разворот) ---
            double target_yaw = std::atan2(dy, dx);
            double angle_err = std::atan2(std::sin(target_yaw - yaw), std::cos(target_yaw - yaw));
            
            // Если ошибка угла меньше 3 градусов — не дергаем носом (Deadband)
            if (std::abs(angle_err) < 0.05) {
                cmd.angular.z = 0.0;
            } else {
                cmd.angular.z = std::clamp(static_cast<float>(angle_err * 2.2f), -0.8f, 0.8f);
            }

            double drive_scalar = std::max(0.0, 1.0 - (std::abs(angle_err) / 0.8));
            cmd.linear.x = std::min(0.25, dist) * drive_scalar;
            cmd.linear.y = 0.0; 

            if (std::abs(angle_err) > 0.8) cmd.linear.x = 0.0; 
        }

        return cmd;
    }
};

}
