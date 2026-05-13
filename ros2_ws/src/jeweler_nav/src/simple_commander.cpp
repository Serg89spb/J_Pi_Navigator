// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <complex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Класс для расчета "отталкивания" от препятствий
class ObstacleManager {
public:
    struct Force { double x, y; };

Force calculateRepulsion(const sensor_msgs::msg::LaserScan::SharedPtr scan, double curr_yaw) {
    Force repulsion = {0.0, 0.0};
    if (!scan) return repulsion;

    double angle = scan->angle_min;
    for (float range : scan->ranges) {
        // Игнорируем шум и слишком далекие объекты
        if (range > 0.10 && range < 0.6) { 
            double global_angle = curr_yaw + angle;
            
            // Линейная сила: 0 на расстоянии 0.6м, максимум на 0.15м
            double force_mag = (0.6 - range) * 1.5; 

            // Направление ОТ препятствия (поэтому минус)
            repulsion.x -= std::cos(global_angle) * force_mag;
            repulsion.y -= std::sin(global_angle) * force_mag;
        }
        angle += scan->angle_increment;
    }
    return repulsion;
}
};

class JewelerCommander : public rclcpp::Node {
public:
    JewelerCommander() : Node("jeweler_commander") {
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, 
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_ = msg->pose.position; has_goal_ = true; });
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { 
                pos_ = msg->pose.pose.position;
                yaw_ = tf2::getYaw(msg->pose.pose.orientation);
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, 
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&JewelerCommander::control_loop, this));
    }

private:
void control_loop() {
    if (!has_goal_) return;

    double dx = target_.x - pos_.x;
    double dy = target_.y - pos_.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist < 0.15) { // Порог остановки 15см
        has_goal_ = false;
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        return;
    }

    // 1. Вектор цели (основной драйвер)
    double attr_x = dx / dist;
    double attr_y = dy / dist;

    // 2. Умное отталкивание (только то, что мешает ехать)
    double rep_x = 0.0;
    double rep_y = 0.0;
    if (last_scan_) {
        double angle = last_scan_->angle_min;
        for (float range : last_scan_->ranges) {
            // Реагируем только на то, что ближе 0.3м
            if (range > 0.10 && range < 0.3) {
                // Если точка лидара ПЕРЕД роботом (в секторе движения)
                double global_angle = yaw_ + angle;
                double force_mag = (0.3 - range) * 3.0; // Сделаем отталкивание резче
                rep_x -= std::cos(global_angle) * force_mag;
                rep_y -= std::sin(global_angle) * force_mag;
            }
            angle += last_scan_->angle_increment;
        }
    }

    // Ограничиваем "панику" от лидара, чтобы он не забивал цель полностью
    double rep_mag = std::sqrt(rep_x * rep_x + rep_y * rep_y);
    if (rep_mag > 1.5) {
        rep_x = (rep_x / rep_mag) * 1.5;
        rep_y = (rep_y / rep_mag) * 1.5;
    }

    // 3. Итоговый вектор
    double res_vx = attr_x + rep_x;
    double res_vy = attr_y + rep_y;

    // 4. ОГРАНИЧЕНИЕ СКОРОСТИ (30 см/с для бодрости)
    double max_v = 0.3; 
    double speed = std::sqrt(res_vx * res_vx + res_vy * res_vy);
    if (speed > 0.01) {
        res_vx = (res_vx / speed) * max_v;
        res_vy = (res_vy / speed) * max_v;
    }

    // 5. ПОВОРОТ: Считаем ошибку угла
    double target_yaw = std::atan2(dy, dx);
    double yaw_error = target_yaw - yaw_;
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    auto cmd = geometry_msgs::msg::Twist();
    
    // Сделаем поворот бодрее (коэф 1.5 и макс 1.0 рад/с)
    if (std::abs(yaw_error) > 0.1) {
        cmd.angular.z = std::clamp(yaw_error * 1.5, -1.0, 1.0);
    }

    // 6. ПЕРЕВОД В ЛОКАЛЬНЫЕ КООРДИНАТЫ (Матрица поворота)
    // Проверь: если едет боком, поменяй знаки или sin/cos
    cmd.linear.x = res_vx * std::cos(yaw_) + res_vy * std::sin(yaw_);
    cmd.linear.y = -res_vx * std::sin(yaw_) + res_vy * std::cos(yaw_);

    cmd_pub_->publish(cmd);
}

    // ROS интерфейсы и данные
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point target_, pos_;
    double yaw_ = 0;
    bool has_goal_ = false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    ObstacleManager obs_manager_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JewelerCommander>());
    rclcpp::shutdown();
    return 0;
}