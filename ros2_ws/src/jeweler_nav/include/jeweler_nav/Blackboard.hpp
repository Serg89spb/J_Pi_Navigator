// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include "jeweler_nav/Variables.hpp"
#include "jeweler_nav/GridMap.hpp"
#include "jeweler_nav/ObstacleManager.hpp"
#include "jeweler_nav/GlobalPlanner.hpp"

namespace jeweler_nav {

class Blackboard {
public:
    // Настройки (из нашего нового Variables.hpp)
    NavigatorVariables params;

    // Состояние цели и робота
    bool has_goal = false;
    bool can_move = false;
    bool exploration_mode = false;
    geometry_msgs::msg::Point current_pos;
    double current_yaw = 0.0;
    double current_angular_vel = 0.0;
    Goal target_pos;

    // Указатели на тяжелые объекты (обновляются в колбэках)
    std::shared_ptr<GridMap> grid_map;
    std::shared_ptr<ObstacleManager> obstacle_manager;
    std::shared_ptr<GlobalPlanner> global_planner;

    // Общий путь (защищен мьютексом, так как его читает контроллер и пишет планировщик)
    std::vector<PathPoint> current_path;
    std::recursive_mutex path_mutex;

    // Выходные данные для приводов
    geometry_msgs::msg::Twist cmd_vel;

    // Для фильтра ускорений
    geometry_msgs::msg::Twist last_cmd;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan;

    sensor_msgs::msg::LaserScan::SharedPtr last_camera_scan;

    geometry_msgs::msg::Point window_start_pos;
    double expected_dist = 0.0;
    std::chrono::steady_clock::time_point window_start_time;

    std::vector<Frontier> black_list_frontiers;
    Frontier current_exploration_frontier;
    bool is_stuck = false;

    bool obstacles_updated = false;    // Для отрисовки кубиков камеры/лидара
    bool exploration_updated = false;  // Для отрисовки сферы цели эксплорации

    Goal backup_target;
    bool is_recovery_active = false;          // Флаг режима спасения

    bool path_updated = false;
    bool is_need_replan = true;
    bool is_replan_succseed = false;
    std::chrono::steady_clock::time_point last_replan_time;
    std::chrono::steady_clock::time_point exploration_deadline;
    std::chrono::steady_clock::time_point last_yaw_time;

    std::vector<std::pair<double, double>> last_inflation_points;
    bool inflation_updated = false;

    Blackboard() = default;
};

using BlackboardPtr = std::shared_ptr<Blackboard>;

} // namespace jeweler_nav
