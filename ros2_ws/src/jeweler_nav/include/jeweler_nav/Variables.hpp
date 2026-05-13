#pragma once

#include <stdint.h>
#include "geometry_msgs/msg/point.hpp"

namespace jeweler_nav {

struct PathPoint {
    double x;
    double y;
    bool allow_strafing = false;
};

struct Frontier {
    double x, y;
    int size;
};

enum class NodeStatus {
    SUCCESS, // Узел успешно выполнил задачу
    FAILURE, // Узел не смог выполнить задачу или условие ложно
    RUNNING  // Узел еще в процессе (нужен повторный тик)
};

enum class GoalType {
    ORIGINAL,
    RECOVERY
};

struct Goal {
    geometry_msgs::msg::Point goal;
    GoalType type = GoalType::ORIGINAL;

    // Метод для получения строкового представления типа
    const char* type_str() const {
        switch (type) {
            case GoalType::ORIGINAL: return "ORIGINAL";
            case GoalType::RECOVERY: return "RECOVERY";
            default: return "UNKNOWN";
        }
    }
};

// Параметры безопасности и детектирования препятствий
struct SafetyParams {
    float stop_dist = 0.40f;          // Дистанция экстренной остановки (м)
    float slow_down_dist = 0.70f;     // Дистанция начала замедления (м)
    int confirm_stop_threshold = 5;    // CONFIRM_STOP: тиков для подтверждения препятствия
    float scan_radius_normal = 0.80f; // SCAN_RADIUS: базовый радиус лидара
    float scan_radius_turn = 0.20f;   // Радиус лидара при активном повороте
};

// Параметры алгоритма восстановления (Independent Recovery)
struct RecoveryParams {
    float min_inflation = 0.05;      // Минимальная инфляция для выхода из тупика 0.05
    float recovery_step = 0.05f;      // Шаг поиска свободной точки 0.05
    float max_recovery_dist = 1.0f;  // Макс. радиус поиска выхода (nearest_safe_point_r) 0.5
    float strafe_speed = 0.15f;       // Скорость перемещения при восстановлении (только стрейф)
    int max_attempts = 5;             // Количество итераций поиска перед отменой цели
    float search_angle_step = 0.392f;
};

// Параметры планирования (Global Planner)
struct PlanningParams {
    double footprint_radius = 0.18;   // Базовый радиус робота для A*
    double replan_cooldown = 60.0;     // Пауза между перепланировками (сек) 2
    
    // Итеративный поиск цели (Multi-stage)
    float search_step_manual = 0.20f; // Шаг расширения поиска для ручной цели
    float search_step_explore = 0.30f;// Шаг расширения поиска для фронтиров
    float max_search_radius = 2.0f;   // Предел поиска валидной точки цели
};

// Параметры ObstacleManager (камера и лидар)
struct ObstacleParams {
    uint8_t confirm_threshold = 120;            // Порог уверенности для основного планировщика
    uint8_t inc_step = 15;                      // Прирост уверенности за кадр
    uint8_t dec_step = 60;                      // Агрессивный сброс (Negative Update)
    float min_cam_dist = 0.35f;                 // Игнорируем всё, что ближе (слепая зона)
    float max_cam_dist = 1.5f;                  // Дальность работы камеры
    float max_lidar_dist = 1.5f;                // Дальность работы лидара
    float inflation_radius = 0.1f;             // Дополнительное раздутие объектов камеры
    float corridor_width = 0.1f;               // Ширина защитного коридора (м)
    float path_corridor_width = 0.08f;          // Ширина защитного коридора на пути
    float min_lx_dist = 0.15f;                  // Минимальное расстояние по X
    float max_confidence_limit = 15.0f;         // Потолок счётчика уверенности
    int recovery_ticks_limit = 200;             // Таймаут сброса (10 секунд при 20Гц)
    size_t min_lidar_intruder_points = 10;      // Порог для лидара
    size_t min_cam_intruder_points = 1;         // Порог для камеры
    float laser_pt_wall_infl = 0.1f;            // Область проверки что точка лазера - в стене
    size_t checked_path_points = 30;            // На какой длине пути проверяем препятствия

    // Раздельные пороги подтверждения для лидара и камеры
    float confirm_replan_count_lidar = 5;
    float confirm_replan_count_cam = 2;

    // Радиусы раздутия динамических препятствий (в ячейках)
    uint8_t lidar_dynamic_inflation_cells = 0;
    uint8_t cam_dynamic_inflation_cells = 1;
};

// Параметры локального контроля (Mecanum / Smoothing)
struct ControlParams {
    double max_accel = 0.10;          // MAX_ACCEL: макс. изменение линейной скорости за 50мс
    double max_alpha = 0.30;          // MAX_ALPHA: макс. изменение угловой скорости
    float deadband_angle = 0.05f;     // Ошибка угла, при которой не вращаемся
    float goal_reach_dist = 0.15f;    // Дистанция до точки для SUCCESS
    float explore_goal_reach_dist = 0.2f;    // Дистанция до точки исследования для SUCCESS
    float off_track_limit = 0.40f;    // Дистанция до начала пути для Emergency Replan
    float efficiency_limit = 0.3f;    // Порог эффективности (efficiency < 0.15)
    float min_expected_dist = 0.1f;   // Мин. дистанция команд для начала проверки
    float stuck_window_seconds = 2.0f; // Окно проверки застревания в секундах //1.8
    float stuck_inject_dist = 0.20f;   // Дистанция инъекции препятствия перед роботом
};

// Параметры исследования (Exploration)
struct ExplorationParams {
    float frontier_min_dist = 0.4f;// Мин. дистанция до нового фронтира
    float frontier_max_dist = 25.0f;// Макс. дистанция до нового фронтира
    int failed_frontier_limit = 5;    // Кол-во неудач до занесения в Blacklist
    float blacklist_radius = 0.30f;   // Радиус "бана" фронтира
};

// Общая структура настроек
struct NavigatorVariables {
    SafetyParams safety;
    RecoveryParams recovery;
    PlanningParams planning;
    ObstacleParams obstacles;
    ControlParams control;
    ExplorationParams exploration;
};

} // namespace jeweler_nav
