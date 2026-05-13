// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <memory>
#include <unordered_map>

namespace jeweler_nav {

class ObstacleManager;
class GridMap;

// Структура узла для алгоритма A*
struct Node {
    int x, y;          // Координаты в клетках
    float g_cost;      // Стоимость от старта до этой клетки
    float h_cost;      // Эвристика (расстояние до цели)
    Node* parent;      // Откуда пришли (для восстановления пути)

    float f_cost() const { return g_cost + h_cost; }

    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
};

class GlobalPlanner {
public:
    GlobalPlanner() = default;

    // Основная функция поиска пути
    std::vector<std::pair<double, double>> makePlan(
        std::shared_ptr<GridMap> map, 
        double start_x, double start_y, 
        double goal_x, double goal_y, 
        double footprint_radius, double nearest_safe_point_r,
        float goal_reach_dist,
        bool include_unknown,
        ObstacleManager* obs_mgr = nullptr
    );

    bool findNearestSafePoint(std::shared_ptr<GridMap> map, double& wx, double& wy, double footprint_radius, double nearest_safe_point_r, ObstacleManager* obs_mgr);
    const std::vector<std::pair<double, double>>& getLastInflationPoints() const {return last_inflation_points_;}
    std::vector<std::pair<double, double>> solveAStar(
        std::shared_ptr<GridMap> map, 
        double sx, double sy, double gx, double gy, 
        double footprint_radius, 
        double nearest_safe_point_r, 
        float goal_reach_dist,
        bool include_unknown,
        ObstacleManager* obs_mgr = nullptr);

private:
    std::vector<std::pair<double, double>> reconstructPath(
        std::shared_ptr<GridMap> map, 
        std::unordered_map<int, int>& came_from, 
        int current_idx
    );
    std::vector<std::pair<double, double>> last_inflation_points_;

};

} // namespace jeweler_nav
