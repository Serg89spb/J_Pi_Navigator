#include "jeweler_nav/GlobalPlanner.hpp"
#include "jeweler_nav/ObstacleManager.hpp"
#include "jeweler_nav/BTLogger.hpp"
#include "jeweler_nav/GridMap.hpp"
#include <map>

using namespace jeweler_nav;

std::vector<std::pair<double, double>> GlobalPlanner::makePlan(
    std::shared_ptr<GridMap> map, 
    double start_x, double start_y, 
    double goal_x, double goal_y, 
    double footprint_radius, double nearest_safe_point_r, 
    float goal_reach_dist,
    bool include_unknown,
    ObstacleManager* obs_mgr)
{
    // 1. Переводим мировые координаты в координаты сетки
    int start_idx = map->worldToIndex(start_x, start_y);
    int goal_idx = map->worldToIndex(goal_x, goal_y);

    if (start_idx < 0 || goal_idx < 0 || !map->isFree(goal_x, goal_y)) {
        return {}; // Путь невозможен
    }

    // Здесь будет логика A* (напишем ниже)
    return solveAStar(map, start_x, start_y, goal_x, goal_y, footprint_radius, nearest_safe_point_r, goal_reach_dist, include_unknown, obs_mgr);
}

std::vector<std::pair<double, double>> GlobalPlanner::solveAStar(
    std::shared_ptr<GridMap> map, double sx, double sy, double gx, double gy, double footprint_radius, double nearest_safe_point_r, float goal_reach_dist, bool include_unknown, ObstacleManager* obs_mgr) 
{
    last_inflation_points_.clear();
    // 1. Корректируем СТАРТ и ЦЕЛЬ
    double valid_sx = sx; double valid_sy = sy;
    findNearestSafePoint(map, valid_sx, valid_sy, footprint_radius, nearest_safe_point_r, obs_mgr); 

    double valid_gx = gx; double valid_gy = gy;
    findNearestSafePoint(map, valid_gx, valid_gy, footprint_radius, nearest_safe_point_r, obs_mgr);

    // ПРОВЕРКА: Не притянули ли мы цель ближе допуска прибытия?
    double dist_to_goal = std::sqrt(std::pow(valid_gx - sx, 2) + std::pow(valid_gy - sy, 2));
    if (dist_to_goal < goal_reach_dist) {
        // Цель слишком близко после коррекции — возвращаем пустой путь, 
        return {}; 
    }

    // 2. Считаем индексы ОДИН РАЗ (используем только valid переменные)
    int start_idx = map->worldToIndex(valid_sx, valid_sy);
    int goal_idx = map->worldToIndex(valid_gx, valid_gy);
    
    if (start_idx < 0 || goal_idx < 0) return {};

    // Очередь с приоритетом и карты стоимости (дальше твой код без изменений)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    
    // Карта посещенных узлов: индекс -> стоимость g_cost
    std::unordered_map<int, float> g_costs;
    // Карта предков для восстановления пути: индекс_ребенка -> индекс_родителя
    std::unordered_map<int, int> came_from;

    // Добавляем стартовую точку
    open_set.push({(int)((valid_sx - map->getOriginX())/map->getResolution()), 
                (int)((valid_sy - map->getOriginY())/map->getResolution()), 
                0.0f, 0.0f, nullptr});
    g_costs[start_idx] = 0.0f;

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        int current_idx = current.y * map->getWidth() + current.x;

        // Если дошли до цели
        if (current_idx == goal_idx) {
            return reconstructPath(map, came_from, current_idx);
        }

        // Проверяем 8 соседей
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;
                int next_idx = ny * map->getWidth() + nx;

                // Проверка границ и препятствий через наш GridMap
                double wx, wy;
                // Нам нужны вспомогательные методы в GridMap для обратного перевода
                map->indexToWorld(next_idx, wx, wy);
                
                bool map_safe = map->isSafe(wx, wy, footprint_radius, include_unknown);
                bool camera_clear = true;
                
                // Передаем указатель или ссылку на менеджер в solveAStar, либо вызываем:
                if (obs_mgr && obs_mgr->isOccupied(wx, wy)) {
                    camera_clear = false;
                    //BTLogger::logMessage("Global Planner", "X:%.5f, Y:%.5f Is occupied", wx, wy);
                }

                if (!map_safe || !camera_clear) {
                    last_inflation_points_.emplace_back(wx, wy);
                    continue; 
                }

                // Стоимость перехода (диагональ чуть дороже: 1.41 против 1.0)
                float move_cost = (dx != 0 && dy != 0) ? 1.414f : 1.0f;
                float new_g_cost = g_costs[current_idx] + move_cost;

                if (g_costs.find(next_idx) == g_costs.end() || new_g_cost < g_costs[next_idx]) {
                    g_costs[next_idx] = new_g_cost;
                    float h = std::sqrt(std::pow(gx - wx, 2) + std::pow(gy - wy, 2));
                    
                    open_set.push({nx, ny, new_g_cost, h, nullptr});
                    came_from[next_idx] = current_idx;
                }
            }
        }
    }

    return {}; // Путь не найден
}

std::vector<std::pair<double, double>> GlobalPlanner::reconstructPath(
    std::shared_ptr<GridMap> map, std::unordered_map<int, int>& came_from, int current_idx) 
{
    std::vector<std::pair<double, double>> path;
    while (came_from.find(current_idx) != came_from.end()) {
        double wx, wy;
        map->indexToWorld(current_idx, wx, wy);
        path.push_back({wx, wy});
        current_idx = came_from[current_idx];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool GlobalPlanner::findNearestSafePoint(std::shared_ptr<GridMap> map, double& wx, double& wy, double footprint_radius, double nearest_safe_point_r, ObstacleManager* obs_mgr) {
    // Используем footprint_radius вместо radius, чтобы проверка была честной
    bool map_safe = map->isSafe(wx, wy, footprint_radius);
    bool obs_safe = (obs_mgr) ? !obs_mgr->isOccupied(wx, wy, 0.05f) : true;
    if (map_safe && obs_safe) return true;

    double step = map->getResolution();
    for (double r = step; r <= nearest_safe_point_r; r += step) { 
        for (double angle = 0; angle < 2.0 * M_PI; angle += M_PI / 8.0) {
            double tx = wx + r * std::cos(angle);
            double ty = wy + r * std::sin(angle);
            
            if (map->isSafe(tx, ty, footprint_radius) && 
               ((obs_mgr) ? !obs_mgr->isOccupied(tx, ty, 0.05f) : true)) {
                wx = tx; wy = ty;
                return true;
            }
        }
    }
    return false;
}
