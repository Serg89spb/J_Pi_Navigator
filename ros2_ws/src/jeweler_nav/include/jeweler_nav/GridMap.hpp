// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#pragma once

#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace jeweler_nav {
class GridMap {
public:
    GridMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        update(msg);
    }

    struct Frontier {
    double x, y;
    int size; // Количество клеток в кластере
    };

    void update(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        data_ = msg->data;
        info_ = msg->info;
    }

    // Перевод из мировых координат (метры) в индекс массива
    int worldToIndex(double wx, double wy) const {
        if (wx < info_.origin.position.x || wy < info_.origin.position.y) return -1;

        unsigned int mx = (unsigned int)((wx - info_.origin.position.x) / info_.resolution);
        unsigned int my = (unsigned int)((wy - info_.origin.position.y) / info_.resolution);

        if (mx >= info_.width || my >= info_.height) return -1;
        int idx = my * info_.width + mx;
        if (idx < 0 || idx >= (int)data_.size()) return -1; // Доп. проверка
        return idx;
    }

    // Проверка: свободна ли клетка (0 - свободно, 100 - стена, -1 - неизвестно)
    bool isFree(double wx, double wy) const {
        int index = worldToIndex(wx, wy);
        if (index < 0) return false;
        
        // В ROS принято: 0-50 — условно свободно, >50 — препятствие
        return data_[index] >= 0 && data_[index] < 50;
    }

    bool isSafe(double wx, double wy, float footprint_radius, bool include_unknown = true) const {
        // 1. Сначала базовая проверка: не в стене ли сам центр?
        int center_idx = worldToIndex(wx, wy);
        if (center_idx < 0 || data_[center_idx] >= 50 || data_[center_idx] == -1) return false;

        // 2. Считаем, сколько это клеток в радиусе (например, 25см / 5см = 5 клеток)
        int cell_radius = std::ceil(footprint_radius / info_.resolution);

        int mx = center_idx % info_.width;
        int my = center_idx / info_.width;

        // 3. Проверяем окрестность (Bounding Box) вокруг точки
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
                int nx = mx + dx;
                int ny = my + dy;

                // Проверка границ карты
                if (nx >= 0 && nx < (int)info_.width && ny >= 0 && ny < (int)info_.height) {
                    int check_idx = ny * info_.width + nx;
                    // Если хоть одна клетка в радиусе занята (>=50) или неизвестна (-1)
                    bool b = include_unknown ? 
                    (data_[check_idx] >= 50 || data_[check_idx] == -1) : 
                    data_[check_idx] >= 50;
                    if (b) {
                        return false; 
                    }
                }
            }
        }
        return true; // В радиусе чисто, можно ехать
    }

    bool isSafeFrontier(double wx, double wy, float footprint_radius) const {
        // 1. Сначала базовая проверка: не в стене ли сам центр?
        int center_idx = worldToIndex(wx, wy);
        if (data_[center_idx] >= 50) return false;

        // 2. Считаем, сколько это клеток в радиусе (например, 25см / 5см = 5 клеток)
        int cell_radius = std::ceil(footprint_radius / info_.resolution);

        int mx = center_idx % info_.width;
        int my = center_idx / info_.width;

        // 3. Проверяем окрестность (Bounding Box) вокруг точки
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
                int nx = mx + dx;
                int ny = my + dy;

                // Проверка границ карты
                if (nx >= 0 && nx < (int)info_.width && ny >= 0 && ny < (int)info_.height) {
                    int check_idx = ny * info_.width + nx;
                    // Если хоть одна клетка в радиусе занята (>=50) или неизвестна (-1)
                    if (data_[check_idx] >= 50) {
                        return false; 
                    }
                }
            }
        }
        return true; // В радиусе чисто, можно ехать
    }

    void indexToWorld(int index, double& wx, double& wy) const {
    unsigned int mx = index % info_.width;
    unsigned int my = index / info_.width;
    wx = info_.origin.position.x + (mx + 0.5) * info_.resolution;
    wy = info_.origin.position.y + (my + 0.5) * info_.resolution;
    }

    void setCell(double wx, double wy, int8_t value) {
    int index = worldToIndex(wx, wy);
    if (index >= 0 && index < (int)data_.size()) {
        data_[index] = value;
    }
    }

    std::vector<Frontier> findFrontiers(float min_dist_from_robot, float max_dist_from_robot, double robot_x, double robot_y) const {
        std::vector<Frontier> frontiers;
        std::vector<bool> visited(data_.size(), false);

        for (int i = 0; i < (int)data_.size(); ++i) {
            // Ищем свободную клетку, которая еще не посещена и граничит с неизвестностью
            if (data_[i] == 0 && !visited[i] && isFrontierCell(i)) {
                // Запускаем BFS/FloodFill, чтобы найти всё облако соседних точек фронтира
                Frontier f = clusterFrontier(i, visited, robot_x, robot_y);
                
                // Отсеиваем слишком маленькие (шум) и слишком близкие к роботу
                double dist = std::sqrt(std::pow(f.x - robot_x, 2) + std::pow(f.y - robot_y, 2));
                if (f.size > min_frontier_size_ && dist > min_dist_from_robot && dist < max_dist_from_robot) {
                    frontiers.push_back(f);
                }
            }
        }
        return frontiers;
    }

    // Вспомогательные геттеры
    uint32_t getWidth() const { return info_.width; }
    uint32_t getHeight() const { return info_.height; }
    float getResolution() const { return info_.resolution; }
    double getOriginX() const { return info_.origin.position.x; }
    double getOriginY() const { return info_.origin.position.y; }

private:
    std::vector<int8_t> data_;
    nav_msgs::msg::MapMetaData info_;
    int min_frontier_size_ = 16;     // Минимальное кол-во точек в кластере

    bool isFrontierCell(int index) const {
    int mx = index % info_.width;
    int my = index / info_.width;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int nx = mx + dx;
            int ny = my + dy;
            if (nx >= 0 && nx < (int)info_.width && ny >= 0 && ny < (int)info_.height) {
                if (data_[ny * info_.width + nx] == -1) return true;
            }
        }
    }
    return false;
}

Frontier clusterFrontier(int start_idx, std::vector<bool>& visited, double robot_x, double robot_y) const {
    std::queue<int> q;
    q.push(start_idx);
    visited[start_idx] = true;

    double best_wx, best_wy;
    indexToWorld(start_idx, best_wx, best_wy);
    double min_dist_to_robot = std::sqrt(std::pow(best_wx - robot_x, 2) + std::pow(best_wy - robot_y, 2));

    int count = 0;
    while (!q.empty()) {
        int curr = q.front(); q.pop();
        count++;

        double wx, wy;
        indexToWorld(curr, wx, wy);
        double d = std::sqrt(std::pow(wx - robot_x, 2) + std::pow(wy - robot_y, 2));

        // Ищем точку, которая БЛИЖЕ ВСЕГО к роботу
        if (d < min_dist_to_robot) {
            min_dist_to_robot = d;
            best_wx = wx; best_wy = wy;
        }

        int mx = curr % info_.width;
        int my = curr / info_.width;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                int nx = mx + dx; int ny = my + dy;
                int n_idx = ny * info_.width + nx;
                if (nx >= 0 && nx < (int)info_.width && ny >= 0 && ny < (int)info_.height) {
                    if (!visited[n_idx] && data_[n_idx] == 0 && isFrontierCell(n_idx)) {
                        visited[n_idx] = true;
                        q.push(n_idx);
                    }
                }
            }
        }
    }
    return {best_wx, best_wy, count};
}
};

}
