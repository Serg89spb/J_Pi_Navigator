// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "jeweler_nav/Variables.hpp"

namespace jeweler_nav {

class GridMap;
class ObstacleManager {
public:
    ObstacleManager(double res, int w, int h, double origin_x, double origin_y, const ObstacleParams& params);

    // Новые функции: обновление от лидара и камеры
    void updateLidarObstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                              double angular_z,
                              const geometry_msgs::msg::Point& current_pos,
                              double current_yaw,
                              std::shared_ptr<GridMap> grid_map,
                              const std::vector<PathPoint>& current_path);

    void updateCamObstacles(const sensor_msgs::msg::LaserScan::SharedPtr camera_scan,
                            double angular_z,
                            const geometry_msgs::msg::Point& current_pos,
                            double current_yaw,
                            std::shared_ptr<GridMap> grid_map,
                            const std::vector<PathPoint>& current_path);

    // Проверка для планировщика A*
    bool isOccupied(double wx, double wy, float inflation_rad = -1, int custom_threshold = -1) const;

    // Проверка: появилось ли новое препятствие на пути?
    bool checkPath(const std::vector<std::pair<double, double>>& path);

    void reset();

    // Для визуализации в Foxglove (возвращает мировые координаты подтвержденных точек)
    struct ObsPoint { double x, y; uint8_t conf; };
    std::vector<ObsPoint> getActiveObstacles() const;
    void injectObstacle(double wx, double wy, uint8_t confidence);
    bool isDynamicBlocked() const;               // Изменена: проверяет оба счётчика

    // Переименована: теперь общая для лидара и камеры
    bool isPointOnPath(double wx, double wy, const std::vector<PathPoint>& current_path, double threshold, size_t first_path_points_num) const;

    const std::vector<geometry_msgs::msg::Point>& getLidarIntruderPoints() const { return last_lidar_intruder_pts_; }
    const std::vector<geometry_msgs::msg::Point>& getCamIntruderPoints() const { return last_cam_intruder_pts_; }
    const std::vector<geometry_msgs::msg::Point>& getLidarOccupiedPoints() const { return last_lidar_occupied_pts_; }
    const std::vector<geometry_msgs::msg::Point>& getCamOccupiedPoints() const { return last_cam_occupied_pts_; }
    void resetConfidence() {
        dynamic_confidence_lidar_ = 0;
        dynamic_confidence_cam_ = 0;
    }

private:
    int worldToIndex(double wx, double wy) const;
    void indexToWorld(int idx, double& wx, double& wy) const;

    double res_;
    int w_, h_;
    double ox_, oy_;
    std::vector<uint8_t> grid_; // Сетка уверенности

    // Раздельные счётчики и списки для лидара и камеры
    float dynamic_confidence_lidar_ = 0.0f;
    int stagnation_timer_lidar_ = 0;
    std::vector<int> dynamic_indices_to_clear_lidar_;

    float dynamic_confidence_cam_ = 0.0f;
    int stagnation_timer_cam_ = 0;
    std::vector<int> dynamic_indices_to_clear_cam_;

    ObstacleParams params_;

    std::vector<geometry_msgs::msg::Point> last_lidar_intruder_pts_;
    std::vector<geometry_msgs::msg::Point> last_cam_intruder_pts_;
    std::vector<geometry_msgs::msg::Point> last_cam_occupied_pts_;
    std::vector<geometry_msgs::msg::Point> last_lidar_occupied_pts_;
};

} // namespace jeweler_nav