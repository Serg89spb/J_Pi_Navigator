// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#include "jeweler_nav/ObstacleManager.hpp"
#include "jeweler_nav/BTLogger.hpp"
#include "jeweler_nav/GridMap.hpp"

using namespace jeweler_nav;

ObstacleManager::ObstacleManager(double res, int w, int h, double origin_x, double origin_y, const ObstacleParams& params)
    : res_(res), w_(w), h_(h), ox_(origin_x), oy_(origin_y), params_(params)
{
    grid_.assign(w_ * h_, 0);
}

int ObstacleManager::worldToIndex(double wx, double wy) const {
    int mx = (int)((wx - ox_) / res_);
    int my = (int)((wy - oy_) / res_);
    if (mx < 0 || mx >= w_ || my < 0 || my >= h_) return -1;
    return my * w_ + mx;
}

void ObstacleManager::indexToWorld(int idx, double& wx, double& wy) const {
    wx = ox_ + (idx % w_ + 0.5) * res_;
    wy = oy_ + (idx / w_ + 0.5) * res_;
}

void ObstacleManager::reset() {
    std::fill(grid_.begin(), grid_.end(), 0);
    dynamic_confidence_lidar_ = 0;
    stagnation_timer_lidar_ = 0;
    dynamic_indices_to_clear_lidar_.clear();
    dynamic_confidence_cam_ = 0;
    stagnation_timer_cam_ = 0;
    dynamic_indices_to_clear_cam_.clear();
}

std::vector<ObstacleManager::ObsPoint> ObstacleManager::getActiveObstacles() const {
    std::vector<ObsPoint> pts;
    for (int i = 0; i < (int)grid_.size(); ++i) {
        if (grid_[i] > 20) { // Показываем даже слабую уверенность для отладки
            double wx, wy;
            indexToWorld(i, wx, wy);
            pts.push_back({wx, wy, grid_[i]});
        }
    }
    return pts;
}

void ObstacleManager::injectObstacle(double wx, double wy, uint8_t confidence) {
    int idx = worldToIndex(wx, wy);
    if (idx >= 0 && idx < (int)grid_.size()) {
        // Используем 254 для фиолетового цвета в отладке. 
        // confidence используем просто чтобы не было warning unused parameter
        (void)confidence; 
        grid_[idx] = 254; 
        
        int neighbors[] = {idx+1, idx-1, idx+w_, idx-w_};
        for (int n_idx : neighbors) {
            if (n_idx >= 0 && n_idx < (int)grid_.size()) grid_[n_idx] = 254;
        }
    }
}

// Умная проверка с учетом раздутия (inflation)
bool ObstacleManager::isOccupied(double wx, double wy, float inflation_rad, int custom_threshold) const {
    int center_idx = worldToIndex(wx, wy);
    if (center_idx < 0) return false;

    // Выбираем порог: если передан custom_threshold, используем его, иначе стандартный
    int threshold = (custom_threshold == -1) ? params_.confirm_threshold : custom_threshold;

    // 1. Проверка самой ячейки
    if (grid_[center_idx] >= threshold) return true;

    // 2. Проверка в радиусе раздутия
    float current_infl = (inflation_rad == -1) ? params_.inflation_radius : inflation_rad;
    if (current_infl > 0.01f) {
        int cell_r = std::ceil(current_infl / res_);
        int mx = center_idx % w_;
        int my = center_idx / w_;

        for (int dx = -cell_r; dx <= cell_r; ++dx) {
            for (int dy = -cell_r; dy <= cell_r; ++dy) {
                int nx = mx + dx; int ny = my + dy;
                if (nx >= 0 && nx < w_ && ny >= 0 && ny < h_) {
                    if (grid_[ny * w_ + nx] >= threshold) return true;
                }
            }
        }
    }
    return false;
}

void ObstacleManager::updateLidarObstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                          double angular_z,
                                          const geometry_msgs::msg::Point& current_pos,
                                          double current_yaw,
                                          std::shared_ptr<GridMap> grid_map,
                                          const std::vector<PathPoint>& current_path)
{
    if (!scan) return;

    last_lidar_intruder_pts_.clear();
    last_lidar_occupied_pts_.clear();

    float current_radius = (std::abs(angular_z) > 0.5) ? 0.5f : params_.max_lidar_dist;

    double cos_y = std::cos(current_yaw);
    double sin_y = std::sin(current_yaw);
    size_t lidar_pts = 0;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float r = scan->ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < 0.15f || r > current_radius) continue;

        double angle = scan->angle_min + i * scan->angle_increment;
        double lx = 0.0772 - r * std::cos(angle);   // смещение лидара по X
        double ly = r * std::sin(angle);

        double wx = current_pos.x + (lx * cos_y - ly * sin_y);
        double wy = current_pos.y + (lx * sin_y + ly * cos_y);

        bool in_corridor = (lx > params_.min_lx_dist && std::abs(ly) < params_.corridor_width);
        bool on_path = isPointOnPath(wx, wy, current_path, params_.path_corridor_width, params_.checked_path_points);

        if (in_corridor || on_path) {
            geometry_msgs::msg::Point pt;
            pt.x = wx; pt.y = wy; pt.z = 0.1;

            // Фильтрация по карте (статичные стены)
            if (grid_map && !grid_map->isSafe(wx, wy, params_.laser_pt_wall_infl)) {
                last_lidar_occupied_pts_.push_back(pt);
                continue;
            }

            lidar_pts++;
            last_lidar_intruder_pts_.push_back(pt);
        }
    }

    bool lidar_detects = (lidar_pts >= params_.min_lidar_intruder_points);

    if (lidar_detects) {
        dynamic_confidence_lidar_ = std::min(params_.max_confidence_limit, dynamic_confidence_lidar_ + 1.0f);
    } else {
        dynamic_confidence_lidar_ = std::max(0.0f, dynamic_confidence_lidar_ - 1.0f);
    }

    if (dynamic_confidence_lidar_ >= params_.confirm_replan_count_lidar) {
        stagnation_timer_lidar_++;

        int infl_cells = params_.lidar_dynamic_inflation_cells;
        for (const auto& pt : last_lidar_intruder_pts_) {
            int center_idx = worldToIndex(pt.x, pt.y);
            if (center_idx < 0 || center_idx >= (int)grid_.size()) continue;

            int mx = center_idx % w_;
            int my = center_idx / w_;
            for (int dx = -infl_cells; dx <= infl_cells; ++dx) {
                for (int dy = -infl_cells; dy <= infl_cells; ++dy) {
                    int nx = mx + dx; int ny = my + dy;
                    if (nx >= 0 && nx < w_ && ny >= 0 && ny < h_) {
                        int idx = ny * w_ + nx;
                        if (grid_[idx] != 250 && grid_[idx] < params_.confirm_threshold) {
                            grid_[idx] = 250;
                            dynamic_indices_to_clear_lidar_.push_back(idx);
                        }
                    }
                }
            }
        }

        if (stagnation_timer_lidar_ > params_.recovery_ticks_limit) {
            for (int idx : dynamic_indices_to_clear_lidar_) {
                if (idx >= 0 && idx < (int)grid_.size()) grid_[idx] = 0;
            }
            dynamic_indices_to_clear_lidar_.clear();
            dynamic_confidence_lidar_ = 0;
            stagnation_timer_lidar_ = 0;
        }
    } else {
        if (!dynamic_indices_to_clear_lidar_.empty()) {
            for (int idx : dynamic_indices_to_clear_lidar_) {
                if (idx >= 0 && idx < (int)grid_.size()) grid_[idx] = 0;
            }
            dynamic_indices_to_clear_lidar_.clear();
        }
        stagnation_timer_lidar_ = 0;
    }
}

void ObstacleManager::updateCamObstacles(const sensor_msgs::msg::LaserScan::SharedPtr camera_scan,
                                         double angular_z,
                                         const geometry_msgs::msg::Point& current_pos,
                                         double current_yaw,
                                         std::shared_ptr<GridMap> grid_map,
                                         const std::vector<PathPoint>& current_path)
{
    if (!camera_scan) return;
    //BTLogger::logMessage("ObstacleManager", "updateCamObstacles"); 

    last_cam_intruder_pts_.clear();
    last_cam_occupied_pts_.clear();

    float current_radius = (std::abs(angular_z) > 0.5) ? 0.5f : params_.max_cam_dist;

    double cos_y = std::cos(current_yaw);
    double sin_y = std::sin(current_yaw);
    size_t cam_pts = 0;

    for (size_t i = 0; i < camera_scan->ranges.size(); ++i) {
        float r = camera_scan->ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < params_.min_cam_dist || r > current_radius) continue;

        //BTLogger::logMessage("ObstacleManager", "Dist %.2f", r); 

        double angle = camera_scan->angle_min + i * camera_scan->angle_increment;
        double lx = r * std::cos(angle);
        double ly = r * std::sin(angle);

        double wx = current_pos.x + (lx * cos_y - ly * sin_y);
        double wy = current_pos.y + (lx * sin_y + ly * cos_y);

        //bool in_corridor = (lx > params_.min_lx_dist && std::abs(ly) < params_.corridor_width);
        bool on_path = isPointOnPath(wx, wy, current_path, params_.path_corridor_width, params_.checked_path_points);

        if (/*in_corridor ||*/ on_path) {
            geometry_msgs::msg::Point pt;
            pt.x = wx; pt.y = wy; pt.z = 0.1;

            if (grid_map && !grid_map->isSafe(wx, wy, params_.laser_pt_wall_infl)) {
                last_cam_occupied_pts_.push_back(pt);
                continue;
            }

            cam_pts++;
            last_cam_intruder_pts_.push_back(pt);
        }
    }

    bool cam_detects = (cam_pts >= params_.min_cam_intruder_points);

    if (cam_detects) {
        dynamic_confidence_cam_ = std::min(params_.max_confidence_limit, dynamic_confidence_cam_ + 2.0f);
    } else {
        dynamic_confidence_cam_ = std::max(0.0f, dynamic_confidence_cam_ - 0.1f);
    }

    // if (cam_pts > 0 || dynamic_confidence_cam_ > 0) {
    // BTLogger::logMessage("ObstacleManager", "Cam Dynamic: [DynPoints: %zu, Conf: %.2f/%.2f]", 
    //    cam_pts, 
    //    dynamic_confidence_cam_, 
    //    params_.confirm_replan_count_cam);
    // }

    if (dynamic_confidence_cam_ >= params_.confirm_replan_count_cam) {
        stagnation_timer_cam_++;

        int infl_cells = params_.cam_dynamic_inflation_cells;
        for (const auto& pt : last_cam_intruder_pts_) {
            int center_idx = worldToIndex(pt.x, pt.y);
            if (center_idx < 0 || center_idx >= (int)grid_.size()) continue;

            int mx = center_idx % w_;
            int my = center_idx / w_;
            for (int dx = -infl_cells; dx <= infl_cells; ++dx) {
                for (int dy = -infl_cells; dy <= infl_cells; ++dy) {
                    int nx = mx + dx; int ny = my + dy;
                    if (nx >= 0 && nx < w_ && ny >= 0 && ny < h_) {
                        int idx = ny * w_ + nx;
                        if (grid_[idx] != 250 && grid_[idx] < params_.confirm_threshold) {
                            grid_[idx] = 250;
                            dynamic_indices_to_clear_cam_.push_back(idx);
                        }
                    }
                }
            }
        }

        if (stagnation_timer_cam_ > params_.recovery_ticks_limit) {
            for (int idx : dynamic_indices_to_clear_cam_) {
                if (idx >= 0 && idx < (int)grid_.size()) grid_[idx] = 0;
            }
            dynamic_indices_to_clear_cam_.clear();
            dynamic_confidence_cam_ = 0;
            stagnation_timer_cam_ = 0;
        }
    } else {
        if (!dynamic_indices_to_clear_cam_.empty()) {
            for (int idx : dynamic_indices_to_clear_cam_) {
                if (idx >= 0 && idx < (int)grid_.size()) grid_[idx] = 0;
            }
            dynamic_indices_to_clear_cam_.clear();
        }
        stagnation_timer_cam_ = 0;
    }
}

bool ObstacleManager::isDynamicBlocked() const {
    return (dynamic_confidence_lidar_ >= params_.confirm_replan_count_lidar) ||
           (dynamic_confidence_cam_ >= params_.confirm_replan_count_cam);
}

bool ObstacleManager::isPointOnPath(double wx, double wy, const std::vector<PathPoint>& current_path,
                                    double threshold, size_t first_path_points_num) const {
    if (current_path.empty()) return false;

    size_t look_ahead = std::min(current_path.size(), first_path_points_num);
    for (size_t i = 0; i < look_ahead; ++i) {
        double d = std::sqrt(std::pow(wx - current_path[i].x, 2) +
                             std::pow(wy - current_path[i].y, 2));
        if (d < threshold) return true;
    }
    return false;
}
