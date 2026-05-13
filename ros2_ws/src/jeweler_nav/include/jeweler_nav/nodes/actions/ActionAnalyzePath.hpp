#pragma once
#include "jeweler_nav/BTNode.hpp"
#include <cmath>

namespace jeweler_nav {

struct PathSegment {
    size_t start_idx;
    size_t end_idx;
};

class ActionAnalyzePath : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
        if (blackboard_->current_path.size() < 2) {
            BTLogger::logStatusChange(name_ + "Too small path", NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS;
            }

        // 1. Сбрасываем старые флаги стрейфа
        for (auto& pt : blackboard_->current_path) pt.allow_strafing = false;

        // 2. Разбиваем путь на прямые отрезки по углу (порог 0.35 рад)
        size_t start = 0;
        std::vector<PathSegment> segments;

        for (size_t i = 1; i < blackboard_->current_path.size(); ++i) {
            double y_curr = std::atan2(blackboard_->current_path[i].y - blackboard_->current_path[i-1].y, 
                                       blackboard_->current_path[i].x - blackboard_->current_path[i-1].x);
            double y_start = std::atan2(blackboard_->current_path[start+1].y - blackboard_->current_path[start].y, 
                                        blackboard_->current_path[start+1].x - blackboard_->current_path[start].x);
            
            if (std::abs(std::atan2(std::sin(y_curr - y_start), std::cos(y_curr - y_start))) > 0.35 || i == blackboard_->current_path.size() - 1) {
                segments.push_back({start, i}); // Запись в структуру
                start = i;
            }
        }

        // 3. Классификация сегментов на стрейф (короткие перемычки < 0.6м)
        for (size_t i = 0; i < segments.size(); ++i) {
        auto& seg = segments[i];
        double len = std::sqrt(std::pow(blackboard_->current_path[seg.end_idx].x - blackboard_->current_path[seg.start_idx].x, 2) + 
                               std::pow(blackboard_->current_path[seg.end_idx].y - blackboard_->current_path[seg.start_idx].y, 2));

            if (len < 0.6 && i > 0 && i < segments.size() - 1) {
                // Проверка безопасности (инфляция 0.20м) по картам
                bool safe = true;
                for (size_t j = seg.start_idx; j <= seg.end_idx; ++j) {
                    if (!blackboard_->grid_map->isSafe(blackboard_->current_path[j].x, blackboard_->current_path[j].y, 
                        0.20f) ||
                        (blackboard_->obstacle_manager && blackboard_->obstacle_manager->isOccupied(blackboard_->current_path[j].x, blackboard_->current_path[j].y, 
                            0.20f))) {
                        safe = false; break;
                    }
                }
                if (safe) {
                    for (size_t j = seg.start_idx; j <= seg.end_idx; ++j) blackboard_->current_path[j].allow_strafing = true;
                }
            }
        }

        blackboard_->is_need_replan = false;
        blackboard_->path_updated = true;
        BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
        return NodeStatus::SUCCESS;
    }
};

} // namespace jeweler_nav
