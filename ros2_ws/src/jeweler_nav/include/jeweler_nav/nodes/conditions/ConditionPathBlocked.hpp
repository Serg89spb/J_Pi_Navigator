#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/ObstacleManager.hpp"

namespace jeweler_nav {

class ConditionPathBlocked : public BTNode {
public:
    using BTNode::BTNode;
    
    NodeStatus tick() override {
        
        // 1. Критическая проверка данных и наличия маршрута
        std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
        
        if (!blackboard_->obstacle_manager || blackboard_->current_path.empty()) {
            BTLogger::logStatusChange(name_ + "Common failed", NodeStatus::FAILURE); 
            return NodeStatus::FAILURE;
        }

        // 2. Проверка быстрой динамической блокировки (лидар)
        if (blackboard_->obstacle_manager->isDynamicBlocked()) {
            blackboard_->is_need_replan = true;
            BTLogger::logStatusChange(name_ + " Lidar", NodeStatus::SUCCESS); 
            BTLogger::logMessage(name_, "Path Blocked");
            return NodeStatus::SUCCESS; // Препятствие подтверждено лидаром
        }

        // 3. Проверка статической блокировки по точкам траектории (сетка уверенности)
        // size_t points_to_check = std::min(blackboard_->current_path.size(), (size_t)15);
        // for (size_t i = 0; i < points_to_check; ++i) {
        //     const auto& pt = blackboard_->current_path[i];
        //     if (blackboard_->obstacle_manager->isOccupied(
        //         pt.x, pt.y, blackboard_->params.obstacles.inflation_radius)) 
        //     {
        //         BTLogger::logStatusChange(name_ + "Other", NodeStatus::SUCCESS); 
        //         return NodeStatus::SUCCESS; // Путь перекрыт на карте
        //     }
        // }
        

        //BTLogger::logMessage(name_, "Path Free");
        BTLogger::logStatusChange(name_ + "Path Free", NodeStatus::FAILURE); 
        return NodeStatus::FAILURE; // Путь свободен по обоим критериям

    }
};

} // namespace jeweler_nav
