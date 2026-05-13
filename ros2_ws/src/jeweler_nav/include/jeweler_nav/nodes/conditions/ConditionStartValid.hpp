#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/BTLogger.hpp"

namespace jeweler_nav {

class ConditionStartValid : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        if (!blackboard_->grid_map) return NodeStatus::FAILURE;

        // Проверяем, находится ли текущая позиция в безопасной зоне
        bool safe = blackboard_->grid_map->isSafe(
            blackboard_->current_pos.x, 
            blackboard_->current_pos.y, 
            blackboard_->params.planning.footprint_radius
        );

        // Дополнительная проверка через камеру, если есть ObstacleManager
        if (safe && blackboard_->obstacle_manager) {
            if (blackboard_->obstacle_manager->isOccupied(
                blackboard_->current_pos.x, 
                blackboard_->current_pos.y)) 
            {
                safe = false;
            }
        }

        NodeStatus status = safe ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        
        // Логируем только если робот "застрял" или "освободился"
        BTLogger::logStatusChange(name_, status);
        
        return status;
    }
};

} // namespace jeweler_nav
