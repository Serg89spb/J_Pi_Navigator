#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/BTLogger.hpp"
#include <cmath>

namespace jeweler_nav {

class ActionRecovery : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        if (!blackboard_->grid_map) return NodeStatus::FAILURE;

        const auto& p = blackboard_->params.recovery;
        const auto& robot_r = blackboard_->params.planning.footprint_radius;

        if(blackboard_->target_pos.type == GoalType::ORIGINAL){
            blackboard_->backup_target = blackboard_->target_pos;
        }
        if(blackboard_->target_pos.type == GoalType::RECOVERY){
            BTLogger::logMessage(name_, " Recovery goal already exist SUCCESS");
            BTLogger::logStatusChange(name_ + " Recovery goal already exist", NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS; 
        }

        double tx = blackboard_->current_pos.x;
        double ty = blackboard_->current_pos.y;
        double yaw = blackboard_->current_yaw;
        
        // Суммарный радиус проверки безопасности
        float safe_check_r = robot_r + p.min_inflation; 

        bool found = false;
        double best_x = 0, best_y = 0;

        auto reach_dist = blackboard_->exploration_mode ? 
        blackboard_->params.control.explore_goal_reach_dist : 
        blackboard_->params.control.goal_reach_dist;
        reach_dist += 0.1f;

        // Поиск по спирали с фильтром: только задняя полусфера (углы от 90 до 270 градусов)
        for (float r = reach_dist; r <= p.max_recovery_dist; r += p.recovery_step) {
            for (double angle = 0; angle < 2.0 * M_PI; angle += p.search_angle_step) {
                // Глобальный угол точки (относительно мировых осей)
                double global_angle = angle;
                // Локальный угол относительно направления робота (0 – вперёд)
                double local_angle = global_angle - yaw;
                // Нормализация в [-pi, pi]
                local_angle = atan2(sin(local_angle), cos(local_angle));
                
                // Пропускаем переднюю полусферу (углы от -90° до +90°)
                if (std::abs(local_angle) < M_PI/2) {
                    continue;
                }
                
                double check_x = tx + r * std::cos(global_angle);
                double check_y = ty + r * std::sin(global_angle);
                
                if (blackboard_->grid_map->isSafe(check_x, check_y, safe_check_r)) {
                    if (!blackboard_->obstacle_manager || 
                        !blackboard_->obstacle_manager->isOccupied(check_x, check_y, safe_check_r)) {
                        best_x = check_x;
                        best_y = check_y;
                        found = true;
                        break;
                    }
                }
            }
            if (found) break;
        }

        if (found) {
            blackboard_->target_pos.goal.x = best_x;
            blackboard_->target_pos.goal.y = best_y;
            blackboard_->target_pos.type = GoalType::RECOVERY;
            blackboard_->has_goal = true;
            
            // Гарантия стрейфа: записываем точку в путь с флагом allow_strafing
            {
                std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
                blackboard_->current_path.clear();
                
                PathPoint recovery_pt;
                recovery_pt.x = best_x;
                recovery_pt.y = best_y;
                recovery_pt.allow_strafing = true; // КРИТИЧНО: принудительный стрейф
                
                blackboard_->current_path.push_back(recovery_pt);
            }

            blackboard_->path_updated = true;
            BTLogger::logMessage(name_, "SUCCESS");
            BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS; 
        }

        BTLogger::logMessage(name_, "FAILURE");
        BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
        return NodeStatus::FAILURE; 
    }
};

} // namespace jeweler_nav
