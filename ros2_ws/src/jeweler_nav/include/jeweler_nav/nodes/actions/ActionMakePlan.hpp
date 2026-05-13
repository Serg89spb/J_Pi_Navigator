#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/BTLogger.hpp"
#include "jeweler_nav/GlobalPlanner.hpp"

namespace jeweler_nav {

class ActionMakePlan : public BTNode {

public:
    ActionMakePlan(const std::string& name, BlackboardPtr blackboard) 
        : BTNode(name, blackboard) {}

    NodeStatus tick() override {
        if (!blackboard_->grid_map) return NodeStatus::FAILURE;

        // 1. Проверяем, свободна ли сама цель. Если нет — ищем ближайшую свободную
        double valid_gx = blackboard_->target_pos.goal.x;
        double valid_gy = blackboard_->target_pos.goal.y;

        // Выбираем шаг поиска в зависимости от режима (Exploration или Manual)
        float search_step = blackboard_->exploration_mode ? 
                        blackboard_->params.planning.search_step_explore : 
                        blackboard_->params.planning.search_step_manual;

        bool goal_ok = blackboard_->global_planner->findNearestSafePoint(
            blackboard_->grid_map, 
            valid_gx, valid_gy, 
            blackboard_->params.planning.footprint_radius,
            search_step,
            blackboard_->obstacle_manager.get()
        );

        if (!goal_ok) {
            BTLogger::logMessage(name_, " Goal Search FAILURE");
            BTLogger::logStatusChange(name_ + "_GoalSearch", NodeStatus::FAILURE);
            blackboard_->is_replan_succseed = false;
            if(!blackboard_->is_stuck){
                blackboard_->is_stuck = true;
            }
            return NodeStatus::FAILURE; 
        }

        // 2. Строим план уже к скорректированной цели

        auto actual_reach_goal_dist = blackboard_->exploration_mode ? 
        blackboard_->params.control.explore_goal_reach_dist :
        blackboard_->params.control.goal_reach_dist;

        auto raw_path = blackboard_->global_planner->makePlan(
            blackboard_->grid_map,
            blackboard_->current_pos.x, blackboard_->current_pos.y,
            valid_gx, valid_gy,
            blackboard_->params.planning.footprint_radius,
            blackboard_->params.recovery.max_recovery_dist,
            actual_reach_goal_dist,
            !blackboard_->exploration_mode,
            blackboard_->obstacle_manager.get()
        );

        blackboard_->last_inflation_points = blackboard_->global_planner->getLastInflationPoints();
        blackboard_->inflation_updated = true;

        if (raw_path.empty()) {
            blackboard_->is_replan_succseed = false;
            blackboard_->has_goal = false;
            BTLogger::logMessage(name_, " FAILURE");
            BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
            return NodeStatus::FAILURE;
        }

        // Если путь найден, обновляем Blackboard
        std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
        blackboard_->current_path.clear();
        for (const auto& pt : raw_path) {
            blackboard_->current_path.push_back({pt.first, pt.second, false});
        }

        blackboard_->can_move = true;
        blackboard_->is_replan_succseed = true;
        blackboard_->obstacle_manager->resetConfidence();
        blackboard_->last_replan_time = std::chrono::steady_clock::now();
        BTLogger::logMessage(name_, " SUCCESS");
        BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
        return NodeStatus::SUCCESS;
    }
};

} // namespace jeweler_nav
