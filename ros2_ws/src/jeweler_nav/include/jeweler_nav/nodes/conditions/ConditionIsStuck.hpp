#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ConditionIsStuck : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
         // Проверяем флаг, который взводит ActionStuckDetect
        auto now = std::chrono::steady_clock::now();
            // Если мы в режиме исследования и время вышло
        if(blackboard_->exploration_mode && !blackboard_->is_stuck && now > blackboard_->exploration_deadline){
            double overtime = std::chrono::duration<double>(now - blackboard_->exploration_deadline).count();
            BTLogger::logMessage(name_, "Deadline EXPIRED by %.1f sec. Ban frontier.", overtime);
            
            // Добавляем текущий неудачный фронтир в бан
            blackboard_->black_list_frontiers.push_back(blackboard_->current_exploration_frontier);
            
            // Сбрасываем цель, чтобы Exploration выбрал новую
            blackboard_->has_goal = false;
            return NodeStatus::FAILURE; 
        }
        if(blackboard_->is_stuck) {
            return NodeStatus::SUCCESS;
        }

        return NodeStatus::FAILURE;
    }
};

} // namespace jeweler_nav
