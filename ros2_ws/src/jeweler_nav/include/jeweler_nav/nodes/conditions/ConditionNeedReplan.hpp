#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ConditionNeedReplan : public BTNode {
public:
    using BTNode::BTNode;
    
    NodeStatus tick() override {
        // Просто проверяем флаг в общей памяти
        if (blackboard_->has_goal && blackboard_->is_need_replan) {
            //BTLogger::logMessage(name_, "%.2f]",elapsed);
            BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS;
        }
        BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
        return NodeStatus::FAILURE;
    }
};

} // namespace jeweler_nav