#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ConditionHasGoal : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        // Просто проверяем флаг в общей памяти
        if (blackboard_->has_goal) {
            BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS;
        }
        BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
        return NodeStatus::FAILURE;
    }
};

} // namespace jeweler_nav
