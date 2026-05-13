#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ConditionReplanSuccseed : public BTNode {
public:
    using BTNode::BTNode;
    
    NodeStatus tick() override {
        if (blackboard_->has_goal && blackboard_->is_replan_succseed) {
            BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS;
        }
        BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
        return NodeStatus::FAILURE;
    }
};

} // namespace jeweler_nav