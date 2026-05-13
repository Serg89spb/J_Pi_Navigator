#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ConditionCanMove : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        if(blackboard_->can_move){
            BTLogger::logStatusChange(name_, NodeStatus::SUCCESS); 
            return NodeStatus::SUCCESS;
        }
        BTLogger::logStatusChange(name_, NodeStatus::FAILURE); 
        return NodeStatus::FAILURE;
    }
};

} // namespace jeweler_nav