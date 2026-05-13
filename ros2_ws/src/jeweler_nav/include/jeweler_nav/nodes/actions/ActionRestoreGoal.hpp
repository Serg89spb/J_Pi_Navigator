#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace jeweler_nav {

class ActionRestoreGoal : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        blackboard_->has_goal = true;
        {
        std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
        blackboard_->current_path.clear();
        }
        blackboard_->is_need_replan = true;
        blackboard_->is_stuck = false;
        BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
        BTLogger::logMessage(name_, "Restore goal from: |Type: %s, X:%.2f, Y:%.2f| - to |Type: %s, X:%.2f, Y:%.2f|",
            blackboard_->target_pos.type_str(), 
            blackboard_->target_pos.goal.x, 
            blackboard_->target_pos.goal.y, 
            blackboard_->backup_target.type_str(), 
            blackboard_->backup_target.goal.x, 
            blackboard_->backup_target.goal.y);

        if(blackboard_->backup_target.type == GoalType::ORIGINAL){
            blackboard_->target_pos = blackboard_->backup_target;
        }
        return NodeStatus::SUCCESS;
    }
};

} // namespace jeweler_nav