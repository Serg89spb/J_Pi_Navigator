#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace jeweler_nav {

class ActionStop : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        // Обнуляем линейные и угловые скорости в общей памяти
        blackboard_->cmd_vel.linear.x = 0.0;
        blackboard_->cmd_vel.linear.y = 0.0;
        blackboard_->cmd_vel.angular.z = 0.0;

        // Действие по остановке считается мгновенно успешным
        //BTLogger::logMessage(name_," ActionStop"); 
        blackboard_->can_move = false;
        return NodeStatus::SUCCESS;
    }

    void halt() override {
        // При принудительной остановке узла также гарантируем ноль
        blackboard_->cmd_vel = geometry_msgs::msg::Twist();
    }
};

} // namespace jeweler_nav