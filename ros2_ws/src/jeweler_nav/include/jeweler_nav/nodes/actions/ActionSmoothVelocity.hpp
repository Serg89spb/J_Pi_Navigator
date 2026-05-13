#pragma once
#include "jeweler_nav/BTNode.hpp"
#include <algorithm>

namespace jeweler_nav {

class ActionSmoothVelocity : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        auto& target = blackboard_->cmd_vel;
        auto& last = blackboard_->last_cmd; // Добавим это поле в Blackboard
        auto& p = blackboard_->params.control;

        BTLogger::logMessage(name_, "Smoothing speed: x=%.3f", blackboard_->cmd_vel.linear.x);

        // 1. Сглаживание X
        double diff_x = target.linear.x - last.linear.x;
        last.linear.x += std::clamp(diff_x, -p.max_accel, p.max_accel);

        // 2. Сглаживание Y (Стрейф)
        double y_step = p.max_accel;
        // Логика мягкого торможения стрейфа
        if (std::abs(target.linear.y) < 0.01 && std::abs(last.linear.y) > 0.05) {
            y_step /= 3.0;
        }
        double diff_y = target.linear.y - last.linear.y;
        last.linear.y += std::clamp(diff_y, -y_step, y_step);

        // 3. Сглаживание Z
        double diff_z = target.angular.z - last.angular.z;
        last.angular.z += std::clamp(diff_z, -p.max_alpha, p.max_alpha);

        // Результат сглаживания возвращаем обратно в cmd_vel для публикации
        target = last;

        return NodeStatus::SUCCESS;
    }
};

} // namespace jeweler_nav
