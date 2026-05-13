#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/LocalReactor.hpp"

namespace jeweler_nav {

class ActionFollowPath : public BTNode {
private:
    std::unique_ptr<LocalReactor> reactor_;
    double last_ang_z_ = 0.0;

public:
    ActionFollowPath(const std::string& name, BlackboardPtr blackboard) 
        : BTNode(name, blackboard) 
    {
        reactor_ = std::make_unique<LocalReactor>();
    }

    NodeStatus tick() override {
        std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
        
        if (blackboard_->current_path.empty()) {
            BTLogger::logMessage(name_, "FAILURE Path empty");
            BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
            return NodeStatus::FAILURE;
        }

        // 1. Очистка хвоста (Snap to robot)
        while (blackboard_->current_path.size() > 1) { // Оставляем хотя бы одну точку (целевую)
            double d = std::sqrt(std::pow(blackboard_->current_path.front().x - blackboard_->current_pos.x, 2) + 
                                std::pow(blackboard_->current_path.front().y - blackboard_->current_pos.y, 2));
            
            if (d < blackboard_->params.control.goal_reach_dist) { 
                blackboard_->current_path.erase(blackboard_->current_path.begin());
            } else {
                break;
            }
        }

        // 2. Проверка финиша (используем параметры)
        double dist_to_final = std::sqrt(std::pow(blackboard_->target_pos.goal.x - blackboard_->current_pos.x, 2) + 
                                        std::pow(blackboard_->target_pos.goal.y - blackboard_->current_pos.y, 2));
        
        auto reach_dist = blackboard_->exploration_mode ? 
        blackboard_->params.control.explore_goal_reach_dist : 
        blackboard_->params.control.goal_reach_dist;

        if (dist_to_final < reach_dist) {
            blackboard_->has_goal = false;
            blackboard_->cmd_vel = geometry_msgs::msg::Twist();
            blackboard_->is_need_replan = true;
            BTLogger::logMessage(name_, "SUCCESS reach goal");
            BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS;
        }

        // 3. ЛОГИКА ВЫБОРА ЦЕЛИ (Сглаживание углов)
        PathPoint target_pt = blackboard_->current_path.front();
        double desired_yaw = blackboard_->current_yaw;

        // Ищем "точку изгиба" в пределах ближайших 10 точек (Sliding Window)
        size_t look_ahead = std::min(blackboard_->current_path.size(), (size_t)10);
        for (size_t i = 0; i < look_ahead; ++i) {
            const auto& p = blackboard_->current_path[i];
            
            // Вектор от робота до конца окна
            double dx = blackboard_->current_path[look_ahead-1].x - blackboard_->current_pos.x;
            double dy = blackboard_->current_path[look_ahead-1].y - blackboard_->current_pos.y;
            double mag = std::sqrt(dx*dx + dy*dy);
            
            if (mag > 0.1) {
                // Расстояние от промежуточной точки до прямой линии движения
                double dist_to_line = std::abs(dy * p.x - dx * p.y + blackboard_->current_path[look_ahead-1].x * blackboard_->current_pos.y 
                                    - blackboard_->current_path[look_ahead-1].y * blackboard_->current_pos.x) / mag;

                // Если путь сильно искривляется (>12см), целимся в точку изгиба
                if (dist_to_line > 0.12) {
                    target_pt = p;
                    break;
                }
                target_pt = blackboard_->current_path[look_ahead-1];
            }
        }

        // 4. ОПРЕДЕЛЕНИЕ ЖЕЛАЕМОГО УГЛА (Yaw)
        if (target_pt.allow_strafing) {
            // При стрейфе сохраняем текущий угол (робот едет боком)
            desired_yaw = blackboard_->current_yaw;
        } else {
            // На прямой линии направляем "нос" на цель
            desired_yaw = std::atan2(target_pt.y - blackboard_->current_pos.y, 
                                    target_pt.x - blackboard_->current_pos.x);
        }

        // 5. Расчет скоростей через LocalReactor
        geometry_msgs::msg::Twist raw_vel = reactor_->computeMecanumVelocity(
            blackboard_->current_pos, 
            blackboard_->current_yaw, 
            target_pt, 
            desired_yaw, 
            false 
        );

        // 5. ВСТРОЕННОЕ СГЛАЖИВАНИЕ (вместо отдельной ноды)
        auto& last = blackboard_->last_cmd;
        auto& p = blackboard_->params.control;

        // Сглаживание X
        double diff_x = raw_vel.linear.x - last.linear.x;
        last.linear.x += std::clamp(diff_x, -p.max_accel, p.max_accel);

        // Сглаживание Y (Стрейф)
        double y_step = p.max_accel;
        if (std::abs(raw_vel.linear.y) < 0.01 && std::abs(last.linear.y) > 0.05) {
            y_step /= 3.0;
        }
        double diff_y = raw_vel.linear.y - last.linear.y;
        last.linear.y += std::clamp(diff_y, -y_step, y_step);

        // Сглаживание Z
        double diff_z = raw_vel.angular.z - last.angular.z;
        last.angular.z += std::clamp(diff_z, -p.max_alpha, p.max_alpha);

        // 6. ЗАПИСЬ РЕЗУЛЬТАТА
        blackboard_->cmd_vel = last; 

        // BTLogger::logMessage(name_, "RUNNING]");
        BTLogger::logStatusChange(name_, NodeStatus::RUNNING);
        return NodeStatus::RUNNING;
    }

    void halt() override {
        blackboard_->cmd_vel = geometry_msgs::msg::Twist();
    }
};

} // namespace jeweler_nav
