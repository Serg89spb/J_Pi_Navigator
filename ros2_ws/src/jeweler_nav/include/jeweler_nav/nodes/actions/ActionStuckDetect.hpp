#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/BTLogger.hpp"
#include <cmath>
#include <chrono>

namespace jeweler_nav {

class ActionStuckDetect : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        auto& p = blackboard_->params.control; 
        auto now = std::chrono::steady_clock::now();
        
        static auto last_tick_time = now;
        double dt = std::chrono::duration<double>(now - last_tick_time).count();
        last_tick_time = now;

        // Линейная скорость из последней команды
        double v_cmd = std::sqrt(std::pow(blackboard_->cmd_vel.linear.x, 2) + 
                                 std::pow(blackboard_->cmd_vel.linear.y, 2));

        auto elapsed = std::chrono::duration<double>(now - blackboard_->window_start_time).count();

        // Интегрируем только если команда значимая
        auto angular = std::abs(blackboard_->current_angular_vel);
        
        if (v_cmd > 0.12 && angular < 0.2) {
            blackboard_->expected_dist += v_cmd * dt;
        }

        //BTLogger::logMessage(name_, "v_cmd: %.2f, ang: %.2ff",  v_cmd, angular);

        if (elapsed >= (double)p.stuck_window_seconds) {
            double actual_dist = std::sqrt(
                std::pow(blackboard_->current_pos.x - blackboard_->window_start_pos.x, 2) + 
                std::pow(blackboard_->current_pos.y - blackboard_->window_start_pos.y, 2)
            );

            //BTLogger::logMessage(name_, "Stuck checker enter, v_cmd: %.2f, ang: %.2f | efficiency: %.2f", 
            //v_cmd, angular, actual_dist / blackboard_->expected_dist);

            if (blackboard_->expected_dist > p.min_expected_dist) {
                double efficiency = actual_dist / blackboard_->expected_dist;

                //BTLogger::logMessage(name_, "Eff: %.3f, Elapsed: %.1f, V_cmd: %.2f , angular: %.2f" , 
                //    efficiency, elapsed, v_cmd, angular);

                if (efficiency < p.efficiency_limit) { 
                    BTLogger::logStatusChange(name_, NodeStatus::FAILURE);
                    //BTLogger::logMessage(name_," FAILURE, Need recovery!");
                    
                    if (blackboard_->obstacle_manager) {
                        double obs_x = blackboard_->current_pos.x + p.stuck_inject_dist * std::cos(blackboard_->current_yaw);
                        double obs_y = blackboard_->current_pos.y + p.stuck_inject_dist * std::sin(blackboard_->current_yaw);
                        blackboard_->obstacle_manager->injectObstacle(obs_x, obs_y, 254);
                    }

                    blackboard_->is_stuck = true;
                    resetWindow(now);
                    return NodeStatus::FAILURE; 
                }
            } else {
                // Если робот стоял, просто сбрасываем окно без логов застревания
                // BTLogger::logMessage(name_, "Idle reset. Exp dist too small: %.3f", blackboard_->expected_dist);
            }
            resetWindow(now);
        }
        BTLogger::logStatusChange(name_, NodeStatus::SUCCESS);
        return NodeStatus::SUCCESS; 
    }

private:
    void resetWindow(std::chrono::steady_clock::time_point now) {
        blackboard_->window_start_time = now;
        blackboard_->window_start_pos = blackboard_->current_pos;
        blackboard_->expected_dist = 0.0;
    }
};

} // namespace jeweler_nav
