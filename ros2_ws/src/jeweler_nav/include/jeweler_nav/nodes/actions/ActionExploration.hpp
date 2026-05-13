#pragma once
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/BTLogger.hpp"
#include <cmath>

namespace jeweler_nav {

class ActionExploration : public BTNode {
private:
    bool isInBlacklist(const GridMap::Frontier& f) {
        for (const auto& bad_f : blackboard_->black_list_frontiers) {
            double d = std::sqrt(std::pow(f.x - bad_f.x, 2) + std::pow(f.y - bad_f.y, 2));
            // Если фронтир ближе чем радиус бана (например, 0.5м) к забаненному - игнорим
            if (d < blackboard_->params.exploration.blacklist_radius) {
                return true;
            }
        }
        return false;
    }

    std::vector<std::pair<double, double>> path_;
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        // 1. Если режим выключен или цель уже есть — узел не работает
        if (!blackboard_->grid_map || !blackboard_->exploration_mode) {
            return NodeStatus::FAILURE;
        }

        if (blackboard_->has_goal && !blackboard_->current_path.empty() && blackboard_->is_replan_succseed) {
            //BTLogger::logMessage(name_, "FAILURE was replan success!");
            BTLogger::logStatusChange(name_ + "was replan success ",NodeStatus::FAILURE);
            return NodeStatus::FAILURE;
        }

        if(blackboard_->has_goal){
            //BTLogger::logMessage(name_, "FAILURE has goal!");
            BTLogger::logStatusChange(name_ + "has goal ",NodeStatus::FAILURE);
            return NodeStatus::FAILURE;
        }

        // 2. Ищем все фронтиры через GridMap
        auto raw_frontiers = blackboard_->grid_map->findFrontiers(
            blackboard_->params.exploration.frontier_min_dist, 
            blackboard_->params.exploration.frontier_max_dist, 
            blackboard_->current_pos.x, 
            blackboard_->current_pos.y
        );

        if (raw_frontiers.empty()) {
            BTLogger::logStatusChange(name_ + "map is finished ", NodeStatus::SUCCESS); // Карта готова
            blackboard_->exploration_mode = false;
            return NodeStatus::SUCCESS;
        }

        auto dist = [](const auto& f, const auto& pos) {
            return std::sqrt(std::pow(f.x - pos.x, 2) + std::pow(f.y - pos.y, 2));
        };

        // 2. Берем 3-5 ближайших кластеров (не из черного списка)
        // Сортируем по дистанции и выбираем топ-5
        std::sort(raw_frontiers.begin(), raw_frontiers.end(), [&](const auto& a, const auto& b) {
            return dist(a, blackboard_->current_pos) < dist(b, blackboard_->current_pos);
        });

        for (size_t i = 0; i < std::min(raw_frontiers.size(), (size_t)5); ++i) {
            const auto& f = raw_frontiers[i];
            if (isInBlacklist(f)) continue;

            // 3. Итеративный поиск точки ПОДСТУПА к фронтиру
            double search_r = 0.1; // Начинаем с 10 см
            double step = 0.15;    // Шаг расширения
            bool found_way = false;
            double target_x = f.x, target_y = f.y;

            while (search_r < dist(f, blackboard_->current_pos)) {
                // Ищем ближайшую БЕЗОПАСНУЮ точку к фронтиру в текущем радиусе
                if (blackboard_->global_planner->findNearestSafePoint(
                    blackboard_->grid_map, 
                    target_x, target_y, 
                    blackboard_->params.planning.footprint_radius, 
                    search_r,
                    blackboard_->obstacle_manager.get())) {
    
                    // Пытаемся построить маршрут до этой точки
                    path_ = blackboard_->global_planner->solveAStar(
                        blackboard_->grid_map, 
                        blackboard_->current_pos.x, blackboard_->current_pos.y, 
                        target_x, target_y, 
                        blackboard_->params.planning.footprint_radius, 0.3,
                        blackboard_->params.control.explore_goal_reach_dist, 
                        blackboard_->obstacle_manager.get());
                    
                    if (!path_.empty()) {
                        BTLogger::logMessage(name_, "found way path size: %.d",path_.size());
                        found_way = true;
                        break; // Нашли проходимую точку подступа!
                    }
                }
                search_r += step; // Не смогли доехать — расширяем поиск "чистого" места
                BTLogger::logMessage(name_,"new search r: %.2f", search_r);
            }

            if (found_way) {
                // 1. Считаем длину пути
                double path_length = 0.0;
                for (size_t i = 1; i < path_.size(); ++i) {
                    path_length += std::sqrt(std::pow(path_[i].first - path_[i-1].first, 2) + 
                                            std::pow(path_[i].second - path_[i-1].second, 2));
                }

                BTLogger::logMessage(name_, "path lenght: %.2f, size: %d",path_length, path_.size());

                // 2. Рассчитываем лимит времени (например: путь / скорость * коэффициент запаса)
                // Допустим, скорость 0.2 м/с, коэффициент 3.0 (на развороты и тупняки)
                double expected_velocity = 0.15; // м/с
                double safety_factor = 3.0;
                double travel_time  = (path_length / expected_velocity) * safety_factor + 10.0;
                auto now = std::chrono::steady_clock::now();

                // 3. Записываем в blackboard
                blackboard_->exploration_deadline = now + 
                                    std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::duration<double>(travel_time));

                
                double timeout_sec = std::chrono::duration<double>(blackboard_->exploration_deadline - now).count();
                // Сохраняем текущий фронтир, чтобы знать, кого банить в случае провала
                blackboard_->current_exploration_frontier = {f.x, f.y, f.size};


                blackboard_->target_pos.goal.x = target_x;
                blackboard_->target_pos.goal.y = target_y;
                blackboard_->has_goal = true;
                blackboard_->exploration_updated = true;
                BTLogger::logMessage(name_, "SUCCESS, goal: X:%.2f,Y:%.2f. Timeout: %.1f sec", 
                    target_x, target_y, timeout_sec);
                BTLogger::logStatusChange(name_ + "find new goal ",NodeStatus::SUCCESS);
                return NodeStatus::SUCCESS;
            } else {
                // Если даже расширение радиуса до самого робота не помогло — в бан этот кластер
                blackboard_->black_list_frontiers.push_back({f.x, f.y, f.size});
            }
        }
        blackboard_->has_goal = false;
        blackboard_->exploration_updated = false;
        BTLogger::logStatusChange(name_ + "can't find goal ",NodeStatus::FAILURE);
        BTLogger::logMessage(name_, "FAILURE");
        return NodeStatus::FAILURE;
    }
};

} // namespace jeweler_nav
