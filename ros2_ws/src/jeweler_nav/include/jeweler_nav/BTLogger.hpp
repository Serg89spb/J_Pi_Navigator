// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#pragma once
#include <string>
#include <unordered_map>
#include <cstdarg> // Для работы с переменным числом аргументов
#include "rclcpp/rclcpp.hpp"
#include "jeweler_nav/Variables.hpp"

namespace jeweler_nav {

class BTLogger {
public:
    static void logStatusChange(const std::string& node_name, NodeStatus current_status) {
        static std::unordered_map<std::string, NodeStatus> last_states;
        if (last_states.find(node_name) == last_states.end() || last_states[node_name] != current_status) {
            std::string status_str = statusToString(current_status);
            RCLCPP_INFO(rclcpp::get_logger("BT"), "[%s] -> %s", node_name.c_str(), status_str.c_str());
            last_states[node_name] = current_status;
        }
    }

    // НОВАЯ ФУНКЦИЯ: Логирование произвольных сообщений с форматированием
    static void logMessage(const std::string& node_name, const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        RCLCPP_INFO(rclcpp::get_logger("BT_DEBUG"), "[%s]: %s", node_name.c_str(), buffer);
        // Если нужно видеть всегда, замените на RCLCPP_INFO
    }

private:
    static std::string statusToString(NodeStatus status) {
        switch (status) {
            case NodeStatus::SUCCESS: return "SUCCESS";
            case NodeStatus::FAILURE: return "FAILURE";
            case NodeStatus::RUNNING: return "RUNNING";
            default: return "UNKNOWN";
        }
    }
};

} // namespace jeweler_nav
