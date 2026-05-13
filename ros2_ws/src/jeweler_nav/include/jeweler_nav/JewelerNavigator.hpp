// Copyright (c) 2026 Sergey Shavlyuga | Jeweler Project | MIT License

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Наши новые компоненты
#include "jeweler_nav/Variables.hpp"
#include "jeweler_nav/Blackboard.hpp"
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class JewelerNavigator : public rclcpp::Node {
public:
    JewelerNavigator();
    virtual ~JewelerNavigator();

private:
    // --- ROS Интерфейсы ---
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr inflation_pub_;

    // --- Таймеры ---
    rclcpp::TimerBase::SharedPtr timer_bt_tick_; // Основной цикл 50мс

    // --- Behavior Tree ---
    std::shared_ptr<Blackboard> blackboard_;
    NodePtr tree_root_;

    void initBehaviorTree();
    void tickBehaviorTree();
    void updateVisuals();

    // --- ROS Колбэки (наполнение Blackboard) ---
    void on_map_received(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg);
    //void on_pose_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void update_robot_pose_from_tf();
    void on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void on_cam_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void on_explore_toggle(const std_msgs::msg::Bool::SharedPtr msg);

    void publish_path_line();
    void publish_path_markers();
    void publish_obstacles();
    void publish_exploration_goal(double x, double y);
    void publishInflationPoints();

    // Вспомогательные объекты (теперь хранятся в Blackboard, здесь только для инициализации)
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<ObstacleManager> obstacle_manager_;

    // Для обновлнеия трансформации
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ПАБЛИШЕРЫ ДЛЯ ВИЗУАЛИЗАЦИИ:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr camera_obs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr exploration_goal_pub_;
};

} // namespace jeweler_nav
