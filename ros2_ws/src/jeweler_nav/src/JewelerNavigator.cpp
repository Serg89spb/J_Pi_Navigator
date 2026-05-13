#include "jeweler_nav/JewelerNavigator.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Базовые компоненты BT
#include "jeweler_nav/BTNode.hpp"
#include "jeweler_nav/Sequence.hpp"
#include "jeweler_nav/Selector.hpp"
#include "jeweler_nav/nodes/decorators/Inverter.hpp"

// Условия (Conditions)
#include "jeweler_nav/nodes/conditions/ConditionHasGoal.hpp"
#include "jeweler_nav/nodes/conditions/ConditionStartValid.hpp"
#include "jeweler_nav/nodes/conditions/ConditionPathBlocked.hpp"
#include "jeweler_nav/nodes/conditions/ConditionAlwaysSuccess.hpp"
#include "jeweler_nav/nodes/conditions/ConditionIsStuck.hpp"
#include "jeweler_nav/nodes/conditions/ConditionNeedReplan.hpp"
#include "jeweler_nav/nodes/conditions/ConditionCanMove.hpp"
#include "jeweler_nav/nodes/conditions/ConditionReplanSuccseed.hpp"

// Действия (Actions)
#include "jeweler_nav/nodes/actions/ActionStop.hpp"
#include "jeweler_nav/nodes/actions/ActionRecovery.hpp"
#include "jeweler_nav/nodes/actions/ActionMakePlan.hpp"
#include "jeweler_nav/nodes/actions/ActionFollowPath.hpp"
#include "jeweler_nav/nodes/actions/ActionSmoothVelocity.hpp"
#include "jeweler_nav/nodes/actions/ActionStuckDetect.hpp"
#include "jeweler_nav/nodes/actions/ActionExploration.hpp"
#include "jeweler_nav/nodes/actions/ActionAnalyzePath.hpp"
#include "jeweler_nav/nodes/actions/ActionRestoreGoal.hpp"

using namespace jeweler_nav;

JewelerNavigator::JewelerNavigator() : Node("jeweler_navigator") {
    
    // 1. Сначала определяем настройки (QoS)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    blackboard_ = std::make_shared<Blackboard>();

    // 2. Теперь создаем подписки, используя эти переменные
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 
        map_qos, 
        std::bind(&JewelerNavigator::on_map_received, this, std::placeholders::_1)
    );

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), 
        std::bind(&JewelerNavigator::on_odom_received, this, std::placeholders::_1));

    //pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //"/pose", 10, std::bind(&JewelerNavigator::on_pose_received, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Остальные подписки (goal и scan) можно оставить со стандартной очередью 10
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&JewelerNavigator::on_goal_received, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&JewelerNavigator::on_scan_received, this, std::placeholders::_1));

    camera_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/camera_scan", 10, std::bind(&JewelerNavigator::on_cam_scan_received, this, std::placeholders::_1));

    explore_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/toggle_exploration", 10, std::bind(&JewelerNavigator::on_explore_toggle, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
    camera_obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/camera_obstacles_debug", 10);
    exploration_goal_pub_ = create_publisher<visualization_msgs::msg::Marker>("/exploration_goal_debug", 10);
    inflation_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/inflation_points", 10);

    // Собираем дерево
    initBehaviorTree();

    // Таймеры
    timer_bt_tick_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&JewelerNavigator::tickBehaviorTree, this));
    blackboard_->global_planner = std::make_shared<GlobalPlanner>();
}

JewelerNavigator::~JewelerNavigator() {}

// --- Обработка данных ---

void JewelerNavigator::on_map_received(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!blackboard_->grid_map) {
        blackboard_->grid_map = std::make_shared<GridMap>(msg);
        if(!blackboard_->obstacle_manager){
            blackboard_->obstacle_manager = std::make_shared<ObstacleManager>(
                msg->info.resolution, msg->info.width, msg->info.height,
                msg->info.origin.position.x, msg->info.origin.position.y,
                blackboard_->params.obstacles);
        }
        RCLCPP_INFO(this->get_logger(), "GridMap and ObstacleManager initialized in Blackboard.");
    } else {
        blackboard_->grid_map->update(msg);
    }
}

void JewelerNavigator::on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (blackboard_) {
        auto q = msg->pose.pose.orientation;
        double new_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        blackboard_->current_angular_vel = msg->twist.twist.angular.z;

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - blackboard_->last_yaw_time).count();
        if (dt > 0.01) {
            double d_yaw = new_yaw - blackboard_->current_yaw;
            // Нормализация
            if (d_yaw > M_PI) d_yaw -= 2.0 * M_PI;
            if (d_yaw < -M_PI) d_yaw += 2.0 * M_PI;
            blackboard_->current_angular_vel = d_yaw / dt;
        }
        blackboard_->current_yaw = new_yaw;
        blackboard_->last_yaw_time = now;
    }
}

// void JewelerNavigator::on_pose_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
//     if (blackboard_) {
//         blackboard_->current_pos = msg->pose.pose.position;
//     }
// }

void JewelerNavigator::update_robot_pose_from_tf() {
    try {
        // Ищем последнюю доступную трансформацию
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        blackboard_->current_pos.x = t.transform.translation.x;
        blackboard_->current_pos.y = t.transform.translation.y;
        //BTLogger::logMessage("NAVIGATOR", "current_pos: x=%2f, y=%2f", t.transform.translation.x, t.transform.translation.y);

    } catch (const tf2::TransformException & ex) {
        // Если SLAM еще не проснулся, просто выходим
        return;
    }
}

void JewelerNavigator::on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    blackboard_->target_pos.goal = msg->pose.position;
    blackboard_->has_goal = true;
    blackboard_->can_move = true;
    
    // Сбрасываем старый путь для новой цели
    {
        std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);
        blackboard_->current_path.clear();
    }
    
    RCLCPP_INFO(this->get_logger(), "New Goal set in Blackboard: X=%.2f, Y=%.2f", 
                blackboard_->target_pos.goal.x, blackboard_->target_pos.goal.y);
}

void JewelerNavigator::on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (blackboard_ && blackboard_->obstacle_manager) {
        blackboard_->last_scan = msg;

        // auto now = std::chrono::steady_clock::now();
        // auto elapsed = std::chrono::duration<double>(now - blackboard_->last_replan_time).count();
        // if(elapsed > 1.0){
            blackboard_->obstacle_manager->updateLidarObstacles(msg,
                blackboard_->cmd_vel.angular.z, blackboard_->current_pos, 
                blackboard_->current_yaw, blackboard_->grid_map,blackboard_->current_path);
        // }
    }
}

void JewelerNavigator::on_cam_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (blackboard_ && blackboard_->obstacle_manager) {
        blackboard_->last_camera_scan = msg;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - blackboard_->last_replan_time).count();
        if(elapsed > 2.0){
        blackboard_->obstacle_manager->updateCamObstacles(msg,
            blackboard_->cmd_vel.angular.z, blackboard_->current_pos, 
            blackboard_->current_yaw, blackboard_->grid_map,blackboard_->current_path);
        }
    }
}


void JewelerNavigator::on_explore_toggle(const std_msgs::msg::Bool::SharedPtr msg) {
    blackboard_->exploration_mode = msg->data;
    if (!blackboard_->exploration_mode) {
        blackboard_->has_goal = false;
    }
}

void JewelerNavigator::initBehaviorTree() {
    if (!blackboard_) {
        blackboard_ = std::make_shared<Blackboard>();
    }
    
    // Сброс окна детектора застревания
    blackboard_->window_start_time = std::chrono::steady_clock::now();
    blackboard_->window_start_pos = blackboard_->current_pos; 
    blackboard_->expected_dist = 0.0;

    auto root = std::make_shared<Selector>("SafetyRoot", blackboard_);

    // --- ВЕТКА 1: ПОЛУЧЕНИЕ ЦЕЛИ (EXPLORATION ИЛИ STOP) ---
    auto goal_selector = std::make_shared<Selector>("GoalSelector", blackboard_);
    // 1.1. Пытаемся найти цель через исследование (сработает, только если has_goal == false)
    goal_selector->addChild(std::make_shared<ActionExploration>("Exploration", blackboard_));

    // 1.2. Если исследование выключено или фронтиров нет — просто стоим
    auto stop_branch = std::make_shared<Sequence>("StopBranch", blackboard_);
    stop_branch->addChild(std::make_shared<Inverter>("GoalMissing?", blackboard_, 
        std::make_shared<ConditionHasGoal>("HasGoal?", blackboard_)));
    stop_branch->addChild(std::make_shared<ActionStop>("StandbyStop", blackboard_));
    goal_selector->addChild(stop_branch);
    root->addChild(goal_selector);

    // --- ВЕТКА 2: СПАСЕНИЕ (RECOVERY) ---
    auto recovery_branch = std::make_shared<Sequence>("RecoveryBranch", blackboard_);
    auto recovery_trigger = std::make_shared<Selector>("RecoveryTrigger", blackboard_);
    // Мы просто застряли (буксуем)
    recovery_trigger->addChild(std::make_shared<ConditionIsStuck>("IsStuck?", blackboard_));

    recovery_branch->addChild(recovery_trigger);
    recovery_branch->addChild(std::make_shared<ActionRecovery>("FindEscapePoint", blackboard_));
    recovery_branch->addChild(std::make_shared<ActionFollowPath>("ExecuteEscape", blackboard_));
    recovery_branch->addChild(std::make_shared<ActionRestoreGoal>("RestoreGoal", blackboard_));

    root->addChild(recovery_branch);


    // --- ВЕТКА 3: ДВИЖЕНИЕ С ЗАЩИТОЙ ---
    auto move_branch = std::make_shared<Selector>("ProtectedMoveBranch", blackboard_);
    
    // 1. ПЛАНИРОВАНИЕ (Делаем один раз или при смене цели)
    auto planning_seq = std::make_shared<Sequence>("PlanningSeq", blackboard_);
    planning_seq->addChild(std::make_shared<ConditionNeedReplan>("NeedReplan", blackboard_));
    planning_seq->addChild(std::make_shared<ActionStop>("StandbyStop", blackboard_));
    planning_seq->addChild(std::make_shared<ActionMakePlan>("InitialPlan", blackboard_));
    planning_seq->addChild(std::make_shared<ActionAnalyzePath>("AnalyzeInitialPath", blackboard_));
    move_branch->addChild(planning_seq);

    auto result_plan_seq = std::make_shared<Sequence>("ResultPlanSeq", blackboard_);
    result_plan_seq->addChild(std::make_shared<Inverter>("ReplanFailed?", blackboard_,
        std::make_shared<ConditionReplanSuccseed>("ReplanSuccseed?", blackboard_)));
    result_plan_seq->addChild(std::make_shared<ActionStop>("StandbyStop", blackboard_));
    move_branch->addChild(result_plan_seq);
    
    auto drive_loop = std::make_shared<Sequence>("DriveLoop", blackboard_);
    
    auto path_block_selec = std::make_shared<Selector>("PathBlockSelec", blackboard_);
    path_block_selec->addChild(std::make_shared<Inverter>("PathFree?", blackboard_,
        std::make_shared<ConditionPathBlocked>("PathBlocked", blackboard_)));
    drive_loop->addChild(path_block_selec);

    // Проверка застревания (вернет SUCCESS если все ок, FAILURE если застряли)
    drive_loop->addChild(std::make_shared<ActionStuckDetect>("StuckCheck", blackboard_));
    // Можем ли двигаться
    drive_loop->addChild(std::make_shared<ConditionCanMove>("CanMove?", blackboard_));
    // Непосредственно движение
    drive_loop->addChild(std::make_shared<ActionFollowPath>("MoveToGoal", blackboard_));

    move_branch->addChild(drive_loop);
    
    root->addChild(move_branch);

    tree_root_ = root;
}

void JewelerNavigator::tickBehaviorTree() {
    if (!blackboard_ || !blackboard_->grid_map) return;
    update_robot_pose_from_tf();

    // 1. Дерево принимает решение
    if (tree_root_) tree_root_->tick();

    // 2. Отправка команды моторам
    cmd_pub_->publish(blackboard_->cmd_vel);

    // 3. Обновление "картинки" для разработчика
    updateVisuals();
}

void JewelerNavigator::publish_path_line() {
    if (!blackboard_ || blackboard_->current_path.empty()) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    // Блокируем мьютекс, так как дерево может пересчитывать путь в другом потоке
    std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);

    for (const auto& pt : blackboard_->current_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = path_msg.header.stamp;
        
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        
        // Ориентация не важна для линии, ставим дефолт
        pose.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
}

void JewelerNavigator::publish_path_markers() {
    if (!blackboard_ || blackboard_->current_path.empty()) return;

    visualization_msgs::msg::MarkerArray msg;
    int id = 0;

    // Сначала очищаем старые маркеры в этой группе (ns)
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.ns = "path_segmentation";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    msg.markers.push_back(clear_marker);

    std::lock_guard<std::recursive_mutex> lock(blackboard_->path_mutex);

    for (const auto& pt : blackboard_->current_path) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "path_segmentation";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        marker.pose.position.z = 0.05; // Сферы чуть над полом

        marker.scale.x = 0.1; 
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Цветовая индикация режима движения из Blackboard
        if (pt.allow_strafing) {
            // ЖЕЛТЫЙ: Разрешен боковой стрейф
            marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; 
        } else {
            // ЗЕЛЕНЫЙ: Обычное прямолинейное движение
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; 
        }
        marker.color.a = 0.8; 

        msg.markers.push_back(marker);
    }
    
    // Используем уже существующий паблишер
    marker_pub_->publish(msg);
}

void JewelerNavigator::publish_obstacles() {
    if (!blackboard_ || !blackboard_->obstacle_manager) return;

    visualization_msgs::msg::MarkerArray msg;

    // --- Очистка старых маркеров в Foxglove ---
    visualization_msgs::msg::Marker clear_msg;
    clear_msg.header.frame_id = "map";
    clear_msg.header.stamp = this->now();
    clear_msg.action = visualization_msgs::msg::Marker::DELETEALL;
    msg.markers.push_back(clear_msg);

    // 1. Отрисовка ЗАНЯТЫХ точек (Красные шарики)
    auto lid_occ_pts = blackboard_->obstacle_manager->getLidarOccupiedPoints();
    if (!lid_occ_pts.empty()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "lidar_debug_occupied";
        m.id = 1000;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 0.02;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
        m.points = lid_occ_pts;
        msg.markers.push_back(m);
    }

    auto cam_occ_pts = blackboard_->obstacle_manager->getCamOccupiedPoints();
    if (!cam_occ_pts.empty()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "lidar_debug_occupied";
        m.id = 1000;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 0.02;
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.8;
        m.points = cam_occ_pts;
        msg.markers.push_back(m);
    }

    // 2. Отрисовка СВОБОДНЫХ (Intruder) точек (Зеленые шарики)
    auto int_lidar_pts = blackboard_->obstacle_manager->getLidarIntruderPoints();
    if (!int_lidar_pts.empty()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "lidar_debug_intruder";
        m.id = 1001;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 0.03; // Зеленые чуть крупнее, их мы ищем
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
        m.points = int_lidar_pts;
        msg.markers.push_back(m);
    }

    auto int_cam_pts = blackboard_->obstacle_manager->getCamIntruderPoints();
    if (!int_cam_pts.empty()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "lidar_debug_intruder";
        m.id = 1002;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 0.03; // Зеленые чуть крупнее, их мы ищем
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
        m.points = int_cam_pts;
        msg.markers.push_back(m);
    }

    auto obstacles = blackboard_->obstacle_manager->getActiveObstacles(); 

    //RCLCPP_INFO(this->get_logger(), "Is 1,0 Occupied? %s", blackboard_->obstacle_manager->isOccupied(1,0) ? "YES" : "NO!");
    
    int id = 0;
    // 1. Отрисовка сетки уверенности (Статика + Камера)
    for (const auto& obs : obstacles) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "obstacle_layer";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.pose.position.x = obs.x;
        m.pose.position.y = obs.y;
        m.pose.position.z = 0.1;
        m.scale.x = m.scale.y = 0.05; 
        m.scale.z = 0.2;

        if (obs.conf == 254) { // Фиолетовый: Метка застревания (STUCK)
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.9;
        } else if (obs.conf >= blackboard_->params.obstacles.confirm_threshold) {
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.6; // Красный: Стена
        } else {
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.3; // Желтый: Шум
        }
        msg.markers.push_back(m);
    }

    camera_obs_pub_->publish(msg);
}


void JewelerNavigator::publish_exploration_goal(double x, double y) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "exploration_target";
    marker.id = 999;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.05; // Повесим повыше, чтобы было видно над роботом

    // Делаем маркер вытянутым по вертикали для заметности
    marker.scale.x = 0.1; 
    marker.scale.y = 0.1;
    marker.scale.z = 0.3;

    // Ярко-фиолетовый цвет (Magenta)
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9;

    exploration_goal_pub_->publish(marker);
}

void JewelerNavigator::publishInflationPoints() {
    // Проверка обновлений, чтобы не спамить в топик одно и то же
    if (!blackboard_ || !blackboard_->inflation_updated || blackboard_->last_inflation_points.empty()) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "grid_inflation";
    marker.id = 55; // Константный ID для перезаписи
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Размер должен соответствовать разрешению твоей сетки (grid_map resolution)
    double res = 0.05; // Поставь тут свое значение, например 0.05
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = 0.01; // Плоские квадраты

    // Цвет "грязный серый" для инфляции
    marker.color.r = 0.3f;
    marker.color.g = 0.3f;
    marker.color.b = 0.3f;
    marker.color.a = 0.5f;

    marker.points.reserve(blackboard_->last_inflation_points.size());

    for (const auto& pt : blackboard_->last_inflation_points) {
        geometry_msgs::msg::Point p;
        // Центрируем куб в ячейке
        p.x = pt.first; 
        p.y = pt.second;
        p.z = 0.005; // Минимальный подъем над картой
        marker.points.push_back(p);
    }

    // Публикуем напрямую Marker, если паблишер позволяет, 
    // или MarkerArray с одним элементом
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(marker);
    inflation_pub_->publish(msg);

    blackboard_->inflation_updated = false;
}


void JewelerNavigator::updateVisuals() {
    // 1. Линия пути (nav_msgs/Path)
    if (blackboard_->path_updated) {
        publish_path_line();    // Линия
        publish_path_markers(); // Сферы стрейфа
        blackboard_->path_updated = false;
    }

    // 2. Препятствия (Camera + Lidar dynamic)
    // Лидар обновляет динамику часто, поэтому флаг можно проверять раз в 500мс через таймер
    // или по флагу из ObstacleManager
    //if (blackboard_->obstacles_updated) {
    publish_obstacles();
    //    blackboard_->obstacles_updated = false;
    //}

    // 3. Цель эксплорации
    if (blackboard_->exploration_updated) {
        publish_exploration_goal(blackboard_->target_pos.goal.x, blackboard_->target_pos.goal.y);
        blackboard_->exploration_updated = false;
    }

    if (blackboard_->inflation_updated) {
    publishInflationPoints();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JewelerNavigator>());
    rclcpp::shutdown();
    return 0;
}
