#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>  // ✅ For lane selector

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <limits>
#include <chrono>
#include <algorithm>
#include <iomanip>

struct Waypoint { double x, y, speed; };

class CsvPurePursuitNode : public rclcpp::Node {
public:
    CsvPurePursuitNode()
    : Node("pure_pursuit_sim_node"),
      speed_error_integral_(0.0), prev_speed_error_(0.0), prev_commanded_speed_(0.0),
      current_x_(0.0), current_y_(0.0), current_yaw_(0.0), current_speed_(0.0),
      current_lane_index_(0)  // ✅ Initialize lane index
    {
        // ===== Parameters =====
        // ✅ Support multiple lanes (like map_controller)
        declare_parameter<std::vector<std::string>>("lane_csv_paths", std::vector<std::string>{});
        declare_parameter<std::string>("csv_file_path", "/home/joon/joon_sim_ws/map_global_path_bound/global_paths/global_optimal.csv");
        declare_parameter<std::string>("path_frame_id", "map");
        declare_parameter<std::string>("planned_path_topic", "/planned_path");
        declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        declare_parameter<std::string>("drive_topic", "/drive");
        declare_parameter<std::string>("lane_selector_topic", "/lane_selector/target_lane");  // ✅ Lane selector topic

        // Adaptive lookahead parameters
        declare_parameter<double>("min_lookahead_distance", 0.8);
        declare_parameter<double>("max_lookahead_distance", 2.5);
        declare_parameter<double>("lookahead_gain", 0.4);
        declare_parameter<double>("lookahead_offset", 0.5);
        declare_parameter<double>("wheelbase", 0.3302);
        declare_parameter<int>("publish_path_period_ms", 500);
        declare_parameter<int>("control_period_ms", 50);

        // Speed control parameters
        declare_parameter<double>("max_speed", 6.0);
        declare_parameter<double>("min_speed", 2.0);

        // PID parameters for speed control
        declare_parameter<bool>("enable_pid", true);  // PID on/off switch
        declare_parameter<double>("speed_kp", 0.3);
        declare_parameter<double>("speed_ki", 0.01);
        declare_parameter<double>("speed_kd", 0.05);
        declare_parameter<double>("speed_change_rate_limit", 2.0);

        // ==== Get parameters ====
        get_parameter("csv_file_path", csv_path_);
        get_parameter("path_frame_id", path_frame_);
        get_parameter("planned_path_topic", planned_path_topic_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("drive_topic", drive_topic_);
        get_parameter("lane_selector_topic", lane_selector_topic_);  // ✅ Get lane selector topic

        get_parameter("min_lookahead_distance", min_lookahead_distance_);
        get_parameter("max_lookahead_distance", max_lookahead_distance_);
        get_parameter("lookahead_gain", lookahead_gain_);
        get_parameter("lookahead_offset", lookahead_offset_);
        get_parameter("wheelbase", wheelbase_);
        int pub_ms, ctrl_ms;
        get_parameter("publish_path_period_ms", pub_ms);
        get_parameter("control_period_ms", ctrl_ms);

        get_parameter("max_speed", max_speed_);
        get_parameter("min_speed", min_speed_);

        get_parameter("enable_pid", enable_pid_);
        get_parameter("speed_kp", speed_kp_);
        get_parameter("speed_ki", speed_ki_);
        get_parameter("speed_kd", speed_kd_);
        get_parameter("speed_change_rate_limit", speed_change_rate_limit_);

        RCLCPP_INFO(get_logger(), "PID Controller: %s", enable_pid_ ? "ENABLED" : "DISABLED");

        // Initialize PID state
        last_control_time_ = now();

        // ===== TF buffer/listener =====
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ===== Load CSV(s) - Support multiple lanes =====
        std::vector<std::string> lane_paths;
        get_parameter("lane_csv_paths", lane_paths);

        if (!lane_paths.empty()) {
            // ✅ Multi-lane mode
            RCLCPP_INFO(get_logger(), "🛣️ Multi-lane mode: loading %zu lanes", lane_paths.size());
            for (size_t i = 0; i < lane_paths.size(); ++i) {
                std::vector<Waypoint> lane_waypoints;
                if (load_csv_to_vector(lane_paths[i], lane_waypoints)) {
                    all_lanes_.push_back(lane_waypoints);
                    RCLCPP_INFO(get_logger(), "  Lane %zu: %s (%zu points)", i, lane_paths[i].c_str(), lane_waypoints.size());
                } else {
                    RCLCPP_ERROR(get_logger(), "  Failed to load lane %zu: %s", i, lane_paths[i].c_str());
                }
            }

            if (all_lanes_.empty()) {
                RCLCPP_FATAL(get_logger(), "No lanes loaded successfully!");
                throw std::runtime_error("No lanes loaded");
            }

            // Set initial lane
            waypoints_map_ = all_lanes_[0];
            multi_lane_mode_ = true;

        } else {
            // ✅ Single-lane mode (backward compatibility)
            RCLCPP_INFO(get_logger(), "🛣️ Single-lane mode: loading %s", csv_path_.c_str());
            if (!load_csv()) {
                RCLCPP_FATAL(get_logger(), "CSV 로드 실패: %s", csv_path_.c_str());
                throw std::runtime_error("CSV load failed");
            }
            multi_lane_mode_ = false;
        }

        RCLCPP_INFO(get_logger(), "✅ Loaded %zu waypoints for current lane (frame=%s)",
                    waypoints_map_.size(), path_frame_.c_str());

        // ===== Publishers/Subscribers =====
        path_pub_  = create_publisher<nav_msgs::msg::Path>(planned_path_topic_, 10);
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
        opp_drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/opp_drive", 10);
        target_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/target_speed", 10);
        marker_pub_= create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&CsvPurePursuitNode::odom_callback, this, std::placeholders::_1)
        );

        // ✅ Subscribe to lane selector (multi-lane mode only)
        if (multi_lane_mode_) {
            lane_selector_sub_ = create_subscription<std_msgs::msg::Int32>(
                lane_selector_topic_, 10,
                std::bind(&CsvPurePursuitNode::lane_selector_callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(get_logger(), "🛣️ Subscribed to lane selector: %s", lane_selector_topic_.c_str());
        }

        // ===== Timers =====
        path_timer_ = create_wall_timer(
            std::chrono::milliseconds(pub_ms),
            std::bind(&CsvPurePursuitNode::publish_path_timer, this)
        );

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(ctrl_ms),
            std::bind(&CsvPurePursuitNode::control_timer, this)
        );

        RCLCPP_INFO(get_logger(), "Node ready. odom=%s, drive=%s, planned_path=%s",
                    odom_topic_.c_str(), drive_topic_.c_str(), planned_path_topic_.c_str());
    }

private:
    // ===== CSV load =====
    bool load_csv() {
        std::ifstream f(csv_path_);
        if (!f.is_open()) return false;

        std::string line;
        waypoints_map_.clear();
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string c0, c1, c2;
            if (!std::getline(ss, c0, ',')) continue;
            if (!std::getline(ss, c1, ',')) continue;
            if (!std::getline(ss, c2, ',')) c2 = "0";
            try {
                double x = std::stod(c0);
                double y = std::stod(c1);
                double speed = std::stod(c2);
                waypoints_map_.push_back({x, y, speed});
            } catch(...) { continue; }
        }
        return !waypoints_map_.empty();
    }

    // ✅ Load CSV into a vector (for multi-lane support)
    bool load_csv_to_vector(const std::string& path, std::vector<Waypoint>& waypoints) {
        std::ifstream f(path);
        if (!f.is_open()) return false;

        std::string line;
        waypoints.clear();
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string c0, c1, c2;
            if (!std::getline(ss, c0, ',')) continue;
            if (!std::getline(ss, c1, ',')) continue;
            if (!std::getline(ss, c2, ',')) c2 = "0";
            try {
                double x = std::stod(c0);
                double y = std::stod(c1);
                double speed = std::stod(c2);
                waypoints.push_back({x, y, speed});
            } catch(...) { continue; }
        }
        return !waypoints.empty();
    }

    // ✅ Lane selector callback
    void lane_selector_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (!multi_lane_mode_) return;

        int desired_lane = msg->data;
        if (desired_lane < 0 || desired_lane >= static_cast<int>(all_lanes_.size())) {
            RCLCPP_WARN(get_logger(), "Invalid lane index: %d (available: 0-%zu)",
                        desired_lane, all_lanes_.size() - 1);
            return;
        }

        if (desired_lane != current_lane_index_) {
            RCLCPP_INFO(get_logger(), "🛣️ Lane change: %d -> %d", current_lane_index_, desired_lane);
            current_lane_index_ = desired_lane;
            waypoints_map_ = all_lanes_[current_lane_index_];
        }
    }

    // ===== Publish /planned_path =====
    void publish_path_timer() {
        if (waypoints_map_.empty()) return;
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = path_frame_;

        path.poses.reserve(waypoints_map_.size());
        for (const auto& wp : waypoints_map_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = wp.x;
            ps.pose.position.y = wp.y;
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        }
        path_pub_->publish(path);
    }

    // ===== Odom callback =====
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        const auto &q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        current_speed_ = msg->twist.twist.linear.x;
    }

    // ===== Main control loop =====
    void control_timer() {
        if (waypoints_map_.empty()) return;

        // Adaptive lookahead distance: lookahead = offset + gain * |speed|
        double adaptive_lookahead = std::clamp(
            lookahead_offset_ + lookahead_gain_ * std::abs(current_speed_),
            min_lookahead_distance_,
            max_lookahead_distance_
        );

        // Lookahead point 찾기
        bool found = false;
        double lx=0, ly=0;
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        // 가장 가까운 포인트
        for (size_t i=0; i<waypoints_map_.size(); ++i) {
            double dx = waypoints_map_[i].x - current_x_;
            double dy = waypoints_map_[i].y - current_y_;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 앞쪽으로 lookahead 검색 (순환)
        size_t waypoint_count = waypoints_map_.size();
        for (size_t offset = 0; offset < waypoint_count; ++offset) {
            size_t i = (closest_idx + offset) % waypoint_count;
            double dx = waypoints_map_[i].x - current_x_;
            double dy = waypoints_map_[i].y - current_y_;
            double dist = std::hypot(dx, dy);

            double local_x = std::cos(-current_yaw_) * dx - std::sin(-current_yaw_) * dy;
            if (local_x > 0.1 && dist >= adaptive_lookahead) {
                lx = waypoints_map_[i].x;
                ly = waypoints_map_[i].y;
                found = true;
                break;
            }
        }
        if (!found) {
            size_t next_idx = (closest_idx + 5) % waypoint_count;
            lx = waypoints_map_[next_idx].x;
            ly = waypoints_map_[next_idx].y;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No forward lookahead found, using waypoint %zu. Closest: idx=%zu, dist=%.2f",
                next_idx, closest_idx, min_dist);
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Vehicle: (%.2f,%.2f,%.2f°) | Lookahead: (%.2f,%.2f) | Ld=%.2f | Closest idx=%zu,dist=%.2f",
            current_x_, current_y_, current_yaw_*180/M_PI, lx, ly, adaptive_lookahead, closest_idx, min_dist);

        // 로컬 좌표
        double dx = lx - current_x_;
        double dy = ly - current_y_;
        double cx =  std::cos(-current_yaw_) * dx - std::sin(-current_yaw_) * dy;
        double cy =  std::sin(-current_yaw_) * dx + std::cos(-current_yaw_) * dy;

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        //     "Global dx=%.2f, dy=%.2f | Local cx=%.2f, cy=%.2f",
        //     dx, dy, cx, cy);

        if (cx <= 0.05) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Lookahead behind/too close: cx=%.2f, cy=%.2f, global_dist=%.2f",
                cx, cy, std::hypot(dx, dy));
        }

        // Pure Pursuit
        double Ld = std::max(adaptive_lookahead, std::hypot(cx, cy));
        double curvature = 2.0 * cy / (Ld * Ld);
        double steer = std::atan(wheelbase_ * curvature);
        double steer_clamped = std::clamp(steer, -M_PI/4.0, M_PI/4.0);

        // Use speed from CSV waypoint (pre-computed based on global path curvature)
        double target_speed = waypoints_map_[closest_idx].speed;
        // Fallback to max_speed if CSV speed is 0 or invalid
        if (target_speed <= 0.0) {
            target_speed = max_speed_;
        }
        target_speed = std::clamp(target_speed, min_speed_, max_speed_);

        // PID (on/off based on parameter)
        double commanded_speed;
        if (enable_pid_) {
            commanded_speed = calculate_pid_speed(target_speed, current_speed_);
        } else {
            // PID disabled: use target speed directly
            commanded_speed = target_speed;
        }

        // Publish command
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "ego_racecar/base_link";
        cmd.drive.speed = commanded_speed;
        cmd.drive.steering_angle = steer_clamped;
        drive_pub_->publish(cmd);

        // Publish target speed
        std_msgs::msg::Float64 target_speed_msg;
        target_speed_msg.data = target_speed;
        target_speed_pub_->publish(target_speed_msg);

        // 시각화 마커 업데이트
        publish_lookahead_marker(lx, ly, adaptive_lookahead);
    }

    void publish_lookahead_marker(double x, double y, double adaptive_ld) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = path_frame_;
        m.header.stamp = now();
        m.ns = "lookahead";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.3;
        m.pose.orientation.w = 1.0;
        double scale = 0.2 + adaptive_ld * 0.1;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;
        marker_pub_->publish(m);
    }

    // ===== PID speed control =====
    double calculate_pid_speed(double target_speed, double current_speed) {
        rclcpp::Time current_time = now();
        double dt = (current_time - last_control_time_).seconds();

        if (dt <= 0.0 || dt > 0.2) {  // Reset if dt is invalid or too large
            speed_error_integral_ = 0.0;
            prev_speed_error_ = 0.0;
            last_control_time_ = current_time;
            return target_speed;
        }

        double error = target_speed - current_speed;

        // Integral term (with windup protection)
        speed_error_integral_ += error * dt;
        speed_error_integral_ = std::clamp(speed_error_integral_, -10.0, 10.0);

        // Derivative term
        double error_derivative = (error - prev_speed_error_) / dt;

        // PID calculation
        double pid_output = speed_kp_ * error +
                           speed_ki_ * speed_error_integral_ +
                           speed_kd_ * error_derivative;

        // Apply PID output to current speed
        double commanded_speed = current_speed + pid_output;

        // Rate limiting
        if (prev_commanded_speed_ != 0.0) {
            double max_speed_change = speed_change_rate_limit_ * dt;
            double speed_diff = commanded_speed - prev_commanded_speed_;
            speed_diff = std::clamp(speed_diff, -max_speed_change, max_speed_change);
            commanded_speed = prev_commanded_speed_ + speed_diff;
        }

        commanded_speed = std::clamp(commanded_speed, min_speed_, max_speed_);

        prev_speed_error_ = error;
        prev_commanded_speed_ = commanded_speed;
        last_control_time_ = current_time;

        return commanded_speed;
    }

private:
    // Params
    std::string csv_path_, path_frame_, planned_path_topic_, odom_topic_, drive_topic_;
    std::string lane_selector_topic_;  // ✅ Lane selector topic

    // Adaptive lookahead parameters
    double min_lookahead_distance_, max_lookahead_distance_, lookahead_gain_, lookahead_offset_;
    double wheelbase_;

    // Speed control parameters
    double max_speed_, min_speed_;

    // PID control parameters
    bool enable_pid_;  // PID on/off flag
    double speed_kp_, speed_ki_, speed_kd_;
    double speed_change_rate_limit_;

    // PID state variables
    double speed_error_integral_;
    double prev_speed_error_;
    double prev_commanded_speed_;
    rclcpp::Time last_control_time_;

    // Data
    std::vector<Waypoint> waypoints_map_;

    // ✅ Multi-lane support
    std::vector<std::vector<Waypoint>> all_lanes_;  // All available lanes
    bool multi_lane_mode_;                          // Flag for multi-lane mode

    // State (must match initialization order in constructor)
    double current_x_, current_y_, current_yaw_, current_speed_;
    int current_lane_index_;                         // Currently selected lane

    // ROS I/F
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr opp_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_speed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lane_selector_sub_;  // ✅ Lane selector subscription
    rclcpp::TimerBase::SharedPtr path_timer_, control_timer_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CsvPurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
