#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <fstream>
#include <string>
#include <sstream>

/**
 * Stanley Controller Node (P-control version)
 * 
 * PID 속도 제어 대신 StanleyController.cpp의 computePID()와 동일한
 * 단순 비례 제어(P-control) 방식으로 변경된 버전입니다.
 */

class StanleyNode : public rclcpp::Node
{
public:
    StanleyNode() : Node("stanley_node"),
                    current_x_(0.0), current_y_(0.0),
                    current_yaw_(0.0), current_speed_(0.0),
                    prev_steering_angle_(0.0), steering_alpha_(0.3)
    {
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&StanleyNode::odom_callback, this, std::placeholders::_1));

        // Stanley Parameters
        this->declare_parameter("csv_file_path", "/home/joon/f1/f1tenth_ws/joon_sim_ws/map_global_path_bound/global_paths/lane_optimal.csv");
        this->declare_parameter("k_e", 0.3);
        this->declare_parameter("k_soft", 0.5);
        this->declare_parameter("max_speed", 5.0);
        this->declare_parameter("min_speed", 1.5);
        this->declare_parameter("wheelbase", 0.3302);
        this->declare_parameter("max_steer", 0.523599); // 30 degrees
        this->declare_parameter("speed_kp", 0.5);
        this->declare_parameter("turn_slowdown_angle", 0.2);
        this->declare_parameter("turn_slowdown_gain", 2.0);
        this->declare_parameter("turn_speed_min_scale", 0.4);

        // Load parameters
        std::string csv_path = this->get_parameter("csv_file_path").as_string();
        k_e_ = this->get_parameter("k_e").as_double();
        k_soft_ = this->get_parameter("k_soft").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        speed_kp_ = this->get_parameter("speed_kp").as_double();
        turn_slowdown_angle_ = this->get_parameter("turn_slowdown_angle").as_double();
        turn_slowdown_gain_ = this->get_parameter("turn_slowdown_gain").as_double();
        turn_speed_min_scale_ = this->get_parameter("turn_speed_min_scale").as_double();
        turn_speed_min_scale_ = std::clamp(turn_speed_min_scale_, 0.0, 1.0);

        // Load waypoints
        if (loadWaypointsFromCSV(csv_path)) {
            RCLCPP_INFO(this->get_logger(), "Stanley: 웨이포인트 로드 완료 (%zu개)", waypoints_.size());
            publishPath();
        } else {
            RCLCPP_ERROR(this->get_logger(), "CSV 파일을 읽을 수 없습니다: %s", csv_path.c_str());
        }

        current_waypoint_index_ = 0;

        // Timer (제어 루프 20Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&StanleyNode::control_callback, this));

        // Timer (경로 시각화)
        path_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&StanleyNode::publishPath, this));

        RCLCPP_INFO(this->get_logger(), "Stanley Controller(P-control) 초기화 완료");
    }

private:
    struct Waypoint {
        double x, y, velocity, heading;
        Waypoint(double x = 0, double y = 0, double v = 0, double h = 0)
            : x(x), y(y), velocity(v), heading(h) {}
    };

    // ROS2 통신
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr path_timer_;

    // Stanley 제어 파라미터
    double k_e_;
    double k_soft_;
    double max_speed_;
    double min_speed_;
    double wheelbase_;
    double max_steer_;
    double speed_kp_;
    double turn_slowdown_angle_;
    double turn_slowdown_gain_;
    double turn_speed_min_scale_;

    // 차량 상태
    double current_x_, current_y_, current_yaw_, current_speed_;
    double prev_steering_angle_;
    double steering_alpha_; // 스티어링 평활화 계수

    // 웨이포인트
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;

    // === 콜백 함수 ===
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        current_x_ = odom_msg->pose.pose.position.x;
        current_y_ = odom_msg->pose.pose.position.y;

        auto q = odom_msg->pose.pose.orientation;
        current_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        current_speed_ = odom_msg->twist.twist.linear.x;
    }

    // === CSV 로드 ===
    bool loadWaypointsFromCSV(const std::string &csv_path)
    {
        std::ifstream csv_file(csv_path);
        if (!csv_file.is_open())
            return false;

        waypoints_.clear();
        std::string line;
        while (std::getline(csv_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;

            while (std::getline(ss, cell, ','))
                row.push_back(cell);

            if (row.size() >= 3)
            {
                try
                {
                    double x = std::stod(row[0]);
                    double y = std::stod(row[1]);
                    double velocity = std::stod(row[2]);
                    waypoints_.emplace_back(x, y, velocity, 0.0);
                }
                catch (const std::exception &)
                {
                    continue;
                }
            }
        }

        csv_file.close();

        if (waypoints_.size() >= 2)
        {
            for (size_t i = 0; i < waypoints_.size() - 1; ++i)
            {
                double dx = waypoints_[i + 1].x - waypoints_[i].x;
                double dy = waypoints_[i + 1].y - waypoints_[i].y;
                waypoints_[i].heading = atan2(dy, dx);
            }
        }
        return !waypoints_.empty();
    }

    // === Stanley 제어 루프 ===
    void control_callback()
    {
        if (waypoints_.empty())
            return;

        size_t closest_idx = findClosestWaypoint();
        double steering_angle = calculateStanleySteeringAngle(closest_idx);

        // 목표 속도
        double target_speed = waypoints_[closest_idx].velocity;
        // StanleyController의 computePID() 방식으로 단순 비례 제어 적용
        double speed_error = target_speed - current_speed_;
        double commanded_speed = target_speed + speed_kp_ * speed_error;

        // 조향량이 클수록 속도를 줄여 과도한 언더스티어를 방지
        double steering_mag = std::abs(steering_angle);
        if (steering_mag > turn_slowdown_angle_) {
            double normalized = steering_mag - turn_slowdown_angle_;
            double slowdown_scale = 1.0 / (1.0 + turn_slowdown_gain_ * normalized);
            slowdown_scale = std::clamp(slowdown_scale, turn_speed_min_scale_, 1.0);
            commanded_speed *= slowdown_scale;
        }

        commanded_speed = std::clamp(commanded_speed, min_speed_, max_speed_);

        // Ackermann 명령 발행
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "ego_racecar/base_link";
        drive_msg.drive.speed = commanded_speed;
        drive_msg.drive.steering_angle = steering_angle;
        drive_pub_->publish(drive_msg);

        // 시각화
        publishClosestPointMarker(waypoints_[closest_idx]);
        publishCurrentPositionMarker();

        static int log_counter = 0;
        if (++log_counter % 20 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Stanley(P): WP=%zu/%zu | Target %.2f m/s | Curr %.2f m/s | Cmd %.2f m/s | Steer %.2f°",
                        closest_idx, waypoints_.size(),
                        target_speed, current_speed_, commanded_speed,
                        steering_angle * 180.0 / M_PI);
        }
    }

    // === 최근접 웨이포인트 ===
    size_t findClosestWaypoint()
    {
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            double dx = current_x_ - waypoints_[i].x;
            double dy = current_y_ - waypoints_[i].y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }
        current_waypoint_index_ = closest_idx;
        return closest_idx;
    }

    // === Stanley 조향각 계산 ===
    double calculateStanleySteeringAngle(size_t closest_idx)
    {
        if (waypoints_.empty() || closest_idx >= waypoints_.size())
            return 0.0;

        const auto &wp = waypoints_[closest_idx];

        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;

        double path_heading = wp.heading;
        double path_cos = cos(path_heading);
        double path_sin = sin(path_heading);

        double cross_track_error = dx * path_sin - dy * path_cos;
        double heading_error = path_heading - current_yaw_;
        while (heading_error > M_PI)
            heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2.0 * M_PI;

        double speed_term = std::max(std::abs(current_speed_), 0.1);
        double cross_track_term = atan(k_e_ * cross_track_error / (k_soft_ + speed_term));

        double steering_angle = heading_error + cross_track_term;
        steering_angle = std::max(-max_steer_, std::min(max_steer_, steering_angle));

        steering_angle = prev_steering_angle_ * (1.0 - steering_alpha_) + steering_angle * steering_alpha_;
        prev_steering_angle_ = steering_angle;

        return steering_angle;
    }

    // === 경로 퍼블리시 ===
    void publishPath()
    {
        if (waypoints_.empty())
            return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (const auto &wp : waypoints_)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = wp.x;
            pose_stamped.pose.position.y = wp.y;

            tf2::Quaternion q;
            q.setRPY(0, 0, wp.heading);
            pose_stamped.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_msg);
    }

    // === 시각화 마커 ===
    void publishClosestPointMarker(const Waypoint &wp)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "closest_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wp.x;
        marker.pose.position.y = wp.y;
        marker.pose.position.z = 0.3;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.a = 1.0;
        marker.color.g = 1.0;
        marker_pub_->publish(marker);
    }

    void publishCurrentPositionMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "current_position";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_x_;
        marker.pose.position.y = current_y_;
        marker.pose.position.z = 0.2;
        marker.scale.x = 0.3;
        marker.scale.y = 0.2;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.8;
        marker.color.b = 0.5;
        marker_pub_->publish(marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyNode>();
    RCLCPP_INFO(node->get_logger(), "=== Stanley Controller (P-control) 시작 ===");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
