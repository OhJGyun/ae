#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

struct Waypoint {
    float x;
    float y;
    float distance;
    int scan_index;
};

struct GapInfo {
    float gap_size;
    int gap_index;
    float gap_diff;
    int goal_index;
    float goal_distance;
};

class DisparityReactiveController : public rclcpp::Node {
public:
    DisparityReactiveController() : Node("disparity_reactive_controller") {
        // Declare and get parameters
        declare_and_get_parameters();

        // Print configuration
        print_configuration();

        // Create publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 10);

        // Create opp_drive publisher for 2-agent scenario
        opp_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/opp_drive", 10);

        // Always create visualization publisher
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/disparity/visualization", 10);

        RCLCPP_INFO(this->get_logger(), "Created visualization publisher on /disparity/visualization");

        // Create subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&DisparityReactiveController::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&DisparityReactiveController::odom_callback, this, _1));

        // Initialize state
        prev_steering_ = 0.0;
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        odom_received_ = false;
        prev_waypoint_x_ = 0.0;
        prev_waypoint_y_ = 0.0;
        waypoint_initialized_ = false;

        RCLCPP_INFO(this->get_logger(), "Disparity Reactive Controller initialized!");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Convert quaternion to yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        odom_received_ = true;
    }

    void declare_and_get_parameters() {
        // Topics
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("drive_topic", "/drive");
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        drive_topic_ = this->get_parameter("drive_topic").as_string();

        // Vehicle parameters
        this->declare_parameter("wheel_base", 0.324);
        this->declare_parameter("car_width", 0.30);
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        car_width_ = this->get_parameter("car_width").as_double();

        // Disparity parameters
        this->declare_parameter("max_range", 15.0);
        this->declare_parameter("gap_threshold", 1.0);
        this->declare_parameter("disparity_margin", 0.35);
        max_range_ = this->get_parameter("max_range").as_double();
        gap_threshold_ = this->get_parameter("gap_threshold").as_double();
        disparity_margin_ = this->get_parameter("disparity_margin").as_double();

        // Control parameters
        this->declare_parameter("v_max", 8.0);
        this->declare_parameter("v_min", 3.0);
        this->declare_parameter("max_steer", 0.45);
        v_max_ = this->get_parameter("v_max").as_double();
        v_min_ = this->get_parameter("v_min").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();

        // Waypoint selection parameters
        this->declare_parameter("lookahead_min", 1.0);
        this->declare_parameter("lookahead_max", 4.0);
        lookahead_min_ = this->get_parameter("lookahead_min").as_double();
        lookahead_max_ = this->get_parameter("lookahead_max").as_double();

        // Control tuning
        this->declare_parameter("steering_gain", 1.0);
        this->declare_parameter("steering_smoothing", 0.6);
        this->declare_parameter("speed_reduction_factor", 0.8);
        steering_gain_ = this->get_parameter("steering_gain").as_double();
        steering_smoothing_ = this->get_parameter("steering_smoothing").as_double();
        speed_reduction_factor_ = this->get_parameter("speed_reduction_factor").as_double();

        // Visualization
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("marker_frame", "map");
        this->declare_parameter("waypoint_marker_size", 0.3);
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        marker_frame_ = this->get_parameter("marker_frame").as_string();
        waypoint_marker_size_ = this->get_parameter("waypoint_marker_size").as_double();

        // Waypoint stability
        this->declare_parameter("enable_waypoint_stability", true);
        this->declare_parameter("waypoint_jump_threshold", 2.0);
        enable_waypoint_stability_ = this->get_parameter("enable_waypoint_stability").as_bool();
        waypoint_jump_threshold_ = this->get_parameter("waypoint_jump_threshold").as_double();

        // Debug
        this->declare_parameter("verbose_logging", true);
        verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
    }

    void print_configuration() {
        RCLCPP_INFO(this->get_logger(), "=================================");
        RCLCPP_INFO(this->get_logger(), "Disparity Reactive Controller Config");
        RCLCPP_INFO(this->get_logger(), "=================================");
        RCLCPP_INFO(this->get_logger(), "Topics:");
        RCLCPP_INFO(this->get_logger(), "  scan: %s", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  drive: %s", drive_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Vehicle:");
        RCLCPP_INFO(this->get_logger(), "  wheel_base: %.3f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "  car_width: %.3f m", car_width_);
        RCLCPP_INFO(this->get_logger(), "Disparity:");
        RCLCPP_INFO(this->get_logger(), "  max_range: %.2f m", max_range_);
        RCLCPP_INFO(this->get_logger(), "  gap_threshold: %.2f m", gap_threshold_);
        RCLCPP_INFO(this->get_logger(), "  disparity_margin: %.2f m", disparity_margin_);
        RCLCPP_INFO(this->get_logger(), "Control:");
        RCLCPP_INFO(this->get_logger(), "  v_max: %.2f m/s", v_max_);
        RCLCPP_INFO(this->get_logger(), "  v_min: %.2f m/s", v_min_);
        RCLCPP_INFO(this->get_logger(), "  max_steer: %.2f rad (%.1f deg)", max_steer_, max_steer_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  steering_gain: %.2f", steering_gain_);
        RCLCPP_INFO(this->get_logger(), "  steering_smoothing: %.2f", steering_smoothing_);
        RCLCPP_INFO(this->get_logger(), "Visualization: %s", enable_visualization_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "=================================");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // Step 1: Preprocess scan data
        std::vector<float> ranges = preprocess_scan(scan_msg);

        // Step 2: Detect disparity and select waypoint
        Waypoint waypoint = detect_disparity_and_select_waypoint(scan_msg, ranges);

        if (waypoint.distance <= 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "No valid waypoint found, sending zero command");
            publish_drive_command(0.0, 0.0);
            return;
        }

        // Step 3: Compute steering angle (reactive pure pursuit)
        float steering_angle = compute_steering_angle(waypoint);

        // Step 4: Compute velocity based on steering
        float velocity = compute_velocity(steering_angle);

        // Step 5: Publish drive command
        publish_drive_command(steering_angle, velocity);

        // Step 6: Visualize waypoint
        if (enable_visualization_) {
            visualize_waypoint(waypoint);
        }

        // Step 7: Log information
        if (verbose_logging_) {
            RCLCPP_INFO(this->get_logger(),
                "Waypoint: (%.2f, %.2f) | Distance: %.2f m | Steering: %.1f deg | Velocity: %.2f m/s",
                waypoint.x, waypoint.y, waypoint.distance,
                steering_angle * 180.0 / M_PI, velocity);
        }
    }

    std::vector<float> preprocess_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        std::vector<float> ranges = scan_msg->ranges;
        int n = ranges.size();

        for (int i = 0; i < n; i++) {
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

            // Only use front 90 degrees
            if (angle < -M_PI / 3.0 || angle > M_PI / 3.0) {
                ranges[i] = 0.0;
                continue;
            }

            // Handle invalid readings
            if (!std::isfinite(ranges[i]) || ranges[i] < 0.0) {
                ranges[i] = 0.0;
            } else if (ranges[i] > max_range_) {
                ranges[i] = max_range_;
            }
        }

        return ranges;
    }

    std::vector<GapInfo> find_all_gaps(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
        const std::vector<float>& ranges) {

        std::vector<GapInfo> gaps;
        int n = ranges.size();

        // Find all significant gaps
        for (int i = 0; i < n - 1; i++) {
            if (ranges[i] <= 0.0 || ranges[i + 1] <= 0.0) continue;

            float diff = ranges[i + 1] - ranges[i];
            float gap_size = std::abs(diff);

            if (gap_size > gap_threshold_) {
                GapInfo gap;
                gap.gap_size = gap_size;
                gap.gap_diff = diff;
                // Gap index points to the closer point
                gap.gap_index = (diff > 0) ? i : i + 1;

                // Calculate goal index for this gap
                float distance_to_gap = ranges[gap.gap_index];
                if (distance_to_gap < 0.1) distance_to_gap = 0.1;

                float offset_angle = std::atan((car_width_ / 2.0 + disparity_margin_) / distance_to_gap);
                float gap_angle = scan_msg->angle_min + gap.gap_index * scan_msg->angle_increment;

                float goal_angle;
                if (diff > 0) {
                    goal_angle = gap_angle + offset_angle;
                } else {
                    goal_angle = gap_angle - offset_angle;
                }

                gap.goal_index = std::round((goal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                gap.goal_index = std::clamp(gap.goal_index, 0, n - 1);
                gap.goal_distance = ranges[gap.goal_index];

                gaps.push_back(gap);
            }
        }

        // Sort gaps by size (largest first)
        std::sort(gaps.begin(), gaps.end(),
            [](const GapInfo& a, const GapInfo& b) { return a.gap_size > b.gap_size; });

        return gaps;
    }

    Waypoint detect_disparity_and_select_waypoint(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
        const std::vector<float>& ranges) {

        Waypoint waypoint;
        waypoint.x = 0.0;
        waypoint.y = 0.0;
        waypoint.distance = 0.0;
        waypoint.scan_index = -1;

        int n = ranges.size();

        // Find all significant gaps
        std::vector<GapInfo> gaps = find_all_gaps(scan_msg, ranges);

        int goal_index = -1;
        bool used_alternative_gap = false;

        if (!gaps.empty()) {
            // Try to select a gap that doesn't cause a waypoint jump
            GapInfo selected_gap = gaps[0]; // Default to largest gap

            if (enable_waypoint_stability_ && waypoint_initialized_) {
                // Check if the primary gap causes a jump
                float primary_x = selected_gap.goal_distance * std::cos(scan_msg->angle_min + selected_gap.goal_index * scan_msg->angle_increment);
                float primary_y = selected_gap.goal_distance * std::sin(scan_msg->angle_min + selected_gap.goal_index * scan_msg->angle_increment);

                float jump_distance = std::sqrt(
                    std::pow(primary_x - prev_waypoint_x_, 2) +
                    std::pow(primary_y - prev_waypoint_y_, 2)
                );

                if (jump_distance > waypoint_jump_threshold_) {
                    // Try to find an alternative gap that doesn't jump as much
                    RCLCPP_WARN(this->get_logger(),
                        "Primary gap causes jump of %.2f m (threshold: %.2f m), searching for alternative...",
                        jump_distance, waypoint_jump_threshold_);

                    for (size_t i = 1; i < gaps.size(); i++) {
                        float alt_x = gaps[i].goal_distance * std::cos(scan_msg->angle_min + gaps[i].goal_index * scan_msg->angle_increment);
                        float alt_y = gaps[i].goal_distance * std::sin(scan_msg->angle_min + gaps[i].goal_index * scan_msg->angle_increment);

                        float alt_jump = std::sqrt(
                            std::pow(alt_x - prev_waypoint_x_, 2) +
                            std::pow(alt_y - prev_waypoint_y_, 2)
                        );

                        if (alt_jump < waypoint_jump_threshold_) {
                            selected_gap = gaps[i];
                            used_alternative_gap = true;
                            RCLCPP_INFO(this->get_logger(),
                                "Using alternative gap #%zu (size: %.2f m, jump: %.2f m)",
                                i, selected_gap.gap_size, alt_jump);
                            break;
                        }
                    }

                    // If no alternative found, still use the primary but log it
                    if (!used_alternative_gap) {
                        RCLCPP_WARN(this->get_logger(),
                            "No stable alternative gap found, using primary gap anyway");
                    }
                }
            }

            goal_index = selected_gap.goal_index;

            if (verbose_logging_) {
                RCLCPP_INFO(this->get_logger(),
                    "Disparity detected: gap=%.2f m at index %d, goal_index=%d%s",
                    selected_gap.gap_size, selected_gap.gap_index, goal_index,
                    used_alternative_gap ? " (alternative)" : "");
            }
        } else {
            // No significant disparity, go towards furthest point
            float max_distance = 0.0;
            std::vector<int> max_indices;

            for (int i = 0; i < n; i++) {
                if (ranges[i] > max_distance) {
                    max_distance = ranges[i];
                    max_indices.clear();
                    max_indices.push_back(i);
                } else if (ranges[i] == max_distance && max_distance > 0.0) {
                    max_indices.push_back(i);
                }
            }

            if (!max_indices.empty()) {
                // Take center of furthest points
                goal_index = max_indices[max_indices.size() / 2];
            }

            if (verbose_logging_) {
                RCLCPP_INFO(this->get_logger(),
                    "No disparity, going to furthest point at index %d (distance=%.2f m)",
                    goal_index, max_distance);
            }
        }

        // Convert goal index to waypoint coordinates
        if (goal_index >= 0 && goal_index < n) {
            float goal_angle = scan_msg->angle_min + goal_index * scan_msg->angle_increment;
            float goal_distance = ranges[goal_index];

            if (goal_distance > 0.0) {
                waypoint.x = goal_distance * std::cos(goal_angle);
                waypoint.y = goal_distance * std::sin(goal_angle);
                waypoint.distance = goal_distance;
                waypoint.scan_index = goal_index;

                // Update previous waypoint for jump detection
                prev_waypoint_x_ = waypoint.x;
                prev_waypoint_y_ = waypoint.y;
                waypoint_initialized_ = true;
            }
        }

        return waypoint;
    }

    float compute_steering_angle(const Waypoint& waypoint) {
        // Reactive Pure Pursuit controller
        // Clamp lookahead distance
        float lookahead = std::clamp(waypoint.distance,
            static_cast<float>(lookahead_min_),
            static_cast<float>(lookahead_max_));

        // Pure pursuit formula: curvature = 2*y/L^2
        float curvature = (2.0 * waypoint.y) / (lookahead * lookahead);

        // Steering angle = atan(wheelbase * curvature) * gain
        float steering = std::atan(wheel_base_ * curvature) * steering_gain_;

        // Clamp to max steering
        steering = std::clamp(steering,
            static_cast<float>(-max_steer_),
            static_cast<float>(max_steer_));

        // Apply exponential smoothing
        float smoothed_steering = steering_smoothing_ * steering +
                                  (1.0 - steering_smoothing_) * prev_steering_;
        prev_steering_ = smoothed_steering;

        return smoothed_steering;
    }

    float compute_velocity(float steering_angle) {
        // Reduce velocity based on steering angle
        float steering_ratio = std::abs(steering_angle) / max_steer_;
        float velocity = v_max_ - steering_ratio * (v_max_ - v_min_) * speed_reduction_factor_;

        return std::clamp(velocity,
            static_cast<float>(v_min_),
            static_cast<float>(v_max_));
    }

    void publish_drive_command(float steering_angle, float velocity) {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;

        drive_pub_->publish(drive_msg);
        opp_drive_pub_->publish(drive_msg);  // Also publish to opp for 2-agent scenario
    }

    void visualize_waypoint(const Waypoint& waypoint) {
        if (!marker_array_pub_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Marker array publisher is not initialized!");
            return;
        }

        if (!enable_visualization_) {
            return;
        }

        if (!odom_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Odometry not received yet, skipping visualization");
            return;
        }

        // Transform waypoint from laser frame to map frame using odometry
        // waypoint is in laser frame (car local), convert to map frame
        double cos_yaw = std::cos(current_yaw_);
        double sin_yaw = std::sin(current_yaw_);

        double waypoint_map_x = current_x_ + waypoint.x * cos_yaw - waypoint.y * sin_yaw;
        double waypoint_map_y = current_y_ + waypoint.x * sin_yaw + waypoint.y * cos_yaw;

        visualization_msgs::msg::MarkerArray marker_array;
        auto timestamp = this->now();

        // 1. Waypoint marker (큰 초록색 구체)
        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = marker_frame_;
        waypoint_marker.header.stamp = timestamp;
        waypoint_marker.ns = "waypoint";
        waypoint_marker.id = 0;
        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoint_marker.pose.position.x = waypoint_map_x;
        waypoint_marker.pose.position.y = waypoint_map_y;
        waypoint_marker.pose.position.z = 0.5;  // 높이를 올려서 더 잘 보이게
        waypoint_marker.pose.orientation.w = 1.0;
        waypoint_marker.scale.x = 0.5;  // 더 크게
        waypoint_marker.scale.y = 0.5;
        waypoint_marker.scale.z = 0.5;
        waypoint_marker.color.r = 0.0;
        waypoint_marker.color.g = 1.0;
        waypoint_marker.color.b = 0.0;
        waypoint_marker.color.a = 1.0;
        waypoint_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(waypoint_marker);

        // 2. 연결선 (두꺼운 노란색)
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = marker_frame_;
        line_marker.header.stamp = timestamp;
        line_marker.ns = "connection_line";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.1;  // 선 두께 증가
        line_marker.color.r = 1.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        line_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = current_x_;
        p_start.y = current_y_;
        p_start.z = 0.0;
        p_end.x = waypoint_map_x;
        p_end.y = waypoint_map_y;
        p_end.z = 0.5;  // 높이 맞춤
        line_marker.points.push_back(p_start);
        line_marker.points.push_back(p_end);
        marker_array.markers.push_back(line_marker);

        // 3. 텍스트 마커 (거리 정보)
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = marker_frame_;
        text_marker.header.stamp = timestamp;
        text_marker.ns = "waypoint_info";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = waypoint_map_x;
        text_marker.pose.position.y = waypoint_map_y;
        text_marker.pose.position.z = 1.0;  // 구체 위에 표시
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.3;  // 텍스트 크기
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "D: " + std::to_string(waypoint.distance).substr(0, 4) + "m";
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(text_marker);

        // 4. 화살표 마커 (방향 표시)
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = marker_frame_;
        arrow_marker.header.stamp = timestamp;
        arrow_marker.ns = "direction_arrow";
        arrow_marker.id = 3;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point arrow_start, arrow_end;
        arrow_start.x = current_x_;
        arrow_start.y = current_y_;
        arrow_start.z = 0.2;
        arrow_end.x = waypoint_map_x;
        arrow_end.y = waypoint_map_y;
        arrow_end.z = 0.2;
        arrow_marker.points.push_back(arrow_start);
        arrow_marker.points.push_back(arrow_end);

        arrow_marker.scale.x = 0.1;  // 화살표 샤프트 두께
        arrow_marker.scale.y = 0.2;  // 화살표 머리 두께
        arrow_marker.scale.z = 0.3;  // 화살표 머리 길이
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 0.8;
        arrow_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(arrow_marker);

        marker_array_pub_->publish(marker_array);

        if (verbose_logging_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Published %zu markers to /disparity/visualization (frame: %s)",
                marker_array.markers.size(), marker_frame_.c_str());
        }
    }

    // ROS Communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr opp_drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    // Parameters - Topics
    std::string scan_topic_;
    std::string drive_topic_;

    // Parameters - Vehicle
    double wheel_base_;
    double car_width_;

    // Parameters - Disparity
    double max_range_;
    double gap_threshold_;
    double disparity_margin_;

    // Parameters - Control
    double v_max_;
    double v_min_;
    double max_steer_;

    // Parameters - Waypoint selection
    double lookahead_min_;
    double lookahead_max_;

    // Parameters - Control tuning
    double steering_gain_;
    double steering_smoothing_;
    double speed_reduction_factor_;

    // Parameters - Visualization
    bool enable_visualization_;
    std::string marker_frame_;
    double waypoint_marker_size_;

    // Parameters - Waypoint stability
    bool enable_waypoint_stability_;
    double waypoint_jump_threshold_;

    // Parameters - Debug
    bool verbose_logging_;

    // State
    float prev_steering_;
    double current_x_;
    double current_y_;
    double current_yaw_;
    bool odom_received_;
    double prev_waypoint_x_;
    double prev_waypoint_y_;
    bool waypoint_initialized_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisparityReactiveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
