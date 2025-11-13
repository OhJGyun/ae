#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA, Float64, Int32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rviz_2d_overlay_msgs.msg import OverlayText
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import os
import csv
import math
from datetime import datetime


class EvaluationNode(Node):
    def __init__(self):
        super().__init__('evaluation_node')

        # Declare and get parameters
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('log_directory', '/home/joon/joon_sim_ws/lap_logs')
        self.declare_parameter('display_lap_times', True)
        self.declare_parameter('controller_name', 'unknown')
        self.declare_parameter('csv_output_directory', '/home/joon/joon_sim_ws/evaluation_results')
        self.declare_parameter('is_obstacle_map', False)  # ✅ 장애물 맵 여부 파라미터

        odom_topic = self.get_parameter('odom_topic').value
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_directory = self.get_parameter('log_directory').value
        self.display_lap_times = self.get_parameter('display_lap_times').value
        self.controller_name = self.get_parameter('controller_name').value
        self.csv_output_directory = self.get_parameter('csv_output_directory').value
        self.is_obstacle_map = self.get_parameter('is_obstacle_map').value

        # ✅ Create controller-specific CSV file path (장애물 맵이면 별도 파일)
        if self.is_obstacle_map:
            self.csv_output_path = os.path.join(
                self.csv_output_directory,
                f"{self.controller_name}_obs_laps.csv"
            )
        else:
            self.csv_output_path = os.path.join(
                self.csv_output_directory,
                f"{self.controller_name}_laps.csv"
            )

        # Lap counter
        self.lap_counter = 0

        # Collision tracking
        self.collision_count = 0
        self.collision_penalty_per_hit = -10.0  # Penalty points per collision

        # ✅ Odometry-based reset detection
        self.last_odom_x = None
        self.last_odom_y = None
        self.odom_stuck_threshold = 0.01  # 오돔이 이 거리 이하로 움직이면 stuck
        self.odom_stuck_count = 0
        self.odom_stuck_limit = 50  # 50회 연속 stuck이면 충돌로 판단 (~2.5초)

        # ✅ Lane selector connection
        self.current_selected_lane = None

        # Try to get sections parameter, use empty list if not found
        try:
            self.declare_parameter('sections', [])
            self.sections = self.get_parameter('sections').value
        except:
            self.sections = []

        self.get_logger().info(f'Loaded {len(self.sections) if self.sections else 0} sectors from config')

        # Create log directory if it doesn't exist
        if self.log_to_file:
            os.makedirs(self.log_directory, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file = os.path.join(self.log_directory, f"lap_times_{timestamp}.txt")
            with open(self.log_file, 'w') as f:
                f.write("Section-based Lap Timer Log\n")
                f.write(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("="*80 + "\n\n")

        # Parse sectors
        self.section_list = []
        if not self.sections:
            self.get_logger().error('❌ NO SECTORS CONFIGURED! Check your YAML file and launch parameters!')
            # Add default sectors if none configured
            self.get_logger().warn('Using hardcoded sectors as fallback')
            self.sections = [
                {'name': 'Sector 1', 'start_line': [20.0, 5.0, 23.6, 5.0], 'end_line': [1.0, 21.8, 1.0, 25.5]},
                {'name': 'Sector 2', 'start_line': [1.0, 21.8, 1.0, 25.5], 'end_line': [11.6, 21.2, 11.6, 17.6]},
                {'name': 'Sector 3', 'start_line': [11.6, 21.2, 11.6, 17.6], 'end_line': [-5.46, 7.8, -1.76, 7.8]},
                {'name': 'Sector 4', 'start_line': [-5.46, 7.8, -1.76, 7.8], 'end_line': [20.0, 5.0, 23.6, 5.0]}
            ]

        # Initialize tracking variables
        self.section_times = {}
        self.crossed_start = [False] * len(self.sections)

        for i, section in enumerate(self.sections):
            section_data = {
                'name': section['name'],
                'start_line': section['start_line'],
                'end_line': section['end_line'],
                'index': i
            }
            self.section_list.append(section_data)
            self.section_times[section['name']] = []
            self.get_logger().info(f"✓ Loaded {section['name']}")
            self.get_logger().info(f"  Start line: ({section['start_line'][0]}, {section['start_line'][1]}) → ({section['start_line'][2]}, {section['start_line'][3]})")
            self.get_logger().info(f"  End line: ({section['end_line'][0]}, {section['end_line'][1]}) → ({section['end_line'][2]}, {section['end_line'][3]})")

        # Initialize section tracking AFTER sections are loaded
        self.current_section = None
        self.section_start_time = None
        self.previous_position = None

        # Lap timing variables
        self.lap_start_time = None
        self.lap_in_progress = False
        self.current_lap_data = {}  # Store times and speeds for each sector

        # Speed tracking variables
        self.current_speed = 0.0
        self.commanded_speed = 0.0
        self.target_speed = 0.0
        self.speed_error = 0.0

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Subscribe to drive commands to get commanded speed
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        # Subscribe to target speed
        self.target_speed_sub = self.create_subscription(
            Float64,
            '/target_speed',
            self.target_speed_callback,
            10
        )

        # Subscribe to collision count
        self.collision_sub = self.create_subscription(
            Int32,
            '/collision_count',
            self.collision_callback,
            10
        )

        # ✅ Subscribe to lane selector (현재 선택된 레인)
        self.lane_selector_sub = self.create_subscription(
            Int32,
            '/lane_selector/target_lane',
            self.lane_selector_callback,
            10
        )

        # Publisher for lap times
        self.lap_time_pub = self.create_publisher(String, '/lap_times', 10)

        # Publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/section_markers', 10)
        self.overlay_text_pub = self.create_publisher(OverlayText, '/lap_time_overlay', 10)

        # Timer for publishing visualization at 10Hz
        self.viz_timer = self.create_timer(0.1, self.publish_visualizations)

        # Timer for publishing section markers at 2Hz
        self.marker_timer = self.create_timer(0.5, self.publish_section_markers)

        # Create CSV output directory if needed
        os.makedirs(self.csv_output_directory, exist_ok=True)
        self.get_logger().info(f'✓ Ensured CSV directory exists: {self.csv_output_directory}')

        # Initialize CSV file (each run creates a new file or appends)
        self.initialize_csv()

        self.get_logger().info('='*50)
        self.get_logger().info('🏁 EVALUATION NODE STARTED')
        self.get_logger().info(f'🎮 Controller: {self.controller_name}')
        self.get_logger().info(f'📍 Monitoring {len(self.section_list)} sectors')
        self.get_logger().info(f'📊 CSV output: {self.csv_output_path}')
        self.get_logger().info('='*50)

    def point_to_line_distance(self, point, line_start, line_end):
        """Calculate perpendicular distance from point to line segment"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end

        # Vector from line_start to line_end
        dx = x2 - x1
        dy = y2 - y1

        # If line is a point
        if dx == 0 and dy == 0:
            return np.sqrt((x0 - x1)**2 + (y0 - y1)**2)

        # Calculate perpendicular distance
        t = max(0, min(1, ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)))
        projection_x = x1 + t * dx
        projection_y = y1 + t * dy

        distance = np.sqrt((x0 - projection_x)**2 + (y0 - projection_y)**2)
        return distance

    def check_line_crossing(self, prev_pos, curr_pos, line):
        """Check if the path from prev_pos to curr_pos crosses the line"""
        if prev_pos is None:
            return False

        x1, y1 = prev_pos
        x2, y2 = curr_pos
        x3, y3 = line[0], line[1]
        x4, y4 = line[2], line[3]

        # Calculate denominator
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if abs(denom) < 1e-10:  # Lines are parallel
            return False

        # Calculate intersection parameters
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

        # Check if intersection is within both line segments
        if 0 <= t <= 1 and 0 <= u <= 1:
            return True

        return False

    def start_section(self, section_index, start_time=None, auto=False):
        """Activate a section timer. Can be triggered manually or chained from a previous section."""
        if not (0 <= section_index < len(self.section_list)):
            return

        section = self.section_list[section_index]
        self.current_section = section_index
        self.section_start_time = start_time if start_time is not None else time.time()

        # Start lap timing if this is the first sector
        if section_index == 0 and not self.lap_in_progress:
            self.lap_in_progress = True
            self.lap_start_time = self.section_start_time
            self.current_lap_data = {}
            self.get_logger().info('🏁 LAP STARTED')

        log_prefix = "Auto-entered" if auto else "Entered"
        self.get_logger().info(f"{log_prefix} {section['name']}")

    def complete_section(self, section_index, completion_time):
        """Finalize the current section and immediately arm the next one."""
        if self.section_start_time is None or not (0 <= section_index < len(self.section_list)):
            return

        section = self.section_list[section_index]
        section_time = completion_time - self.section_start_time
        self.section_times[section['name']].append(section_time)

        # Store data for current lap
        section_name = section['name']
        self.current_lap_data[f'{section_name}_time'] = section_time
        self.current_lap_data[f'{section_name}_avg_speed'] = self.current_speed

        log_msg = f"{section['name']}: {section_time:.4f}s (avg speed: {self.current_speed:.2f} m/s)"
        self.get_logger().info(f"Completed {log_msg}")

        times = self.section_times[section['name']]
        if len(times) > 1:
            avg_time = np.mean(times)
            best_time = np.min(times)
            self.get_logger().info(
                f"  Stats - Best: {best_time:.4f}s, Avg: {avg_time:.4f}s, Laps: {len(times)}"
            )
        else:
            avg_time = best_time = section_time

        # Check if lap is complete (last sector finished)
        if section_index == len(self.section_list) - 1 and self.lap_in_progress:
            total_lap_time = completion_time - self.lap_start_time
            self.get_logger().info(f'🏁 LAP COMPLETE - Total Time: {total_lap_time:.3f}s')
            self.save_lap_to_csv()
            self.lap_in_progress = False

        msg_out = String()
        msg_out.data = log_msg
        self.lap_time_pub.publish(msg_out)

        if self.log_to_file:
            with open(self.log_file, 'a') as f:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                f.write(f"[{timestamp}] {log_msg}\n")
                if len(times) > 1:
                    f.write(f"  Best: {best_time:.4f}s, Avg: {avg_time:.4f}s, Laps: {len(times)}\n")

        # Prepare for the next section
        self.current_section = None
        self.section_start_time = None
        next_index = (section_index + 1) % len(self.section_list)
        # Chain into the next section using the same crossing timestamp
        self.start_section(next_index, start_time=completion_time, auto=True)

    def target_speed_callback(self, msg):
        """Callback for target speed"""
        self.target_speed = msg.data

    def drive_callback(self, msg):
        """Callback for drive commands - captures commanded speed"""
        self.commanded_speed = msg.drive.speed
        self.speed_error = self.commanded_speed - self.current_speed

    def collision_callback(self, msg):
        """Callback for collision count from gym_bridge"""
        self.collision_count = msg.data
        self.get_logger().warn(f'💥 Collision count updated: {self.collision_count}')

    def lane_selector_callback(self, msg):
        """Callback for lane selector - track current selected lane"""
        self.current_selected_lane = msg.data
        self.get_logger().info(f'🛣️ Lane selected: {self.current_selected_lane}', throttle_duration_sec=2.0)

    def odom_callback(self, msg):
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        current_time = time.time()

        # Update current speed from odometry
        self.current_speed = msg.twist.twist.linear.x
        self.speed_error = self.commanded_speed - self.current_speed

        # ✅ Odometry-based stuck/collision detection
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        if self.last_odom_x is not None and self.last_odom_y is not None:
            odom_move = math.sqrt(
                (curr_x - self.last_odom_x)**2 + (curr_y - self.last_odom_y)**2
            )

            # 오돔이 거의 움직이지 않고, commanded_speed가 0이 아니면 stuck 의심
            if odom_move < self.odom_stuck_threshold and abs(self.commanded_speed) > 0.1:
                self.odom_stuck_count += 1
                if self.odom_stuck_count >= self.odom_stuck_limit:
                    self.collision_count += 1
                    self.get_logger().warn(
                        f'⚠️ Odometry stuck detected! Movement: {odom_move:.4f}m, '
                        f'Stuck count: {self.odom_stuck_count}, Total collisions: {self.collision_count}'
                    )
                    self.odom_stuck_count = 0  # Reset counter
            else:
                # 정상 이동하면 카운터 리셋
                if self.odom_stuck_count > 0:
                    self.odom_stuck_count = max(0, self.odom_stuck_count - 5)

        self.last_odom_x = curr_x
        self.last_odom_y = curr_y

        if self.current_section is not None:
            active_section = self.section_list[self.current_section]
            if self.check_line_crossing(self.previous_position, current_pos, active_section['end_line']):
                self.complete_section(self.current_section, current_time)
        else:
            for i, section in enumerate(self.section_list):
                if self.check_line_crossing(self.previous_position, current_pos, section['start_line']):
                    self.start_section(i, start_time=current_time)
                    break

        # Update previous position
        self.previous_position = current_pos

    def publish_section_markers(self):
        """Publish visualization markers for sector lines"""
        marker_array = MarkerArray()

        # Green color for lines
        line_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # GREEN

        marker_id = 0

        for i, section in enumerate(self.section_list):
            # Start line marker
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = "sector_lines"
            start_marker.id = marker_id
            marker_id += 1
            start_marker.type = Marker.LINE_STRIP
            start_marker.action = Marker.ADD
            start_marker.scale.x = 0.3  # Line width
            start_marker.color = line_color
            start_marker.pose.orientation.w = 1.0
            start_marker.lifetime.sec = 0  # Persistent
            start_marker.lifetime.nanosec = 0

            # Add start line points
            p1 = Point()
            p1.x = float(section['start_line'][0])
            p1.y = float(section['start_line'][1])
            p1.z = 0.0
            p2 = Point()
            p2.x = float(section['start_line'][2])
            p2.y = float(section['start_line'][3])
            p2.z = 0.0
            start_marker.points = [p1, p2]
            marker_array.markers.append(start_marker)

            # Start line label (text only for START)
            start_text = Marker()
            start_text.header.frame_id = "map"
            start_text.header.stamp = self.get_clock().now().to_msg()
            start_text.ns = "sector_labels"
            start_text.id = marker_id
            marker_id += 1
            start_text.type = Marker.TEXT_VIEW_FACING
            start_text.action = Marker.ADD
            start_text.pose.position.x = float((p1.x + p2.x) / 2)
            start_text.pose.position.y = float((p1.y + p2.y) / 2)
            start_text.pose.position.z = 1.0
            start_text.pose.orientation.w = 1.0
            start_text.scale.z = 0.7  # Text size
            start_text.color = line_color
            start_text.text = f"{section['name']} START"
            start_text.lifetime.sec = 0  # Persistent
            start_text.lifetime.nanosec = 0
            marker_array.markers.append(start_text)

            # End line marker (no text)
            end_marker = Marker()
            end_marker.header.frame_id = "map"
            end_marker.header.stamp = self.get_clock().now().to_msg()
            end_marker.ns = "sector_lines"
            end_marker.id = marker_id
            marker_id += 1
            end_marker.type = Marker.LINE_STRIP
            end_marker.action = Marker.ADD
            end_marker.scale.x = 0.3  # Line width
            end_marker.color = line_color
            end_marker.pose.orientation.w = 1.0
            end_marker.lifetime.sec = 0  # Persistent
            end_marker.lifetime.nanosec = 0

            # Add end line points
            p3 = Point()
            p3.x = float(section['end_line'][0])
            p3.y = float(section['end_line'][1])
            p3.z = 0.0
            p4 = Point()
            p4.x = float(section['end_line'][2])
            p4.y = float(section['end_line'][3])
            p4.z = 0.0
            end_marker.points = [p3, p4]
            marker_array.markers.append(end_marker)

        self.marker_pub.publish(marker_array)

    def publish_visualizations(self):
        """Publish overlay text with lap times info"""
        overlay = OverlayText()

        # Action: ADD to display the overlay
        overlay.action = OverlayText.ADD

        # Size and position - increased to show all 4 sectors
        overlay.width = 500
        overlay.height = 1000
        overlay.horizontal_distance = 10  # 10 pixels from left
        overlay.vertical_distance = 10     # 10 pixels from top

        # Alignment: top-left corner
        overlay.horizontal_alignment = OverlayText.LEFT
        overlay.vertical_alignment = OverlayText.TOP

        # Colors
        overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.7)  # Semi-transparent black
        overlay.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White text

        # Text properties
        overlay.line_width = 2
        overlay.text_size = 14.0
        overlay.font = "DejaVu Sans Mono"

        # Build text content
        text_lines = ["=== LAP TIMER ===\n"]

        # Add speed tracking info at the top
        text_lines.append("--- SPEED TRACKING ---")
        text_lines.append(f"Target:    {self.target_speed:.2f} m/s")
        text_lines.append(f"Commanded: {self.commanded_speed:.2f} m/s")
        text_lines.append(f"Current:   {self.current_speed:.2f} m/s")

        # Tracking error (target vs current - overall system performance)
        tracking_error = self.target_speed - self.current_speed
        text_lines.append(f"Track Err: {tracking_error:+.3f} m/s")

        # PID error (commanded vs current - vehicle response)
        pid_error = self.commanded_speed - self.current_speed
        text_lines.append(f"PID Err:   {pid_error:+.3f} m/s")
        text_lines.append("")

        for section in self.section_list:
            section_name = section['name']
            times = self.section_times[section_name]

            text_lines.append(f"\n{section_name}:")

            if times:
                last_time = times[-1]
                best_time = np.min(times)
                avg_time = np.mean(times)

                text_lines.append(f"  Last: {last_time:.3f}s")
                text_lines.append(f"  Best: {best_time:.3f}s")
                text_lines.append(f"  Avg:  {avg_time:.3f}s")
                text_lines.append(f"  Laps: {len(times)}")
            else:
                text_lines.append("  No data yet")

        # Add current section info
        if self.current_section is not None:
            current_time = time.time() - self.section_start_time if self.section_start_time else 0
            text_lines.append(f"\n--- CURRENT ---")
            text_lines.append(f"{self.section_list[self.current_section]['name']}")
            text_lines.append(f"Time: {current_time:.3f}s")

        overlay.text = "\n".join(text_lines)
        self.overlay_text_pub.publish(overlay)

    def destroy_node(self):
        # Write final summary
        if self.log_to_file:
            with open(self.log_file, 'a') as f:
                f.write("\n" + "="*80 + "\n")
                f.write("Session Summary\n")
                f.write("="*80 + "\n")
                for section_name, times in self.section_times.items():
                    if times:
                        f.write(f"\n{section_name}:\n")
                        f.write(f"  Total laps: {len(times)}\n")
                        f.write(f"  Best time: {np.min(times):.4f}s\n")
                        f.write(f"  Worst time: {np.max(times):.4f}s\n")
                        f.write(f"  Average time: {np.mean(times):.4f}s\n")
                        f.write(f"  Std deviation: {np.std(times):.4f}s\n")

        super().destroy_node()


    def initialize_csv(self):
        """Initialize CSV file with headers or check existing lap count"""
        file_exists = os.path.exists(self.csv_output_path)

        if not file_exists:
            with open(self.csv_output_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Create header
                header = ['lap_number', 'timestamp', 'total_lap_time']
                for section in self.section_list:
                    header.append(f"{section['name']}_time")
                    header.append(f"{section['name']}_avg_speed")
                # ✅ Add lane selector info and map type
                header.extend(['collision_count', 'overall_score', 'is_obstacle_map', 'selected_lane'])
                writer.writerow(header)
            self.get_logger().info(f'✓ Created new CSV file: {self.csv_output_path}')
            self.lap_counter = 0
        else:
            # Count existing laps
            with open(self.csv_output_path, 'r', newline='') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)  # Skip header
                self.lap_counter = sum(1 for _ in reader)
            self.get_logger().info(f'✓ CSV file exists with {self.lap_counter} laps: {self.csv_output_path}')

    def save_lap_to_csv(self):
        """Append completed lap data as a new row to CSV"""
        if not self.current_lap_data:
            return

        # Increment lap counter
        self.lap_counter += 1

        # Calculate total lap time
        total_lap_time = sum(self.current_lap_data.get(f'{section["name"]}_time', 0.0)
                            for section in self.section_list)

        # Calculate score (lower lap time = higher score)
        # Base score: 1000 / lap_time (so 40s lap = 25.0 score, 45s lap = 22.2 score)
        # Collision penalty: -10 points per collision
        base_score = 1000.0 / total_lap_time if total_lap_time > 0 else 0.0
        collision_penalty = self.collision_count * self.collision_penalty_per_hit
        overall_score = base_score + collision_penalty

        # Append new lap row to CSV
        with open(self.csv_output_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Build row data
            row = [
                self.lap_counter,
                datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                f'{total_lap_time:.3f}'
            ]

            # Add section times and speeds
            for section in self.section_list:
                section_name = section['name']
                row.append(f'{self.current_lap_data.get(f"{section_name}_time", 0.0):.3f}')
                row.append(f'{self.current_lap_data.get(f"{section_name}_avg_speed", 0.0):.2f}')

            # Add collision count and overall score
            row.append(f'{self.collision_count}')
            row.append(f'{overall_score:.2f}')
            # ✅ Add map type and lane selector info
            row.append('True' if self.is_obstacle_map else 'False')
            row.append(f'{self.current_selected_lane if self.current_selected_lane is not None else "N/A"}')
            writer.writerow(row)

        self.get_logger().info(
            f'📊 Saved Lap {self.lap_counter} to CSV: Total: {total_lap_time:.3f}s, '
            f'Collisions: {self.collision_count}, Score: {overall_score:.2f}, '
            f'Map: {"OBS" if self.is_obstacle_map else "NORMAL"}, Lane: {self.current_selected_lane}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
