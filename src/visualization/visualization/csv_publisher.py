#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSV Visualization Node
Reads CSV files with x,y coordinates and visualizes them in RViz
"""

import os
import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class CSVPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')

        # Parameters
        self.declare_parameter('map_bounds_csv', '/home/ojg/ae/csv/map_bounds_rescaled.csv')
        self.declare_parameter('marker_frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('line_width', 0.05)
        self.declare_parameter('point_size', 0.1)
        self.declare_parameter('z_height', 0.0)

        # Get parameters
        self.map_bounds_csv = self.get_parameter('map_bounds_csv').get_parameter_value().string_value
        self.frame_id = self.get_parameter('marker_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self.point_size = self.get_parameter('point_size').get_parameter_value().double_value
        self.z_height = self.get_parameter('z_height').get_parameter_value().double_value

        # Load CSV data
        self.centerline_points = []
        self.left_bound_points = []
        self.right_bound_points = []
        self._load_map_bounds_csv()

        # Publisher
        self.pub = self.create_publisher(MarkerArray, 'csv_markers', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_markers)

        self.get_logger().info(f'CSV Publisher initialized')
        self.get_logger().info(f'  Centerline points: {len(self.centerline_points)}')
        self.get_logger().info(f'  Left boundary points: {len(self.left_bound_points)}')
        self.get_logger().info(f'  Right boundary points: {len(self.right_bound_points)}')
        self.get_logger().info(f'  Frame: {self.frame_id}')

    def _load_map_bounds_csv(self):
        """Load map bounds CSV with centerline and left/right boundaries"""
        csv_path = self.map_bounds_csv

        if not os.path.exists(csv_path):
            self.get_logger().warn(f'CSV file not found: {csv_path}')
            return

        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)

                # Read header
                header = next(reader, None)
                if header:
                    self.get_logger().info(f'CSV header: {header}')

                # Read data
                for row in reader:
                    if len(row) >= 6:
                        try:
                            # centerline_x, centerline_y, left_bound_x, left_bound_y, right_bound_x, right_bound_y
                            centerline_x = float(row[0])
                            centerline_y = float(row[1])
                            left_bound_x = float(row[2])
                            left_bound_y = float(row[3])
                            right_bound_x = float(row[4])
                            right_bound_y = float(row[5])

                            self.centerline_points.append((centerline_x, centerline_y))
                            self.left_bound_points.append((left_bound_x, left_bound_y))
                            self.right_bound_points.append((right_bound_x, right_bound_y))
                        except ValueError as e:
                            self.get_logger().warn(f'Invalid row: {row}, error: {e}')
                            continue

            self.get_logger().info(f'Loaded {len(self.centerline_points)} points from {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading CSV {csv_path}: {e}')

    def _create_line_marker(
        self,
        marker_id: int,
        namespace: str,
        points: List[Tuple[float, float]],
        color: ColorRGBA
    ) -> Marker:
        """Create a LINE_STRIP marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale.x = self.line_width

        # Color
        marker.color = color

        # Lifetime (0 = forever)
        marker.lifetime = Duration(seconds=0).to_msg()

        # Points
        for x, y in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.z_height
            marker.points.append(p)

        return marker

    def _create_points_marker(
        self,
        marker_id: int,
        namespace: str,
        points: List[Tuple[float, float]],
        color: ColorRGBA
    ) -> Marker:
        """Create a SPHERE_LIST marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale.x = self.point_size
        marker.scale.y = self.point_size
        marker.scale.z = self.point_size

        # Color
        marker.color = color

        # Lifetime (0 = forever)
        marker.lifetime = Duration(seconds=0).to_msg()

        # Points
        for x, y in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.z_height
            marker.points.append(p)

        return marker

    def _publish_markers(self):
        """Publish all markers"""
        marker_array = MarkerArray()

        # Centerline (Yellow)
        if self.centerline_points:
            centerline_color = ColorRGBA()
            centerline_color.r = 1.0
            centerline_color.g = 1.0
            centerline_color.b = 0.0
            centerline_color.a = 1.0

            # Line
            centerline_line = self._create_line_marker(
                marker_id=0,
                namespace='centerline_line',
                points=self.centerline_points,
                color=centerline_color
            )
            marker_array.markers.append(centerline_line)

            # Points
            centerline_points_marker = self._create_points_marker(
                marker_id=1,
                namespace='centerline_points',
                points=self.centerline_points,
                color=centerline_color
            )
            marker_array.markers.append(centerline_points_marker)

        # Left Boundary (Red)
        if self.left_bound_points:
            left_color = ColorRGBA()
            left_color.r = 1.0
            left_color.g = 0.0
            left_color.b = 0.0
            left_color.a = 0.8

            # Line
            left_line = self._create_line_marker(
                marker_id=2,
                namespace='left_boundary_line',
                points=self.left_bound_points,
                color=left_color
            )
            marker_array.markers.append(left_line)

            # Points
            left_points = self._create_points_marker(
                marker_id=3,
                namespace='left_boundary_points',
                points=self.left_bound_points,
                color=left_color
            )
            marker_array.markers.append(left_points)

        # Right Boundary (Blue)
        if self.right_bound_points:
            right_color = ColorRGBA()
            right_color.r = 0.0
            right_color.g = 0.0
            right_color.b = 1.0
            right_color.a = 0.8

            # Line
            right_line = self._create_line_marker(
                marker_id=4,
                namespace='right_boundary_line',
                points=self.right_bound_points,
                color=right_color
            )
            marker_array.markers.append(right_line)

            # Points
            right_points = self._create_points_marker(
                marker_id=5,
                namespace='right_boundary_points',
                points=self.right_bound_points,
                color=right_color
            )
            marker_array.markers.append(right_points)

        # Cross lines connecting left and right boundaries (White with transparency)
        if self.left_bound_points and self.right_bound_points:
            if len(self.left_bound_points) == len(self.right_bound_points):
                cross_color = ColorRGBA()
                cross_color.r = 1.0
                cross_color.g = 1.0
                cross_color.b = 1.0
                cross_color.a = 0.3

                # Create individual line markers for each cross line
                for i in range(len(self.left_bound_points)):
                    cross_line = Marker()
                    cross_line.header.frame_id = self.frame_id
                    cross_line.header.stamp = self.get_clock().now().to_msg()
                    cross_line.ns = 'cross_lines'
                    cross_line.id = 1000 + i  # Start from 1000 to avoid conflicts
                    cross_line.type = Marker.LINE_STRIP
                    cross_line.action = Marker.ADD
                    cross_line.pose.orientation.w = 1.0

                    # Scale
                    cross_line.scale.x = self.line_width * 0.5  # Thinner lines

                    # Color
                    cross_line.color = cross_color

                    # Lifetime
                    cross_line.lifetime = Duration(seconds=0).to_msg()

                    # Points - connect left to right
                    left_point = Point()
                    left_point.x = self.left_bound_points[i][0]
                    left_point.y = self.left_bound_points[i][1]
                    left_point.z = self.z_height

                    right_point = Point()
                    right_point.x = self.right_bound_points[i][0]
                    right_point.y = self.right_bound_points[i][1]
                    right_point.z = self.z_height

                    cross_line.points.append(left_point)
                    cross_line.points.append(right_point)

                    marker_array.markers.append(cross_line)

        # Publish
        if marker_array.markers:
            self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CSVPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
