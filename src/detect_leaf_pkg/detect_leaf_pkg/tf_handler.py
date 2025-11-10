#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TF Handler for Leaf Detection
Converts 3D points from RealSense optical frame to robot base_link
using ROS2 TF system (PointStamped version)
"""

import rclpy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import CameraInfo
import pyrealsense2 as rs


class TFHandler:
    """Handles coordinate transformations using ROS2 TF system"""

    def __init__(self, node, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        self.node = node
        self.intrinsics = None
        
        # --- Hard constraint: XYZ offsets for base_link conversion (in meters) ---
        self.x_offset = x_offset  # Default: 0m
        self.y_offset = y_offset  # Default: 0m
        self.z_offset = z_offset  # Default: 0m

        # --- Initialize TF system ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # --- Subscribe to CameraInfo to obtain intrinsics ---
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # --- Frame names (unified naming to avoid hard-coding errors) ---
        self.base_frame = 'base_link'                       # UR5e base (consistent with URDF definition)
        self.camera_frame = 'camera_link'                   # RealSense body
        self.optical_frame = 'camera_color_optical_frame'   # RealSense color optical frame

        self.get_logger().info(
            f'✓ TF Handler initialized (PointStamped version, '
            f'offsets: x={self.x_offset}m, y={self.y_offset}m, z={self.z_offset}m)'
        )

    # ------------------------------------------------------------
    # Simplified logger access
    # ------------------------------------------------------------
    def get_logger(self):
        return self.node.get_logger()

    # ------------------------------------------------------------
    # Camera intrinsics callback
    # ------------------------------------------------------------
    def camera_info_callback(self, msg):
        """Store camera intrinsics when available"""
        if self.intrinsics is not None:
            return

        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]

        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4

        self.intrinsics.coeffs = list(msg.d)
        self.get_logger().info('✓ Camera intrinsics loaded')

    # ------------------------------------------------------------
    # Pixel + depth → 3D point (camera optical frame)
    # ------------------------------------------------------------
    def pixel_to_3d(self, pixel_u, pixel_v, depth_value_mm):
        """
        Convert pixel (u,v) + depth(mm) to 3D coordinates (m) in camera_color_optical_frame
        """
        if self.intrinsics is None or depth_value_mm == 0:
            return None
        try:
            depth_m = depth_value_mm * 0.001  # Convert to meters
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                [pixel_u, pixel_v],
                depth_m
            )
            return tuple(point_3d)
        except Exception as e:
            self.get_logger().error(f'✗ 3D deprojection error: {e}')
            return None

    # ------------------------------------------------------------
    # Optical → Camera_link
    # ------------------------------------------------------------
    def optical_to_camera_link(self, point_optical):
        """
        Transform point from camera_color_optical_frame to camera_link using TF
        """
        try:
            if not self.tf_buffer.can_transform(
                self.camera_frame, self.optical_frame,
                rclpy.time.Time(), Duration(seconds=2.0)
            ):
                self.get_logger().warn(
                    f'TF not available: {self.optical_frame} → {self.camera_frame}'
                )
                return None

            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.optical_frame,
                rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )

            # Use PointStamped to simplify code
            point_stamped = PointStamped()
            point_stamped.header.stamp = transform.header.stamp
            point_stamped.header.frame_id = self.optical_frame
            point_stamped.point.x = float(point_optical[0])
            point_stamped.point.y = float(point_optical[1])
            point_stamped.point.z = float(point_optical[2])

            # Use do_transform_point to directly transform the point
            transformed = do_transform_point(point_stamped, transform)
            return [transformed.point.x, transformed.point.y, transformed.point.z]

        except Exception as e:
            self.get_logger().error(f'✗ Optical→Camera_link TF error: {e}')
            return None

    # ------------------------------------------------------------
    # Optical → Base_link (directly from camera_color_optical_frame → base_link)
    # ------------------------------------------------------------
    def camera_to_base(self, point_camera):
        """
        Transform point from camera_color_optical_frame to robot base_link
        Direct transform using hand-to-eye calibration (defined in URDF)
        """
        try:
            if not self.tf_buffer.can_transform(
                self.base_frame, self.optical_frame,
                rclpy.time.Time(), Duration(seconds=2.0)
            ):
                self.get_logger().warn(
                    f'TF not available: {self.optical_frame} → {self.base_frame}'
                )
                return None

            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame,
                rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )

            # Use PointStamped to simplify code
            point_stamped = PointStamped()
            point_stamped.header.stamp = transform.header.stamp
            point_stamped.header.frame_id = self.optical_frame
            point_stamped.point.x = float(point_camera[0])
            point_stamped.point.y = float(point_camera[1])
            point_stamped.point.z = float(point_camera[2])

            # Use do_transform_point to directly transform the point
            transformed = do_transform_point(point_stamped, transform)
            
            # Log transformation details (before applying offsets)
            self.get_logger().debug(
                f'TF transform: camera_optical({point_camera[0]:.3f}, {point_camera[1]:.3f}, {point_camera[2]:.3f}) '
                f'-> base_link({transformed.point.x:.3f}, {transformed.point.y:.3f}, {transformed.point.z:.3f}) '
                f'[offsets: ({self.x_offset:.3f}, {self.y_offset:.3f}, {self.z_offset:.3f})]'
            )
            
            # Apply hard constraint: add offsets to all coordinates
            result_x = transformed.point.x + self.x_offset
            result_y = transformed.point.y + self.y_offset
            result_z = transformed.point.z + self.z_offset
            
            self.get_logger().debug(
                f'Final coordinates (base_link + offsets): ({result_x:.3f}, {result_y:.3f}, {result_z:.3f})'
            )
            
            return [result_x, result_y, result_z]

        except Exception as e:
            self.get_logger().error(f'✗ Camera→Base TF error: {e}')
            return None
