#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Automation Task Orchestrator
Automated task management integrating leaf detection and robot arm control
"""

import rclpy
import rclpy.time
import rclpy.duration
import json
from rclpy.node import Node
from arm_msgs.srv import LeafDetectionSrv
from arduino_communication.srv import LeafCommand
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import subprocess
import time
import sys
import traceback

# For reading current robot pose
from tf2_ros import Buffer, TransformListener


class AutomationOrchestrator(Node):
    """
    Automation Task Orchestrator
    
    Coordinates leaf detection and robot arm movement for automated picking task flow:
    1. Detect leaf positions
    2. Move robot arm to leaf positions for picking
    3. Move robot arm to trash bin to discard leaves
    4. Return to initial position
    """
    
    def __init__(self):
        super().__init__('automation_orchestrator')
        
        # Initialize TF2 for reading robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize leaf detection service client
        self.client = self.create_client(LeafDetectionSrv, 'leaf_detection_srv')
        
        # Initialize Arduino communication service client
        self.arduino_client = self.create_client(LeafCommand, 'send_command')
        
        # Publisher to notify detection handler that automation task is running
        self.automation_running_pub = self.create_publisher(Bool, '/automation_task/running', 10)
        
        # Load configuration parameters
        self._load_parameters()
        
        # Try to get initial robot position as home (wait a bit for TF to be ready)
        self._initialize_home_from_current_pose()
        
        # Log configuration information
        self._log_configuration()
    
    def _load_parameters(self):
        """Load and store all ROS parameters"""
        # Detection parameters
        self.declare_parameter('min_area', 0.0)
        self.declare_parameter('confidence', 0.0)
        
        # Coordinate bias parameters (applied to all coordinates by default)
        self.declare_parameter('bias_x', 0.0)     # X-axis bias (meters)
        self.declare_parameter('bias_y', 0.0)     # Y-axis bias (meters)
        self.declare_parameter('bias_z', 0.15)     # Z-axis bias (meters)
        
        # Z-axis constraints (safety limits)
        self.declare_parameter('z_min', 0.05)    # Minimum Z coordinate (safety lower bound)
        self.declare_parameter('z_max', 1.80)     # Maximum Z coordinate (safety upper bound)
        
        # Home position configuration (Cartesian)
        self.declare_parameter('home_x', 0.25)
        self.declare_parameter('home_y', 0.10)
        self.declare_parameter('home_z', 0.55)
        
        # Home joint angles (radians) - hardcoded default
        self.home_joints = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': -1.3090,
            'elbow_joint': 1.5708,
            'wrist_1_joint': -1.8326,
            'wrist_2_joint': -1.5708,
            'wrist_3_joint': 0.0
        }
        
        # Trash bin position configuration
        self.declare_parameter('trash_x', 0.10)
        self.declare_parameter('trash_y', 0.50)
        self.declare_parameter('trash_z', 0.25)  # Trash bin discard position (increased from 0.20 to 0.25 for safety)
        
        # Task flow parameters
        self.declare_parameter('wait_between_leaves', 2.0)    # Wait time between processing leaves (seconds)
        self.declare_parameter('arm_movement_timeout', 60.0)  # Robot arm movement timeout (seconds)
        self.declare_parameter('detection_timeout', 60.0)     # Detection service timeout (seconds)
        self.declare_parameter('arduino_action_wait', 3.0)    # Wait time after Arduino action (seconds)
        self.declare_parameter('spray_height_offset', 0.05)   # Additional height offset for spray operation (meters)
        self.declare_parameter('use_joint_constraints', True) # Enable joint path constraints for arm movement
        self.declare_parameter('max_velocity_scaling', 0.15)  # Maximum velocity scaling factor (0.0-1.0, default: 0.15 = 15%)
        self.declare_parameter('max_acceleration_scaling', 0.15)  # Maximum acceleration scaling factor (0.0-1.0, default: 0.15 = 15%)
        self.declare_parameter('unhealthy_z_threshold', 0.20)  # Z coordinate threshold for unhealthy leaves
        self.declare_parameter('unhealthy_z_min', 0.15)  # Minimum Z coordinate for unhealthy leaves when Z < threshold
        
        # Store parameter values
        self.min_area = self.get_parameter('min_area').value
        self.confidence = self.get_parameter('confidence').value
        self.bias_x = self.get_parameter('bias_x').value
        self.bias_y = self.get_parameter('bias_y').value
        self.bias_z = self.get_parameter('bias_z').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_z = self.get_parameter('home_z').value
        self.trash_x = self.get_parameter('trash_x').value
        self.trash_y = self.get_parameter('trash_y').value
        self.trash_z = self.get_parameter('trash_z').value
        self.wait_between_leaves = self.get_parameter('wait_between_leaves').value
        self.arm_movement_timeout = self.get_parameter('arm_movement_timeout').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.arduino_action_wait = self.get_parameter('arduino_action_wait').value
        self.spray_height_offset = self.get_parameter('spray_height_offset').value
        self.use_joint_constraints = self.get_parameter('use_joint_constraints').value
        self.max_velocity_scaling = self.get_parameter('max_velocity_scaling').value
        self.max_acceleration_scaling = self.get_parameter('max_acceleration_scaling').value
        self.unhealthy_z_threshold = self.get_parameter('unhealthy_z_threshold').value
        self.unhealthy_z_min = self.get_parameter('unhealthy_z_min').value
    
    def _initialize_home_from_current_pose(self):
        """
        Initialize home position - using hardcoded joint angles
        """
        self.get_logger().info("Using hardcoded home joint angles:")
        for joint, angle in self.home_joints.items():
            self.get_logger().info(f"  {joint}: {angle:.4f} rad ({angle * 180 / 3.14159:.2f}°)")
    
    def _log_configuration(self):
        """Log configuration information"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("Automation Task Orchestrator started")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Detection parameters: min_area={self.min_area}, confidence={self.confidence}")
        self.get_logger().info(f"Coordinate bias: bias_x={self.bias_x:.3f}m, bias_y={self.bias_y:.3f}m, bias_z={self.bias_z:.3f}m")
        self.get_logger().info(f"Z-axis constraints: z_min={self.z_min}m, z_max={self.z_max}m")
        self.get_logger().info(f"Home position: ({self.home_x:.3f}, {self.home_y:.3f}, {self.home_z:.3f})")
        self.get_logger().info(f"Trash bin position: ({self.trash_x:.3f}, {self.trash_y:.3f}, {self.trash_z:.3f})")
        self.get_logger().info(f"Flow parameters: wait_between_leaves={self.wait_between_leaves}s")
        self.get_logger().info(f"Spray height offset: {self.spray_height_offset:.3f}m")
        self.get_logger().info(f"Joint constraints: {'ENABLED' if self.use_joint_constraints else 'DISABLED'}")
        self.get_logger().info(f"Speed limits: velocity={self.max_velocity_scaling*100:.0f}%, acceleration={self.max_acceleration_scaling*100:.0f}%")
        self.get_logger().info(f"Unhealthy leaf Z adjustment: threshold={self.unhealthy_z_threshold:.3f}m, min={self.unhealthy_z_min:.3f}m")
        self.get_logger().info("=" * 80)
    
    def wait_for_service(self, timeout_sec=30.0):
        """
        Wait for leaf detection service and Arduino service to be available
        
        Args:
            timeout_sec: Timeout duration (seconds)
            
        Returns:
            bool: Whether the services are available
        """
        self.get_logger().info(f"Waiting for leaf detection service... (timeout: {timeout_sec}s)")
        
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(" Leaf detection service unavailable!")
            self.get_logger().error("Please ensure the following service is running:")
            self.get_logger().error("  - ros2 launch detect_leaf_pkg leaf_detection_server.launch.py")
            return False
        
        self.get_logger().info(" Leaf detection service ready")
        
        # Wait for Arduino service
        self.get_logger().info(f"Waiting for Arduino communication service... (timeout: {timeout_sec}s)")
        if not self.arduino_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(" Arduino communication service unavailable!")
            self.get_logger().error("Please ensure the following service is running:")
            self.get_logger().error("  - ros2 run arduino_communication leafServerNode")
            return False
        
        self.get_logger().info(" Arduino communication service ready")
        return True
    
    def detect_leaves(self):
        """
        Call leaf detection service
        
        Returns:
            LeafDetectionSrv.Response or None: Detection results
        """
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Leaf detection service unavailable")
            return None
        
        try:
            request = LeafDetectionSrv.Request()
            request.command = "detect"
            request.min_area = self.min_area
            request.confidence = self.confidence
            
            self.get_logger().info(
                f"Sending detection request: min_area={self.min_area}, confidence={self.confidence}"
            )
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.detection_timeout)
            
            if not future.done():
                self.get_logger().error(f"Detection service call timeout ({self.detection_timeout}s)")
                return None
            
            response = future.result()
            if response.success:
                self._log_detection_results(response)
                return response
            else:
                self.get_logger().error(f"Detection failed: {response.message}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Detection service call exception: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().debug(f"Exception traceback:\n{traceback.format_exc()}")
            return None
    
    def _log_detection_results(self, response):
        """
        Log detection results
        
        Args:
            response: Detection service response
        """
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info(" Leaf Detection Results")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Status: {response.message}")
        self.get_logger().info(f"Leaves detected: {response.num_leaves}")
        
        # Parse yellow tape information for logging
        has_yellow_tape_list = []
        if response.debug_info:
            try:
                debug_data = json.loads(response.debug_info)
                has_yellow_tape_list = debug_data.get('has_yellow_tape', [])
            except (json.JSONDecodeError, TypeError):
                pass
        
        for i, point in enumerate(response.coordinates):
            has_yellow = has_yellow_tape_list[i] if i < len(has_yellow_tape_list) else False
            leaf_type = "[Unhealthy]" if has_yellow else "[Healthy]"
            self.get_logger().info(
                f"  Leaf {i+1}: X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m {leaf_type}"
            )
        
        if response.debug_info:
            self.get_logger().debug(f"Debug info: {response.debug_info}")
        
        self.get_logger().info("=" * 80 + "\n")
    
    def _clamp_z_coordinate(self, z):
        """
        Clamp Z coordinate within safe range
        
        Args:
            z: Original Z coordinate
            
        Returns:
            float: Clamped Z coordinate
        """
        original_z = z
        if z < self.z_min:
            z = self.z_min
            self.get_logger().warn(
                f" Z coordinate {original_z:.3f}m below minimum {self.z_min:.3f}m, "
                f"clamped to {z:.3f}m"
            )
        elif z > self.z_max:
            z = self.z_max
            self.get_logger().warn(
                f" Z coordinate {original_z:.3f}m above maximum {self.z_max:.3f}m, "
                f"clamped to {z:.3f}m"
            )
        return z
    
    def move_arm_to_pose(self, x, y, z, apply_bias=True):
        """
        Move robot arm to specified position
        
        Args:
            x: X coordinate (meters)
            y: Y coordinate (meters)
            z: Z coordinate (meters)
            apply_bias: Whether to apply coordinate bias (default True)
            
        Returns:
            bool: Whether movement was successful
        """
        # Apply coordinate bias (enabled by default)
        if apply_bias:
            target_x = x + self.bias_x
            target_y = y + self.bias_y
            # Add bias_z to account for tool positioning
            target_z = z + self.bias_z
            self.get_logger().info(
                f"Applying coordinate bias: original({x:.3f}, {y:.3f}, {z:.3f}) -> "
                f"biased({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) "
                f"[bias=({self.bias_x:.3f}, {self.bias_y:.3f}, {self.bias_z:.3f})]"
            )
        else:
            target_x = x
            target_y = y
            target_z = z
        
        # Apply Z-axis constraints
        original_clamped_z = target_z
        target_z = self._clamp_z_coordinate(target_z)
        if original_clamped_z != target_z:
            self.get_logger().warn(
                f"Z coordinate clamped: {original_clamped_z:.3f}m -> {target_z:.3f}m"
            )
        
        self.get_logger().info(
            f"Moving robot arm to position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )
        
        try:
            cmd = [
                'ros2', 'launch', 'arm_manipulation', 'move_arm_to_pose_launch.py',
                f'x:={target_x}', f'y:={target_y}', f'z:={target_z}',
                f'use_constraints:={str(self.use_joint_constraints).lower()}',
                f'max_velocity_scaling:={self.max_velocity_scaling}',
                f'max_acceleration_scaling:={self.max_acceleration_scaling}'
            ]
            
            self.get_logger().info(f"Executing command: {' '.join(cmd)}")
            
            # Ensure correct environment variables (inherit from current process)
            import os
            env = os.environ.copy()
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.arm_movement_timeout,
                env=env,  # Use current environment variables
            )
            
            # Always print output for debugging
            if result.stdout and result.stdout.strip():
                self.get_logger().info(f"Command output:\n{result.stdout}")
            if result.stderr and result.stderr.strip():
                self.get_logger().warn(f"Command warnings/errors:\n{result.stderr}")
            
            if result.returncode == 0:
                self.get_logger().info(" Robot arm movement successful")
                return True
            else:
                self.get_logger().error(" Robot arm movement failed")
                self.get_logger().error(f"Return code: {result.returncode}")
                if result.stdout and result.stdout.strip():
                    self.get_logger().error(f"Standard output:\n{result.stdout}")
                if result.stderr and result.stderr.strip():
                    self.get_logger().error(f"Error output:\n{result.stderr}")
                else:
                    self.get_logger().error("(No error output, launch file may have failed to start)")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f" Robot arm movement timeout (>{self.arm_movement_timeout}s)"
            )
            return False
        except Exception as e:
            self.get_logger().error(f" Robot arm movement exception: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().error(f"Exception traceback:\n{traceback.format_exc()}")
            return False
    
    def move_arm_to_trash(self):
        """
        Move robot arm to trash bin position
        
        Returns:
            bool: Whether movement was successful
        """
        self.get_logger().info(
            f"Moving to trash bin: ({self.trash_x:.3f}, {self.trash_y:.3f}, {self.trash_z:.3f})"
        )
        return self.move_arm_to_pose(
            self.trash_x, self.trash_y, self.trash_z, 
            apply_bias=False  # Trash bin position does not use bias
        )
    
    def move_arm_to_home(self):
        """
        Move robot arm to home position using MoveIt planning (same pipeline as leaf/trash moves).
        
        Returns:
            bool: Whether movement was successful
        """
        self.get_logger().info(
            f"Returning to home position using MoveIt planning at "
            f"({self.home_x:.3f}, {self.home_y:.3f}, {self.home_z:.3f})..."
        )
        
        # For home, do NOT apply bias; use the configured Cartesian position directly.
        success = self.move_arm_to_pose(
            self.home_x,
            self.home_y,
            self.home_z,
            apply_bias=False,
        )
        
        if success:
            self.get_logger().info(" Robot arm returned to home position (MoveIt)")
        else:
            self.get_logger().error(" Return to home via MoveIt planning failed")
        
        return success
    
    def send_arduino_command(self, command):
        """
        Send command to Arduino via service
        
        Args:
            command: Command string (e.g., "VACUUM_ON", "VACUUM_OFF", "SPRAY_ON", "SPRAY_OFF")
            
        Returns:
            bool: Whether command was sent successfully
        """
        if not self.arduino_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Arduino communication service unavailable")
            return False
        
        try:
            request = LeafCommand.Request()
            request.command = command
            
            self.get_logger().info(f"Sending Arduino command: {command}")
            
            future = self.arduino_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                self.get_logger().error("Arduino command service call timeout")
                return False
            
            response = future.result()
            self.get_logger().info(f"Arduino response: {response.response}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Arduino command exception: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().debug(f"Exception traceback:\n{traceback.format_exc()}")
            return False
    
    def _process_single_leaf(self, leaf_index, total_leaves, leaf_point, has_yellow_tape=False):
        """
        Process a single leaf
        
        Args:
            leaf_index: Leaf index (starting from 1)
            total_leaves: Total number of leaves
            leaf_point: Leaf position coordinates
            has_yellow_tape: Whether the leaf has yellow tape (unhealthy)
            
        Returns:
            bool: Whether processing was successful
        """
        leaf_type = "unhealthy (with yellow tape)" if has_yellow_tape else "healthy"
        self.get_logger().info(f"\nProcessing leaf {leaf_index}/{total_leaves} ({leaf_type})...")
        
        if has_yellow_tape:
            # Unhealthy leaf workflow:
            # 1. Move to spray height first (above leaf)
            # 2. Move vertically down to leaf target Z
            # 3. Open vacuum (VACUUM_ON)
            # 4. Wait 3 seconds
            # 5. Move to trash bin (vacuum stays on during movement)
            # 6. Close vacuum (VACUUM_OFF) when arrived at bin
            
            self.get_logger().info(f"Leaf {leaf_index} is unhealthy, starting vacuum pickup...")
            
            # Step 1: Move to spray height first (above the leaf)
            # 使用原始识别 Z 做喷雾高度，始终应用 bias，这样 XY 与第二步一致
            spray_height = leaf_point.z + self.spray_height_offset
            self.get_logger().info(
                f"Step 1: Moving to spray height above leaf: Z={spray_height:.3f}m "
                f"(detected Z={leaf_point.z:.3f}m + offset={self.spray_height_offset:.3f}m, "
                f"XY with bias)"
            )
            if not self.move_arm_to_pose(leaf_point.x, leaf_point.y, spray_height, apply_bias=True):
                self.get_logger().warn(f" Leaf {leaf_index} movement to spray height failed")
                return False
            
            # Step 2: Move vertically down to leaf target Z
            # 只改变目标 Z，不改变 apply_bias，保证两步的 XY 完全一致（都带 bias）
            original_z = leaf_point.z
            
            # target_z_effective: 末端执行器期望到达的“物理”Z（不包含 bias_z）
            # target_z_command: 传给 move_arm_to_pose 的 Z（内部仍会加 bias_z）
            if original_z >= self.unhealthy_z_threshold:
                # 高于阈值：沿用原始 Z（正常带 Z bias）
                target_z_effective = original_z
                target_z_command = original_z
                self.get_logger().info(
                    f"Step 2: Detected Z {original_z:.3f}m >= threshold {self.unhealthy_z_threshold:.3f}m, "
                    f"using original Z with bias (command_z={target_z_command:.3f}m, bias_z={self.bias_z:.3f}m)"
                )
            else:
                # 低于阈值：希望最终物理 Z = unhealthy_z_min，且“不加 bias”
                # 由于 move_arm_to_pose 内部会对 Z 加上 bias_z，这里先减去 bias_z 再传入，
                # 这样加回去后有效 Z 仍然是 unhealthy_z_min
                target_z_effective = self.unhealthy_z_min
                target_z_command = self.unhealthy_z_min - self.bias_z
                self.get_logger().info(
                    f"Step 2: Detected Z {original_z:.3f}m < threshold {self.unhealthy_z_threshold:.3f}m, "
                    f"using unhealthy_z_min without extra Z bias: effective_Z={target_z_effective:.3f}m "
                    f"(command_z={target_z_command:.3f}m, bias_z={self.bias_z:.3f}m)"
                )
            
            self.get_logger().info(
                f"Moving vertically down to leaf target Z (effective): {target_z_effective:.3f}m "
                f"with XY bias applied"
            )
            if not self.move_arm_to_pose(leaf_point.x, leaf_point.y, target_z_command, apply_bias=True):
                self.get_logger().warn(f" Leaf {leaf_index} vertical movement to target Z failed")
                return False
            
            # Step 3: Open vacuum
            if not self.send_arduino_command("VACUUM_ON"):
                self.get_logger().warn(f" Leaf {leaf_index} failed to open vacuum")
                return False
            
            # Step 4: Wait 3 seconds
            self.get_logger().info(f"Waiting {self.arduino_action_wait}s for vacuum to pick up leaf...")
            time.sleep(self.arduino_action_wait)
            
            # Step 5: Move to trash bin (vacuum stays on)
            self.get_logger().info(f"Moving to trash bin with vacuum ON...")
            if not self.move_arm_to_trash():
                self.get_logger().warn(f" Leaf {leaf_index} movement to trash bin failed")
                # Try to close vacuum even if movement failed
                self.send_arduino_command("VACUUM_OFF")
                return False
            
            # Step 6: Close vacuum when arrived at bin
            self.get_logger().info(f"Arrived at trash bin, closing vacuum...")
            if not self.send_arduino_command("VACUUM_OFF"):
                self.get_logger().warn(f" Leaf {leaf_index} failed to close vacuum")
                return False
            
            self.get_logger().info(f" Leaf {leaf_index} discarded successfully")
        
        else:
            # Healthy leaf workflow:
            # Step 1: Move to leaf position with spray height offset
            target_z = leaf_point.z + self.spray_height_offset
            self.get_logger().info(
                f"Applying spray height offset: original Z={leaf_point.z:.3f}m, "
                f"with offset={target_z:.3f}m (offset={self.spray_height_offset:.3f}m)"
            )
            
            if not self.move_arm_to_pose(leaf_point.x, leaf_point.y, target_z):
                self.get_logger().warn(f" Leaf {leaf_index} movement failed")
                return False
            
            # Step 2: Open spray (SPRAY_ON)
            self.get_logger().info(f"Leaf {leaf_index} is healthy, starting spray treatment...")
            
            if not self.send_arduino_command("SPRAY_ON"):
                self.get_logger().warn(f" Leaf {leaf_index} failed to open spray")
                return False
            
            # Step 3: Wait 3 seconds
            self.get_logger().info(f"Waiting {self.arduino_action_wait}s for spray treatment...")
            time.sleep(self.arduino_action_wait)
            
            # Step 4: Close spray (SPRAY_OFF)
            if not self.send_arduino_command("SPRAY_OFF"):
                self.get_logger().warn(f" Leaf {leaf_index} failed to close spray")
                return False
            
            self.get_logger().info(f" Leaf {leaf_index} spray treatment completed")
        
        return True
    
    def run_automation_loop(self):
        """
        Run automation task loop
        
        Returns:
            bool: Whether task completed successfully
        """
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Starting automation task loop")
        self.get_logger().info("=" * 80)
        
        # Notify detection handler that automation task has started (fix blue box positions)
        msg = Bool()
        msg.data = True
        self.automation_running_pub.publish(msg)
        self.get_logger().info("Notified detection handler: automation task started, blue box positions will be fixed")
        
        # Step 1: Detect leaves
        response = self.detect_leaves()
        
        if response is None:
            self.get_logger().error("Detection failed, task terminated")
            return False
        
        if response.num_leaves == 0:
            self.get_logger().warn("No leaves detected, task ended")
            return False
        
        # Parse yellow tape information from debug_info
        has_yellow_tape_list = []
        if response.debug_info:
            try:
                debug_data = json.loads(response.debug_info)
                has_yellow_tape_list = debug_data.get('has_yellow_tape', [])
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warn("Could not parse yellow tape info from debug_info")
        
        # Log leaf types
        unhealthy_count = sum(1 for has_yellow in has_yellow_tape_list if has_yellow)
        healthy_count = response.num_leaves - unhealthy_count
        self.get_logger().info(
            f"Leaf classification: {unhealthy_count} unhealthy (with yellow tape), "
            f"{healthy_count} healthy"
        )
        
        # Step 2: Process all detected leaves
        success_count = 0
        for i, leaf_point in enumerate(response.coordinates):
            has_yellow = has_yellow_tape_list[i] if i < len(has_yellow_tape_list) else False
            
            if self._process_single_leaf(i + 1, response.num_leaves, leaf_point, has_yellow):
                success_count += 1
            
            # Wait before processing next leaf (only if not the last one)
            if i < response.num_leaves - 1:
                self.get_logger().info(
                    f"Waiting {self.wait_between_leaves}s before processing next leaf..."
                )
                time.sleep(self.wait_between_leaves)
        
        # Step 3: Print task completion summary
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Leaf processing complete")
        self.get_logger().info(f"Successfully processed: {success_count}/{response.num_leaves} leaves")
        self.get_logger().info("=" * 80 + "\n")
        
        # Step 4: Return to home position
        self.get_logger().info("Starting return to home position...")
        home_success = self.move_arm_to_home()
        
        if home_success:
            self.get_logger().info(" Returned to home position")
        else:
            self.get_logger().error("\n" + "!" * 80)
            self.get_logger().error(" WARNING: Return to home position failed! ")
            self.get_logger().error("!" * 80)
            self.get_logger().error(
                f"Target position: ({self.home_x:.3f}, {self.home_y:.3f}, {self.home_z:.3f})"
            )
            self.get_logger().error("Please check actual arm position in RViz")
            self.get_logger().error("Manual arm movement or path re-planning may be needed")
            self.get_logger().error("!" * 80 + "\n")
        
        # Notify detection handler that automation task has ended (blue box positions can update normally)
        msg = Bool()
        msg.data = False
        self.automation_running_pub.publish(msg)
        self.get_logger().info("Notified detection handler: automation task ended, blue box positions will update normally")
        
        # Final summary
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Automation task flow complete")
        if not home_success:
            self.get_logger().info(" Notice: Robot arm did not successfully return to home position")
        self.get_logger().info("=" * 80 + "\n")
        
        # Consider task flow complete even if return home failed
        return True


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    orchestrator = AutomationOrchestrator()
    
    # Wait for service availability
    if not orchestrator.wait_for_service(timeout_sec=30.0):
        orchestrator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    
    try:
        # Run automation task
        success = orchestrator.run_automation_loop()
    except KeyboardInterrupt:
        orchestrator.get_logger().info("\nReceived interrupt signal, shutting down...")
        success = False
    except Exception as e:
        orchestrator.get_logger().error(f"\nUnhandled exception: {str(e)}")
        orchestrator.get_logger().error(f"Exception traceback:\n{traceback.format_exc()}")
        success = False
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()
        print("Automation orchestrator shutdown complete")
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
