#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Automation Task Orchestrator
Automated task management integrating leaf detection and robot arm control
"""

import rclpy
from rclpy.node import Node
from arm_msgs.srv import LeafDetectionSrv
from geometry_msgs.msg import Point
import subprocess
import time
import sys
import traceback


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
        
        # Initialize leaf detection service client
        self.client = self.create_client(LeafDetectionSrv, 'leaf_detection_srv')
        
        # Load configuration parameters
        self._load_parameters()
        
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
        self.declare_parameter('bias_z', 0.5)     # Z-axis bias (meters)
        
        # Z-axis constraints (safety limits)
        self.declare_parameter('z_min', 0.05)    # Minimum Z coordinate (safety lower bound)
        self.declare_parameter('z_max', 1.80)     # Maximum Z coordinate (safety upper bound)
        
        # Home position configuration
        self.declare_parameter('home_x', 0.25)
        self.declare_parameter('home_y', 0.10)
        self.declare_parameter('home_z', 0.55)
        
        # Trash bin position configuration
        self.declare_parameter('trash_x', 0.10)
        self.declare_parameter('trash_y', 0.50)
        self.declare_parameter('trash_z', 0.20)  # Trash bin discard position
        
        # Task flow parameters
        self.declare_parameter('wait_between_leaves', 2.0)  # Wait time between processing leaves (seconds)
        self.declare_parameter('arm_movement_timeout', 60.0)  # Robot arm movement timeout (seconds)
        self.declare_parameter('detection_timeout', 10.0)    # Detection service timeout (seconds)
        
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
        self.get_logger().info("=" * 80)
    
    def wait_for_service(self, timeout_sec=30.0):
        """
        Wait for leaf detection service to be available
        
        Args:
            timeout_sec: Timeout duration (seconds)
            
        Returns:
            bool: Whether the service is available
        """
        self.get_logger().info(f"Waiting for leaf detection service... (timeout: {timeout_sec}s)")
        
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("❌ Leaf detection service unavailable!")
            self.get_logger().error("Please ensure the following service is running:")
            self.get_logger().error("  - ros2 launch detect_leaf_pkg leaf_detection_server.launch.py")
            return False
        
        self.get_logger().info("✓ Leaf detection service ready")
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
        self.get_logger().info("🌿 Leaf Detection Results")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Status: {response.message}")
        self.get_logger().info(f"Leaves detected: {response.num_leaves}")
        
        for i, point in enumerate(response.coordinates):
            self.get_logger().info(
                f"  Leaf {i+1}: X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m"
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
                f"⚠ Z coordinate {original_z:.3f}m below minimum {self.z_min:.3f}m, "
                f"clamped to {z:.3f}m"
            )
        elif z > self.z_max:
            z = self.z_max
            self.get_logger().warn(
                f"⚠ Z coordinate {original_z:.3f}m above maximum {self.z_max:.3f}m, "
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
                f'x:={target_x}', f'y:={target_y}', f'z:={target_z}'
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
                self.get_logger().info("✓ Robot arm movement successful")
                return True
            else:
                self.get_logger().error("❌ Robot arm movement failed")
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
                f"❌ Robot arm movement timeout (>{self.arm_movement_timeout}s)"
            )
            return False
        except Exception as e:
            self.get_logger().error(f"❌ Robot arm movement exception: {str(e)}")
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
        Move robot arm to home position
        
        Returns:
            bool: Whether movement was successful
        """
        self.get_logger().info(
            f"Returning to home position: ({self.home_x:.3f}, {self.home_y:.3f}, {self.home_z:.3f})"
        )
        return self.move_arm_to_pose(
            self.home_x, self.home_y, self.home_z, 
            apply_bias=False  # Home position does not use bias
        )
    
    def _process_single_leaf(self, leaf_index, total_leaves, leaf_point):
        """
        Process a single leaf
        
        Args:
            leaf_index: Leaf index (starting from 1)
            total_leaves: Total number of leaves
            leaf_point: Leaf position coordinates
            
        Returns:
            bool: Whether processing was successful
        """
        self.get_logger().info(f"\nProcessing leaf {leaf_index}/{total_leaves}...")
        
        # Move to leaf position for picking
        if not self.move_arm_to_pose(leaf_point.x, leaf_point.y, leaf_point.z):
            self.get_logger().warn(f"⚠ Leaf {leaf_index} picking failed")
            return False
        
        self.get_logger().info(f"✓ Leaf {leaf_index} picked successfully")
        
        # Move to trash bin to discard
        self.get_logger().info(f"Moving to trash bin to discard leaf {leaf_index}...")
        if not self.move_arm_to_trash():
            self.get_logger().warn(f"⚠ Leaf {leaf_index} discard failed")
            return False
        
        self.get_logger().info(f"✓ Leaf {leaf_index} discarded successfully")
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
        
        # Step 1: Detect leaves
        response = self.detect_leaves()
        
        if response is None:
            self.get_logger().error("Detection failed, task terminated")
            return False
        
        if response.num_leaves == 0:
            self.get_logger().warn("No leaves detected, task ended")
            return False
        
        # Step 2: Process all detected leaves
        success_count = 0
        for i, leaf_point in enumerate(response.coordinates):
            if self._process_single_leaf(i + 1, response.num_leaves, leaf_point):
                success_count += 1
            
            # Wait before processing next leaf
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
            self.get_logger().info("✓ Returned to home position")
        else:
            self.get_logger().error("\n" + "!" * 80)
            self.get_logger().error("❌❌❌ WARNING: Return to home position failed! ❌❌❌")
            self.get_logger().error("!" * 80)
            self.get_logger().error(
                f"Target position: ({self.home_x:.3f}, {self.home_y:.3f}, {self.home_z:.3f})"
            )
            self.get_logger().error("Please check actual arm position in RViz")
            self.get_logger().error("Manual arm movement or path re-planning may be needed")
            self.get_logger().error("!" * 80 + "\n")
        
        # Final summary
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Automation task flow complete")
        if not home_success:
            self.get_logger().info("⚠ Notice: Robot arm did not successfully return to home position")
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
