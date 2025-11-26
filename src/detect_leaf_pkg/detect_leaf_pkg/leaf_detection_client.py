#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Detection Client Node
Client for calling leaf detection service
"""

import rclpy
import json
from rclpy.node import Node
from arm_msgs.srv import LeafDetectionSrv
from geometry_msgs.msg import Point
import sys


class LeafDetectionClient(Node):
    """Client for leaf detection service"""
    
    def __init__(self):
        super().__init__('leaf_detection_client')
        self.client = self.create_client(LeafDetectionSrv, 'leaf_detection_srv')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('✓ Service available! Ready for commands.')
    
    def send_request(self, min_area=0.0, confidence=0.0):
        """Send detection request to service"""
        try:
            request = LeafDetectionSrv.Request()
            request.command = "detect"
            request.min_area = min_area
            request.confidence = confidence
            
            self.get_logger().info(f"Sending request - min_area: {min_area}, confidence: {confidence}")
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.log_detection_results(response)
                else:
                    self.get_logger().error(f"Service failed: {response.message}")
                return response
            else:
                self.get_logger().error("Service call failed: No response received")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            return None
    
    def log_detection_results(self, response):
        """Format and log detection results"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("🌿 Leaf Detection Results")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Status: {response.message}")
        self.get_logger().info(f"Leaves found: {response.num_leaves}")
        
        # Parse additional info from debug_info JSON
        has_yellow = []
        yellow_ratio = []
        health_status = []
        
        if response.debug_info:
            try:
                debug_data = json.loads(response.debug_info)
                has_yellow = debug_data.get('has_yellow_tape', [])
                yellow_ratio = debug_data.get('yellow_ratio', [])
                health_status = debug_data.get('health_status', [])
            except (json.JSONDecodeError, TypeError):
                # If debug_info is not JSON, just use it as-is
                pass
        
        for i, point in enumerate(response.coordinates):
            tape_flag = has_yellow[i] if i < len(has_yellow) else False
            ratio = yellow_ratio[i] if i < len(yellow_ratio) else 0.0
            health_label = health_status[i] if i < len(health_status) else ('unhealthy' if tape_flag else 'healthy')
            tape_tag = '[Tape]' if tape_flag else '[Healthy]'
            self.get_logger().info(
                f"  Leaf {i+1}: "
                f"X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m "
                f"{tape_tag} ratio={ratio:.2f} status={health_label}"
            )
        
        if response.debug_info:
            try:
                # Only log if it's not JSON (to avoid duplicate info)
                debug_data = json.loads(response.debug_info)
                if debug_data.get('debug_info'):
                    self.get_logger().info(f"Debug info: {debug_data.get('debug_info')}")
            except (json.JSONDecodeError, TypeError):
                self.get_logger().info(f"Debug info: {response.debug_info}")
        
        self.get_logger().info("=" * 80 + "\n")


def print_usage():
    """Display usage instructions"""
    print("\nUsage:")
    print("  Enter commands in format: <min_area> <confidence>")
    print("  Example: '2000 0.0' - Detect leaves with min_area=2000")
    print("  Commands:")
    print("    <min_area> <confidence> - Detect leaves with specified parameters")
    print("    exit                    - Quit the program")
    print()


def main(args=None):
    rclpy.init(args=args)
    client = LeafDetectionClient()
    
    print_usage()
    
    try:
        while True:
            try:
                user_input = input("Enter command (min_area confidence) or 'exit' to quit: ").strip()
                
                if user_input.lower() == 'exit':
                    break
                
                if not user_input:
                    continue
                
                parts = user_input.split()
                if len(parts) >= 2:
                    try:
                        min_area = float(parts[0])
                        confidence = float(parts[1])
                        client.send_request(min_area, confidence)
                    except ValueError:
                        client.get_logger().error("Invalid input: min_area and confidence must be numbers")
                elif len(parts) == 1:
                    try:
                        min_area = float(parts[0])
                        client.send_request(min_area, 0.0)
                    except ValueError:
                        client.get_logger().error("Invalid input: min_area must be a number")
                else:
                    # Default request
                    client.send_request(0.0, 0.0)
                
            except KeyboardInterrupt:
                print("\nOperation cancelled by user")
                break
            except Exception as e:
                client.get_logger().error(f"Unexpected error: {str(e)}")
                continue
                
    finally:
        client.destroy_node()
        rclpy.shutdown()
        print("Client shutdown complete.")


if __name__ == '__main__':
    main()

