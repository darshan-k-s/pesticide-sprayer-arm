#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Detection Server Node
Refactored based on camera package framework - uses service interface instead of continuous publishing
"""

import threading
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_msgs.srv import LeafDetectionSrv
from .detection_handler import DetectionHandler
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from .tf_handler import TFHandler


class LeafDetectionServer(Node):
    """Leaf detection server node with service interface"""
    
    def __init__(self):
        super().__init__('leaf_detection_server')
        
        # Setup callback groups for concurrent processing
        self.service_group = ReentrantCallbackGroup()
        self.image_group = ReentrantCallbackGroup()
        
        # Setup components (modular design)
        self.tf_handler = TFHandler(self)
        
        self.detector = DetectionHandler(
            node=self,
            tf_handler=self.tf_handler
        )
        
        # Setup image subscribers with synchronization
        self.setup_subscribers()
        
        # Create service
        self.srv = self.create_service(
            LeafDetectionSrv,
            'leaf_detection_srv',
            self.handle_leaf_detection_request,
            callback_group=self.service_group
        )
        
        self.get_logger().info('🌿 Leaf Detection Server ready')
        self.get_logger().info('📡 Service /leaf_detection_srv available')
    
    def setup_subscribers(self):
        """Configure image subscribers and synchronizer"""
        self.color_sub = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw',
            callback_group=self.image_group
        )
        
        self.depth_sub = Subscriber(
            self,
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            callback_group=self.image_group
        )
        
        # Synchronize color and depth images
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,  # 100ms tolerance for synchronization
        )
        self.ts.registerCallback(self.detector.update_frames)
    
    async def handle_leaf_detection_request(self, request, response):
        """Handle leaf detection service request"""
        try:
            result = await self.detector.handle_request(request)
            
            response.coordinates = result.get('coordinates', [])
            response.num_leaves = result.get('num_leaves', 0)
            response.success = result.get('success', False)
            response.message = result.get('message', '')
            
            # Include additional info in debug_info as JSON
            debug_data = {
                'debug_info': result.get('debug_info', ''),
                'has_yellow_tape': result.get('has_yellow_tape', []),
                'yellow_ratio': result.get('yellow_ratio', []),
                'health_status': result.get('health_status', [])
            }
            try:
                response.debug_info = json.dumps(debug_data)
            except Exception:
                response.debug_info = result.get('debug_info', '')
            
            # Visualization is now handled by independent leaf_visualization_node
                
            return response
            
        except Exception as e:
            self.get_logger().error(f'✗ Service request error: {str(e)}')
            response.success = False
            response.message = f"Service error: {str(e)}"
            response.coordinates = []
            response.num_leaves = 0
            response.debug_info = ''
            return response


def main():
    rclpy.init()
    server = LeafDetectionServer()
    
    # Use multi-threaded executor for concurrent processing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    
    # Start the executor in a background thread
    def spin_executor():
        executor.spin()
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(server, timeout_sec=0.1)
    except KeyboardInterrupt:
        server.get_logger().info("Shutting down leaf detection server")
    finally:
        server.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()

