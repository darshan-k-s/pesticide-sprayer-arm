#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Visualization Node
Independent node for visualizing leaf detection results
Subscribes to annotated images and publishes visualization markers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import json
from cv_bridge import CvBridge


class LeafVisualizationNode(Node):
    """Independent visualization node for leaf detection results"""
    
    def __init__(self):
        super().__init__('leaf_visualization_node')
        
        self.bridge = CvBridge()
        
        # Coordinate smoothing parameters
        self.smoothed_positions = {}  # {marker_id: [x, y, z]}
        self.smoothing_factor = 0.7  # 0-1, larger value = smoother (but slower response)
        self.min_update_threshold = 0.01  # Minimum update threshold (meters), skip updates below this value
        self.marker_counter = 0  # For generating stable marker IDs
        
        # Update frequency limiting
        self.last_update_time = None
        self.min_update_interval = 0.1  # Minimum update interval (seconds), approximately 10Hz
        
        # Subscribe to annotated image (for monitoring)
        self.annotated_image_sub = self.create_subscription(
            Image,
            '/leaf_detection/annotated_image',
            self.annotated_image_callback,
            10
        )
        
        # Subscribe to detection results (for marker visualization)
        self.detection_results_sub = self.create_subscription(
            String,
            '/leaf_detection/detection_results',
            self.detection_results_callback,
            10
        )
        
        # Publish RViz markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/leaf_detection/leaf_markers',
            10
        )
        
        self.get_logger().info('🎨 Leaf Visualization Node started')
        self.get_logger().info('📡 Subscribed to /leaf_detection/annotated_image')
        self.get_logger().info('📡 Subscribed to /leaf_detection/detection_results')
        self.get_logger().info('📡 Publishing to /leaf_detection/leaf_markers')
        self.get_logger().info('💡 Visualize in RViz by subscribing to these topics')
        self.get_logger().info('✨ Coordinate smoothing enabled (smoothing_factor=0.7, min_update=10Hz)')
    
    def annotated_image_callback(self, msg):
        """Handle annotated image messages (for logging/monitoring)"""
        try:
            self.get_logger().debug('Received annotated image')
        except Exception as e:
            self.get_logger().error(f'✗ Error processing annotated image: {str(e)}')
    
    def detection_results_callback(self, msg):
        """Handle detection results and update markers"""
        try:
            # Frequency limiting: only update when minimum interval is exceeded
            now = self.get_clock().now()
            if self.last_update_time is not None:
                time_diff = (now - self.last_update_time).nanoseconds / 1e9
                if time_diff < self.min_update_interval:
                    return  # Skip this update
            
            data = json.loads(msg.data)
            # Get coordinates (may include both camera and base coordinates)
            leaf_coordinates = data.get('coordinates', [])
            
            self.get_logger().debug(f'Received detection results: {len(leaf_coordinates)} leaves')
            
            # If base_coordinates exist, merge them into leaf_coordinates
            base_coordinates = data.get('base_coordinates', [])
            if base_coordinates and len(base_coordinates) > 0:
                self.get_logger().debug(f'Found {len(base_coordinates)} base_coordinates')
                # Merge base coordinates with camera coordinates
                for i, (leaf, base_coord) in enumerate(zip(leaf_coordinates, base_coordinates)):
                    if base_coord and isinstance(base_coord, dict):
                        leaf['base_coordinates'] = [base_coord.get('x', 0), base_coord.get('y', 0), base_coord.get('z', 0)]
                        self.get_logger().debug(f'Leaf {i}: Added base_coordinates {leaf["base_coordinates"]}')
                    elif base_coord and isinstance(base_coord, (list, tuple)) and len(base_coord) >= 3:
                        leaf['base_coordinates'] = list(base_coord)[:3]
                        self.get_logger().debug(f'Leaf {i}: Added base_coordinates (list format) {leaf["base_coordinates"]}')
            else:
                self.get_logger().debug('No base_coordinates found, will use camera frame coordinates')
            
            self.last_update_time = now
            self.update_markers(leaf_coordinates)
        except Exception as e:
            self.get_logger().error(f'✗ Error processing detection results: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def update_markers(self, leaf_coordinates):
        """
        Update RViz markers based on leaf coordinates with smoothing
        
        Args:
            leaf_coordinates: List of leaf coordinate dictionaries
        """
        if not leaf_coordinates:
            # Clear all markers
            marker_array = MarkerArray()
            # Delete all existing markers
            for marker_id in list(self.smoothed_positions.keys()):
                delete_marker = Marker()
                delete_marker.header.frame_id = "base_link"  # Use common frame
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.ns = "leaf_detections"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
            
            if marker_array.markers:
                self.marker_pub.publish(marker_array)
            
            # Clear smoothed positions
            self.smoothed_positions.clear()
            self.marker_counter = 0
            return
        
        try:
            marker_array = MarkerArray()
            current_marker_ids = set()
            
            for i, leaf in enumerate(leaf_coordinates):
                # Try to use base coordinates if available, otherwise use camera coordinates
                base_coords = leaf.get('base_coordinates')
                point_3d = leaf.get('point_3d')
                
                if base_coords and len(base_coords) >= 3:
                    # Use base frame coordinates
                    raw_x, raw_y, raw_z = float(base_coords[0]), float(base_coords[1]), float(base_coords[2])
                    frame_id = "base_link"  # Use base_link instead of base
                elif point_3d is not None:
                    # Fallback to camera frame coordinates
                    raw_x, raw_y, raw_z = float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
                    frame_id = "camera_color_optical_frame"
                else:
                    continue
                
                # Use stable marker ID (based on position, rounded to centimeters)
                # This ensures leaves at the same location use the same ID
                pos_key = (round(raw_x * 100), round(raw_y * 100), round(raw_z * 100))
                
                # Generate or get marker ID
                if pos_key not in self.smoothed_positions:
                    marker_id = self.marker_counter
                    self.marker_counter += 1
                    # Initialize smoothed position
                    self.smoothed_positions[pos_key] = {'id': marker_id, 'pos': [raw_x, raw_y, raw_z], 'frame': frame_id}
                else:
                    marker_id = self.smoothed_positions[pos_key]['id']
                    frame_id = self.smoothed_positions[pos_key]['frame']  # Keep the same frame
                
                current_marker_ids.add(pos_key)
                
                # Apply smoothing filter
                old_pos = self.smoothed_positions[pos_key]['pos']
                smoothed_x = old_pos[0] * self.smoothing_factor + raw_x * (1 - self.smoothing_factor)
                smoothed_y = old_pos[1] * self.smoothing_factor + raw_y * (1 - self.smoothing_factor)
                smoothed_z = old_pos[2] * self.smoothing_factor + raw_z * (1 - self.smoothing_factor)
                
                # Check if update is needed (change exceeds threshold)
                change = ((smoothed_x - old_pos[0])**2 + 
                         (smoothed_y - old_pos[1])**2 + 
                         (smoothed_z - old_pos[2])**2)**0.5
                
                if change < self.min_update_threshold:
                    # Change too small, use old position
                    x, y, z = old_pos
                else:
                    # Update position
                    self.smoothed_positions[pos_key]['pos'] = [smoothed_x, smoothed_y, smoothed_z]
                    x, y, z = smoothed_x, smoothed_y, smoothed_z
                
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "leaf_detections"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.05  # 5cm radius
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                marker.lifetime.sec = 0  # 0 = permanent, until deleted
                
                marker_array.markers.append(marker)
            
            # Delete markers that no longer exist
            markers_to_delete = []
            for pos_key in list(self.smoothed_positions.keys()):
                if pos_key not in current_marker_ids:
                    marker_id = self.smoothed_positions[pos_key]['id']
                    frame_id = self.smoothed_positions[pos_key]['frame']
                    
                    delete_marker = Marker()
                    delete_marker.header.frame_id = frame_id
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.ns = "leaf_detections"
                    delete_marker.id = marker_id
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
                    
                    markers_to_delete.append(pos_key)
            
            # Remove deleted markers from dictionary
            for pos_key in markers_to_delete:
                del self.smoothed_positions[pos_key]
            
            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)
                self.get_logger().debug(f'Published {len([m for m in marker_array.markers if m.action == Marker.ADD])} markers')
            else:
                self.get_logger().warn('No valid coordinates found for markers')
            
        except Exception as e:
            self.get_logger().error(f'✗ RViz visualization error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LeafVisualizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

