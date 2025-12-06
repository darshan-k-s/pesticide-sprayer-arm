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
        
        # Coordinate smoothing parameters for leaves
        self.smoothed_positions = {}  # {marker_id: [x, y, z]}
        self.smoothing_factor = 0.7  # 0-1, larger value = smoother (but slower response)
        self.min_update_threshold = 0.01  # Minimum update threshold (meters), skip updates below this value
        self.marker_counter = 0  # For generating stable marker IDs
        
        # Blue box visualization parameters
        self.smoothed_blue_box_positions = {}  # {pos_key: {'pos': [x, y, z], 'size': [w, h, d], 'id': marker_id, 'frame': frame_id, 'box_id': box_id}}
        self.blue_box_marker_counter = 1000  # Start from 1000 to avoid conflicts with leaf markers
        self.all_blue_box_marker_ids = set()  # Track all marker IDs that have been created
        self.last_blue_box_count = 0  # Track last detected box count for change detection
        self.position_match_threshold = 0.2  # Position matching threshold in meters (20cm)
        
        # Update frequency limiting
        self.last_update_time = None
        self.min_update_interval = 0.1  # Minimum update interval (seconds), approximately 10Hz
        self.last_blue_box_update_time = None
        self.min_blue_box_update_interval = 0.5  # Blue box update interval (seconds), 2Hz to reduce RViz load during planning
        
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
        
        # Subscribe to blue box results (for blue box visualization)
        self.blue_box_results_sub = self.create_subscription(
            String,
            '/leaf_detection/blue_box_results',
            self.blue_box_results_callback,
            10
        )
        
        # Publish RViz markers (for both leaves and blue boxes)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/leaf_detection/leaf_markers',
            10
        )
        
        # Publish blue box markers separately
        self.blue_box_marker_pub = self.create_publisher(
            MarkerArray,
            '/leaf_detection/blue_box_markers',
            10
        )
        
        self.get_logger().info('Leaf Visualization Node started')
        self.get_logger().info('Subscribed to /leaf_detection/annotated_image')
        self.get_logger().info('Subscribed to /leaf_detection/detection_results')
        self.get_logger().info('Subscribed to /leaf_detection/blue_box_results')
        self.get_logger().info('Publishing to /leaf_detection/leaf_markers')
        self.get_logger().info('Publishing to /leaf_detection/blue_box_markers')
        self.get_logger().info(' Visualize in RViz by subscribing to these topics')
        self.get_logger().info('Coordinate smoothing enabled (smoothing_factor=0.7, min_update=10Hz)')
    
    def annotated_image_callback(self, msg):
        """Handle annotated image messages (for logging/monitoring)"""
        try:
            self.get_logger().debug('Received annotated image')
        except Exception as e:
            self.get_logger().error(f' Error processing annotated image: {str(e)}')
    
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
            
            # Also check for blue boxes in detection results (may have converted coordinates)
            blue_boxes = data.get('blue_boxes', [])
            if blue_boxes and len(blue_boxes) > 0:
                self.get_logger().debug(f'Found {len(blue_boxes)} blue boxes in detection results')
                self.update_blue_box_markers(blue_boxes)
            
            self.last_update_time = now
            self.update_markers(leaf_coordinates)
        except Exception as e:
            self.get_logger().error(f' Error processing detection results: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def blue_box_results_callback(self, msg):
        """Handle blue box detection results and update markers"""
        try:
            # Frequency limiting: use longer interval for blue boxes to reduce RViz load
            # This is especially important during path planning when RViz is rendering many things
            now = self.get_clock().now()
            if self.last_blue_box_update_time is not None:
                time_diff = (now - self.last_blue_box_update_time).nanoseconds / 1e9
                if time_diff < self.min_blue_box_update_interval:
                    return  # Skip this update to reduce RViz load
            
            data = json.loads(msg.data)
            blue_boxes = data.get('blue_boxes', [])
            
            self.get_logger().debug(f'Received blue box results: {len(blue_boxes)} boxes')
            
            self.last_blue_box_update_time = now
            self.update_blue_box_markers(blue_boxes)
        except Exception as e:
            self.get_logger().error(f' Error processing blue box results: {str(e)}')
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
            for pos_key, marker_info in list(self.smoothed_positions.items()):
                marker_id = int(marker_info.get('id', 0))
                frame_id = marker_info.get('frame', 'base_link')
                delete_marker = Marker()
                delete_marker.header.frame_id = frame_id  # Use stored frame
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
                
                has_yellow = bool(leaf.get('has_yellow_tape', False))
                if has_yellow:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
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
            self.get_logger().error(f' RViz visualization error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def update_blue_box_markers(self, blue_boxes):
        """
        Update RViz markers for blue boxes (obstacles)
        Simplified: All boxes are treated as obstacles, no box_id distinction
        Aggressive cleanup: Delete all unmatched markers to prevent residues
        
        Args:
            blue_boxes: List of blue box dictionaries
        """
        marker_array = MarkerArray()
        
        if not blue_boxes:
            # No boxes detected: delete ALL existing markers
            for pos_key, box_info in list(self.smoothed_blue_box_positions.items()):
                marker_id = int(box_info.get('id', 0))
                frame_id = box_info.get('frame', 'base_link')
                
                delete_marker = Marker()
                delete_marker.header.frame_id = frame_id
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.ns = "blue_boxes"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
                
                if marker_id in self.all_blue_box_marker_ids:
                    self.all_blue_box_marker_ids.remove(marker_id)
            
            if marker_array.markers:
                self.blue_box_marker_pub.publish(marker_array)
                self.get_logger().info(f'Deleted {len(marker_array.markers)} blue box markers (no boxes detected)')
            
            self.smoothed_blue_box_positions.clear()
            self.all_blue_box_marker_ids.clear()
            self.last_blue_box_count = 0
            return
        
        try:
            current_box_count = len(blue_boxes)
            
            # If detected box count changed, more aggressive cleanup (prevent residue when two boxes merge)
            box_count_changed = (current_box_count != self.last_blue_box_count)
            
            # Match current boxes to existing positions (by position only, no box_id)
            matched_positions = set()
            position_to_box_map = {}  # Map from position key to box index
            
            # First pass: Match boxes by position proximity
            for box_idx, box in enumerate(blue_boxes):
                point_3d = box.get('point_3d')
                if point_3d is None:
                    continue
                
                if isinstance(point_3d, (list, tuple)):
                    raw_x, raw_y, raw_z = float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
                else:
                    continue
                
                # Try to find matching existing position
                best_match = None
                best_distance = float('inf')
                
                for existing_pos_key, old_info in self.smoothed_blue_box_positions.items():
                    if existing_pos_key in matched_positions:
                        continue
                    
                    old_pos = old_info.get('pos', [0, 0, 0])
                    distance = ((raw_x - old_pos[0])**2 + 
                               (raw_y - old_pos[1])**2 + 
                               (raw_z - old_pos[2])**2)**0.5
                    
                    # If box count changed, use stricter matching threshold
                    match_threshold = self.position_match_threshold * 0.5 if box_count_changed else self.position_match_threshold
                    
                    if distance < match_threshold and distance < best_distance:
                        best_match = existing_pos_key
                        best_distance = distance
                
                if best_match is not None:
                    # Reuse existing position
                    matched_positions.add(best_match)
                    position_to_box_map[best_match] = box_idx
                else:
                    # New position - create new key
                    pos_key = f"obstacle_{int(raw_x*20)}_{int(raw_y*20)}_{int(raw_z*20)}"
                    position_to_box_map[pos_key] = box_idx
            
            # If box count decreased (e.g., two boxes merged into one), delete all old markers and recreate
            if box_count_changed and current_box_count < self.last_blue_box_count:
                # Delete all existing markers
                for pos_key, box_info in list(self.smoothed_blue_box_positions.items()):
                    marker_id = int(box_info.get('id', 0))
                    frame_id = box_info.get('frame', 'base_link')
                    
                    delete_marker = Marker()
                    delete_marker.header.frame_id = frame_id
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.ns = "blue_boxes"
                    delete_marker.id = marker_id
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
                    
                    if marker_id in self.all_blue_box_marker_ids:
                        self.all_blue_box_marker_ids.remove(marker_id)
                
                # Clear tracking dictionary, force recreation of all markers
                self.smoothed_blue_box_positions.clear()
                matched_positions.clear()
                position_to_box_map = {}
                
                # Recreate position mapping
                for box_idx, box in enumerate(blue_boxes):
                    point_3d = box.get('point_3d')
                    if point_3d is None:
                        continue
                    
                    if isinstance(point_3d, (list, tuple)):
                        raw_x, raw_y, raw_z = float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
                        pos_key = f"obstacle_{int(raw_x*20)}_{int(raw_y*20)}_{int(raw_z*20)}"
                        position_to_box_map[pos_key] = box_idx
            
            # Delete ALL unmatched old markers (aggressive cleanup to prevent residues)
            for pos_key in list(self.smoothed_blue_box_positions.keys()):
                if pos_key not in matched_positions:
                    box_info = self.smoothed_blue_box_positions[pos_key]
                    marker_id = int(box_info.get('id', 0))
                    frame_id = box_info.get('frame', 'base_link')
                    
                    delete_marker = Marker()
                    delete_marker.header.frame_id = frame_id
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.ns = "blue_boxes"
                    delete_marker.id = marker_id
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
                    
                    if marker_id in self.all_blue_box_marker_ids:
                        self.all_blue_box_marker_ids.remove(marker_id)
                    
                    # Remove from tracking immediately
                    del self.smoothed_blue_box_positions[pos_key]
            
            # Create/update markers for all current boxes
            for pos_key, box_idx in position_to_box_map.items():
                box = blue_boxes[box_idx]
                point_3d = box.get('point_3d')
                size_3d = box.get('size_3d')
                
                if point_3d is None or size_3d is None:
                    continue
                
                if isinstance(point_3d, (list, tuple)):
                    raw_x, raw_y, raw_z = float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
                else:
                    continue
                
                width = float(size_3d.get('width', 0.1))
                height = float(size_3d.get('height', 0.1))
                depth = float(size_3d.get('depth', 0.1))
                
                frame_id = "base_link"
                
                # Generate or get marker ID
                if pos_key not in self.smoothed_blue_box_positions:
                    marker_id = self.blue_box_marker_counter
                    self.blue_box_marker_counter += 1
                    self.smoothed_blue_box_positions[pos_key] = {
                        'id': marker_id,
                        'pos': [raw_x, raw_y, raw_z],
                        'size': [width, height, depth],
                        'frame': frame_id
                    }
                    self.all_blue_box_marker_ids.add(marker_id)
                    x, y, z = raw_x, raw_y, raw_z
                    w, h, d = width, height, depth
                else:
                    marker_id = self.smoothed_blue_box_positions[pos_key]['id']
                    frame_id = self.smoothed_blue_box_positions[pos_key]['frame']
                    
                    # Update position and size with smoothing
                    old_pos = self.smoothed_blue_box_positions[pos_key]['pos']
                    
                    # Calculate distance moved
                    distance_moved = ((raw_x - old_pos[0])**2 + 
                                    (raw_y - old_pos[1])**2 + 
                                    (raw_z - old_pos[2])**2)**0.5
                    
                    if distance_moved > self.position_match_threshold:
                        # Large movement: fast response (less smoothing)
                        fast_smoothing = 0.2
                        smoothed_x = old_pos[0] * fast_smoothing + raw_x * (1 - fast_smoothing)
                        smoothed_y = old_pos[1] * fast_smoothing + raw_y * (1 - fast_smoothing)
                        smoothed_z = old_pos[2] * fast_smoothing + raw_z * (1 - fast_smoothing)
                    else:
                        # Small movement: normal smoothing
                        smoothed_x = old_pos[0] * self.smoothing_factor + raw_x * (1 - self.smoothing_factor)
                        smoothed_y = old_pos[1] * self.smoothing_factor + raw_y * (1 - self.smoothing_factor)
                        smoothed_z = old_pos[2] * self.smoothing_factor + raw_z * (1 - self.smoothing_factor)
                    
                    old_size = self.smoothed_blue_box_positions[pos_key]['size']
                    size_smoothing = 0.8
                    smoothed_w = old_size[0] * size_smoothing + width * (1 - size_smoothing)
                    smoothed_h = old_size[1] * size_smoothing + height * (1 - size_smoothing)
                    smoothed_d = old_size[2] * size_smoothing + depth * (1 - size_smoothing)
                    
                    # Update stored position and size
                    self.smoothed_blue_box_positions[pos_key]['pos'] = [smoothed_x, smoothed_y, smoothed_z]
                    self.smoothed_blue_box_positions[pos_key]['size'] = [smoothed_w, smoothed_h, smoothed_d]
                    x, y, z = smoothed_x, smoothed_y, smoothed_z
                    w, h, d = smoothed_w, smoothed_h, smoothed_d
                
                # Create obstacle marker (blue)
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "blue_boxes"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Set box dimensions
                marker.scale.x = max(w, 0.01)
                marker.scale.y = max(h, 0.01)
                marker.scale.z = max(d, 0.01)
                
                # Blue color for obstacles (RGB: 0, 0, 1) - bright blue
                # Following leaf display style, uniformly displayed as blue
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.8  # Same transparency as leaves
                
                marker.lifetime.sec = 0  # Permanent until deleted
                
                marker_array.markers.append(marker)
            
            self.last_blue_box_count = current_box_count
            
            # Only publish if there are markers to publish
            # Reduce publishing frequency during path planning to prevent RViz freezing
            if len(marker_array.markers) > 0:
                # Track markers before adding new ones to detect new boxes
                existing_marker_ids_before = self.all_blue_box_marker_ids.copy()
                
                # Throttle publishing: only publish if significant changes or enough time has passed
                should_publish = True
                if len(marker_array.markers) > 5:  # Multiple markers, be more conservative
                    # Check if this is just position updates (ADD actions with existing IDs)
                    num_new = len([m for m in marker_array.markers 
                                  if m.action == Marker.ADD and m.id not in existing_marker_ids_before])
                    num_deletes = len([m for m in marker_array.markers if m.action == Marker.DELETE])
                    # Only publish if there are new boxes or deletions, or if enough time passed
                    if num_new == 0 and num_deletes == 0:
                        # This is just position updates, check time since last publish
                        if hasattr(self, '_last_blue_box_publish_time'):
                            time_since_publish = (self.get_clock().now() - self._last_blue_box_publish_time).nanoseconds / 1e9
                            if time_since_publish < 1.0:  # Don't publish position updates more than 1Hz
                                should_publish = False
                
                if should_publish:
                    self.blue_box_marker_pub.publish(marker_array)
                    if not hasattr(self, '_last_blue_box_publish_time'):
                        self._last_blue_box_publish_time = self.get_clock().now()
                    else:
                        self._last_blue_box_publish_time = self.get_clock().now()
                    num_add = len([m for m in marker_array.markers if m.action == Marker.ADD])
                    num_delete = len([m for m in marker_array.markers if m.action == Marker.DELETE])
                    if num_delete > 0 or num_add > 0:
                        self.get_logger().debug(
                            f'Blue boxes (obstacles): {num_add} active, {num_delete} deleted'
                        )
            
        except Exception as e:
            self.get_logger().error(f' Blue box RViz visualization error: {str(e)}')
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

