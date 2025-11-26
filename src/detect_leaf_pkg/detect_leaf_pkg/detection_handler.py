#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Detection Handler
Handles leaf detection using PlantCV library
"""

import cv2
import numpy as np
from plantcv import plantcv as pcv
from cv_bridge import CvBridge
from datetime import datetime
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image


class DetectionHandler:
    """Handles leaf detection logic using PlantCV"""
    
    def __init__(self, node, tf_handler=None):
        self.node = node
        self.tf_handler = tf_handler
        self.bridge = CvBridge()
        
        # PlantCV settings
        pcv.params.debug = None  # Disable debug output
        
        # Current frame data
        self.current_frame = None
        self.current_depth = None
        
        # Latest detection results (for service to use)
        self.latest_leaf_data = None
        self.latest_coordinates_base = None
        self.latest_leaf_attributes = {
            'has_yellow_tape': [],
            'yellow_ratio': [],
            'health_status': []
        }
        import threading
        self.latest_data_lock = threading.Lock()
        
        # Track published blue box IDs for cleanup
        self.published_blue_box_ids = set()  # Track which box IDs have been published to MoveIt
        
        # Detection mode: continuous or on-demand
        self.continuous_detection = node.declare_parameter('continuous_detection', True).value
        self.min_area_default = 2000.0  # Default min area
        self.detect_yellow_tape = node.declare_parameter('detect_yellow_tape', True).value
        self.yellow_ratio_threshold = node.declare_parameter('yellow_ratio_threshold', 0.05).value  # Lower threshold to 0.05 (5%) for easier yellow tape detection
        # HSV color range parameters (configurable, stricter defaults for detecting real yellow tape)
        # Yellow tape is usually bright yellow with high saturation and brightness
        self.yellow_hsv_lower = node.declare_parameter('yellow_hsv_lower', [20, 100, 100]).value
        self.yellow_hsv_upper = node.declare_parameter('yellow_hsv_upper', [30, 255, 255]).value
        
        # Blue box detection parameters
        self.detect_blue_box = node.declare_parameter('detect_blue_box', True).value
        self.blue_min_area = node.declare_parameter('blue_min_area', 3000.0).value
        # Blue in HSV: H value 100-130 (blue range), high S and V values
        # Using adjusted parameters: S Lower=147 to reduce false positives
        self.blue_hsv_lower = node.declare_parameter('blue_hsv_lower', [100, 147, 50]).value
        self.blue_hsv_upper = node.declare_parameter('blue_hsv_upper', [130, 255, 255]).value
        
        # Publisher for detection results (for visualization node to subscribe)
        self.detection_results_pub = node.create_publisher(
            String,
            '/leaf_detection/detection_results',
            10
        )
        
        # Publisher for annotated image (continuously published)
        self.annotated_image_pub = node.create_publisher(
            Image,
            '/leaf_detection/annotated_image',
            10
        )
        
        self.healthy_leaves_pub = node.create_publisher(
            String,
            '/leaf_detection/healthy_leaves',
            10
        )
        self.unhealthy_leaves_pub = node.create_publisher(
            String,
            '/leaf_detection/unhealthy_leaves',
            10
        )
        
        # Publisher for blue box detection results
        self.blue_box_results_pub = node.create_publisher(
            String,
            '/leaf_detection/blue_box_results',
            10
        )
        
        # Publisher for MoveIt CollisionObjects (for dynamic_obstacles_monitor)
        from moveit_msgs.msg import CollisionObject
        self.moveit_collision_pub = node.create_publisher(
            CollisionObject,
            '/obsFromImg',
            10
        )
        
        self.get_logger().info('✓ DetectionHandler initialized')
        if self.continuous_detection:
            self.get_logger().info('📡 Continuous detection mode: images will be published continuously')
        if self.detect_yellow_tape:
            self.get_logger().info(
                f'🎯 Yellow tape detection enabled (threshold={self.yellow_ratio_threshold:.3f}, '
                f'HSV range: {self.yellow_hsv_lower} - {self.yellow_hsv_upper})'
            )
        if self.detect_blue_box:
            self.get_logger().info(
                f'📦 Blue box detection enabled (min_area={self.blue_min_area}, '
                f'HSV range: {self.blue_hsv_lower} - {self.blue_hsv_upper})'
            )
    
    def get_logger(self):
        """Get node logger"""
        return self.node.get_logger()
    
    def update_frames(self, color_msg, depth_msg):
        """Update current frame and depth from synchronized messages, and perform continuous detection"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            # Continuous detection mode: detect and publish on every frame
            if self.continuous_detection:
                self._perform_continuous_detection(color_msg.header)
        except Exception as e:
            self.get_logger().error(f'✗ Image conversion failed: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _perform_continuous_detection(self, header):
        """Perform detection on current frame and publish results continuously"""
        # Frame counter for debugging
        if not hasattr(self, '_frame_count'):
            self._frame_count = 0
        self._frame_count += 1
        
        if self.current_frame is None:
            self.get_logger().warn(f'Frame {self._frame_count}: current_frame is None')
            return
        
        if self.current_depth is None:
            self.get_logger().warn(f'Frame {self._frame_count}: current_depth is None, skipping detection')
            return
        
        try:
            # Perform detection with default parameters
            detection_result, leaf_data, bounding_boxes = self.detect_leaves_with_plantcv(
                self.current_frame,
                min_area=self.min_area_default
            )
            
            # Log detection status periodically
            if self._frame_count % 30 == 0:
                if leaf_data and leaf_data.get('num_leaves', 0) > 0:
                    self.get_logger().info(f'Frame {self._frame_count}: Detected {leaf_data["num_leaves"]} leaves, publishing image...')
                else:
                    self.get_logger().debug(f'Frame {self._frame_count}: No leaves detected, publishing original image...')
            
            # Update latest results (thread-safe)
            with self.latest_data_lock:
                if leaf_data and leaf_data.get('num_leaves', 0) > 0:
                    self.latest_leaf_data = leaf_data
                    # Also compute base coordinates for latest data
                    self.latest_coordinates_base = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
                    has_yellow, yellow_ratio, health_status = self._extract_leaf_attributes(
                        leaf_data.get('coordinates', [])
                    )
                    self.latest_leaf_attributes = {
                        'has_yellow_tape': has_yellow,
                        'yellow_ratio': yellow_ratio,
                        'health_status': health_status
                    }
                    
                    # Log base frame coordinates periodically
                    if self._frame_count % 30 == 0 and self.latest_coordinates_base:
                        self.get_logger().info('📍 Latest base coordinates:')
                        for i, point in enumerate(self.latest_coordinates_base):
                            self.get_logger().info(f'  Leaf {i+1}: X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m')
                else:
                    self.latest_leaf_data = None
                    self.latest_coordinates_base = []
                    self.latest_leaf_attributes = {
                        'has_yellow_tape': [],
                        'yellow_ratio': [],
                        'health_status': []
                    }
            
            # Publish health status summary
            self._publish_health_status(leaf_data)
            
            # Publish detection results to topic (for visualization node)
            if leaf_data:
                # Convert to base coordinates if tf_handler is available
                base_coords_list = []
                if self.tf_handler:
                    base_coords_msg = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
                    # Convert Point messages to dict for JSON serialization
                    for point_msg in base_coords_msg:
                        base_coords_list.append({
                            'x': point_msg.x,
                            'y': point_msg.y,
                            'z': point_msg.z
                        })
                
                # Clean blue boxes for JSON serialization (remove numpy arrays)
                blue_boxes_clean = self._clean_blue_boxes_for_json(leaf_data.get('blue_boxes', []))
                
                results_json = {
                    'num_leaves': leaf_data['num_leaves'],
                    'timestamp': leaf_data.get('timestamp', ''),
                    'coordinates': leaf_data.get('coordinates', []),  # Camera frame coordinates
                    'base_coordinates': base_coords_list if base_coords_list else [],  # Base frame coordinates
                    'blue_boxes': blue_boxes_clean  # Blue box coordinates (cleaned for JSON)
                }
                results_msg = String()
                results_msg.data = json.dumps(results_json)
                self.detection_results_pub.publish(results_msg)
                
                # Publish blue box results separately
                blue_boxes = leaf_data.get('blue_boxes', [])
                if len(blue_boxes) > 0:
                    blue_box_json = {
                        'num_blue_boxes': len(blue_boxes),
                        'timestamp': leaf_data.get('timestamp', ''),
                        'blue_boxes': blue_boxes_clean
                    }
                    blue_box_msg = String()
                    blue_box_msg.data = json.dumps(blue_box_json)
                    self.blue_box_results_pub.publish(blue_box_msg)
                    
                    # Publish blue boxes as MoveIt CollisionObjects to /obsFromImg
                    self._publish_blue_boxes_to_moveit(blue_boxes)
                else:
                    # No blue boxes detected: remove all previously published boxes
                    self._publish_blue_boxes_to_moveit([])
            else:
                # Publish empty results if no leaves detected
                results_json = {
                    'num_leaves': 0,
                    'timestamp': datetime.now().isoformat(),
                    'coordinates': [],
                    'blue_boxes': []
                }
                results_msg = String()
                results_msg.data = json.dumps(results_json)
                self.detection_results_pub.publish(results_msg)
            
            # Publish annotated image (with or without detections)
            annotated_image = self.draw_annotations(
                self.current_frame, 
                leaf_data.get('coordinates', []) if leaf_data else [],
                leaf_data.get('blue_boxes', []) if leaf_data else []
            )
            if annotated_image is not None:
                try:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                    annotated_msg.header = header
                    self.annotated_image_pub.publish(annotated_msg)
                except Exception as e:
                    self.get_logger().error(f'✗ Error publishing annotated image: {str(e)}')
                    import traceback
                    self.get_logger().error(traceback.format_exc())
                    
        except Exception as e:
            self.get_logger().error(f'✗ Continuous detection error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            # Still publish original image even if detection fails
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(self.current_frame, "bgr8")
                annotated_msg.header = header
                self.annotated_image_pub.publish(annotated_msg)
            except:
                pass
    
    def _publish_health_status(self, leaf_data):
        """Publish healthy/unhealthy leaf summaries as separate topics"""
        healthy_leaves = []
        unhealthy_leaves = []
        
        if leaf_data and leaf_data.get('coordinates'):
            for leaf in leaf_data['coordinates']:
                is_unhealthy = bool(leaf.get('has_yellow_tape', False))
                payload = {
                    'id': leaf.get('id'),
                    'center': leaf.get('center', {}),
                    'bounding_box': leaf.get('bounding_box', {}),
                    'yellow_ratio': leaf.get('yellow_ratio', 0.0),
                    'health_status': 'unhealthy' if is_unhealthy else 'healthy'
                }
                if is_unhealthy:
                    unhealthy_leaves.append(payload)
                else:
                    healthy_leaves.append(payload)
        else:
            # No leaves detected, publish empty payloads
            healthy_leaves = []
            unhealthy_leaves = []
        
        healthy_msg = String()
        healthy_msg.data = json.dumps({
            'timestamp': datetime.now().isoformat(),
            'num_leaves': len(healthy_leaves),
            'leaves': healthy_leaves
        })
        unhealthy_msg = String()
        unhealthy_msg.data = json.dumps({
            'timestamp': datetime.now().isoformat(),
            'num_leaves': len(unhealthy_leaves),
            'leaves': unhealthy_leaves
        })
        
        self.healthy_leaves_pub.publish(healthy_msg)
        self.unhealthy_leaves_pub.publish(unhealthy_msg)
    
    def _convert_to_base_coordinates(self, leaf_coordinates):
        """Convert leaf coordinates to base frame"""
        coordinates = []
        from geometry_msgs.msg import Point
        
        if self.tf_handler:
            for i, leaf in enumerate(leaf_coordinates):
                point_3d = leaf.get('point_3d')
                if point_3d is not None:
                    # Log original camera optical frame coordinates
                    self.get_logger().debug(
                        f'Leaf {i+1} - Camera optical frame: '
                        f'({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})'
                    )
                    
                    base_coords = self.tf_handler.camera_to_base(point_3d)
                    if base_coords:
                        self.get_logger().debug(
                            f'Leaf {i+1} - Base_link frame (after TF + offsets): '
                            f'({base_coords[0]:.3f}, {base_coords[1]:.3f}, {base_coords[2]:.3f})'
                        )
                        
                        point_msg = Point()
                        point_msg.x = float(base_coords[0])
                        point_msg.y = float(base_coords[1])
                        point_msg.z = float(base_coords[2])
                        coordinates.append(point_msg)
                    else:
                        self.get_logger().warn(f'Leaf {i+1} - Failed to convert to base coordinates')
        else:
            for leaf in leaf_coordinates:
                point_3d = leaf.get('point_3d')
                if point_3d is not None:
                    point_msg = Point()
                    point_msg.x = float(point_3d[0])
                    point_msg.y = float(point_3d[1])
                    point_msg.z = float(point_3d[2])
                    coordinates.append(point_msg)
        
        return coordinates
    
    def _extract_leaf_attributes(self, leaf_records):
        """Extract yellow tape indicators and health labels from detection results"""
        has_yellow = []
        yellow_ratio = []
        health_status = []
        for leaf in leaf_records or []:
            has_yellow.append(bool(leaf.get('has_yellow_tape', False)))
            yellow_ratio.append(float(leaf.get('yellow_ratio', 0.0)))
            health_status.append(str(leaf.get('health_status', 'unknown')))
        return has_yellow, yellow_ratio, health_status
    
    def _clean_blue_boxes_for_json(self, blue_boxes):
        """Clean blue box data for JSON serialization (remove numpy arrays)"""
        cleaned = []
        for box in blue_boxes:
            cleaned_box = {
                'id': box.get('id'),
                'type': box.get('type'),
                'num_faces': box.get('num_faces', 1),
                'center': box.get('center', {}),
                'bounding_box': box.get('bounding_box', {}),
                'area': box.get('area', 0.0),
                'depth_mm': box.get('depth_mm', 0.0),
                'point_3d': list(box.get('point_3d')) if box.get('point_3d') is not None else None,
                'size_3d': box.get('size_3d')
            }
            # Remove 'faces' field as it contains numpy arrays (contours)
            cleaned.append(cleaned_box)
        return cleaned
    
    def _publish_blue_boxes_to_moveit(self, blue_boxes):
        """Publish blue boxes as MoveIt CollisionObjects to /obsFromImg"""
        from moveit_msgs.msg import CollisionObject
        from std_msgs.msg import Header
        
        # Get current box IDs
        current_box_ids = set()
        
        for box in blue_boxes:
            point_3d = box.get('point_3d')
            size_3d = box.get('size_3d')
            
            if point_3d is None or size_3d is None:
                continue
            
            box_id = f"blue_box_{box['id']:03d}"
            current_box_ids.add(box_id)
            
            # Convert camera frame coordinates to base frame if tf_handler is available
            base_point_3d = point_3d
            if self.tf_handler:
                base_coords = self.tf_handler.camera_to_base(point_3d)
                if base_coords:
                    base_point_3d = base_coords
                else:
                    self.get_logger().warn(f'Failed to convert blue box {box.get("id")} to base frame, removing old marker')
                    # If conversion fails, remove the old marker if it exists
                    if box_id in self.published_blue_box_ids:
                        self._remove_blue_box_from_moveit(box_id)
                        self.published_blue_box_ids.discard(box_id)
                    continue
            
            # Create CollisionObject message
            collision_obj = CollisionObject()
            collision_obj.header = Header()
            collision_obj.header.frame_id = "base_link"  # Use base_link frame for MoveIt
            collision_obj.header.stamp = self.node.get_clock().now().to_msg()
            
            collision_obj.id = box_id
            collision_obj.operation = CollisionObject.ADD  # 0 = ADD
            
            # Set primitive (box)
            from shape_msgs.msg import SolidPrimitive
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX  # 1 = BOX
            primitive.dimensions = [
                float(size_3d['width']),
                float(size_3d['height']),
                float(size_3d['depth'])
            ]
            collision_obj.primitives = [primitive]
            
            # Set pose
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = float(base_point_3d[0])
            pose.position.y = float(base_point_3d[1])
            pose.position.z = float(base_point_3d[2])
            pose.orientation.w = 1.0  # No rotation
            collision_obj.primitive_poses = [pose]
            
            # Publish
            self.moveit_collision_pub.publish(collision_obj)
            self.published_blue_box_ids.add(box_id)
            self.get_logger().debug(
                f'Published blue box {collision_obj.id} to /obsFromImg: '
                f'pos=({base_point_3d[0]:.3f}, {base_point_3d[1]:.3f}, {base_point_3d[2]:.3f}), '
                f'size=({size_3d["width"]:.3f}, {size_3d["height"]:.3f}, {size_3d["depth"]:.3f})'
            )
        
        # Remove boxes that are no longer detected
        boxes_to_remove = self.published_blue_box_ids - current_box_ids
        for box_id in boxes_to_remove:
            self._remove_blue_box_from_moveit(box_id)
            self.published_blue_box_ids.discard(box_id)
    
    def _remove_blue_box_from_moveit(self, box_id):
        """Remove a blue box from MoveIt planning scene"""
        from moveit_msgs.msg import CollisionObject
        from std_msgs.msg import Header
        
        collision_obj = CollisionObject()
        collision_obj.header = Header()
        collision_obj.header.frame_id = "base_link"
        collision_obj.header.stamp = self.node.get_clock().now().to_msg()
        collision_obj.id = box_id
        collision_obj.operation = CollisionObject.REMOVE  # 1 = REMOVE
        
        self.moveit_collision_pub.publish(collision_obj)
        self.get_logger().debug(f'Removed blue box {box_id} from MoveIt planning scene')
    
    async def handle_request(self, request):
        """Handle detection service request"""
        if request.command == "detect":
            return await self._detect_leaves(request)
        else:
            return {
                'success': False,
                'message': f"Unknown command: {request.command}",
                'coordinates': [],
                'num_leaves': 0,
                'has_yellow_tape': [],
                'yellow_ratio': [],
                'health_status': [],
                'debug_info': ''
            }
    
    async def _detect_leaves(self, request):
        """Handle detection service request - use latest results or re-detect with custom params"""
        if self.current_frame is None or self.current_depth is None:
            return {
                'success': False,
                'message': "No frame available",
                'coordinates': [],
                'num_leaves': 0,
                'has_yellow_tape': [],
                'yellow_ratio': [],
                'health_status': [],
                'debug_info': ''
            }
        
        try:
            # If min_area is 0 or matches default, use latest continuous detection results
            use_custom_params = (request.min_area > 0 and request.min_area != self.min_area_default)
            
            if not use_custom_params and self.continuous_detection:
                # Use latest detection results (faster, no re-computation)
                with self.latest_data_lock:
                    if self.latest_coordinates_base is not None:
                        num_leaves = len(self.latest_coordinates_base)
                        if num_leaves > 0:
                            attrs = self.latest_leaf_attributes
                            return {
                                'success': True,
                                'message': f"Detected {num_leaves} leaves (using latest continuous detection)",
                                'coordinates': self.latest_coordinates_base.copy(),
                                'num_leaves': num_leaves,
                                'has_yellow_tape': list(attrs.get('has_yellow_tape', [])),
                                'yellow_ratio': list(attrs.get('yellow_ratio', [])),
                                'health_status': list(attrs.get('health_status', [])),
                                'debug_info': json.dumps({
                                    'source': 'latest_continuous_detection',
                                    'has_base_coords': self.tf_handler is not None
                                })
                            }
                        else:
                            return {
                                'success': False,
                                'message': "No leaves detected in latest frame",
                                'coordinates': [],
                                'num_leaves': 0,
                                'has_yellow_tape': [],
                                'yellow_ratio': [],
                                'health_status': [],
                                'debug_info': json.dumps({'source': 'latest_continuous_detection'})
                            }
            
            # Re-detect with custom parameters or if continuous detection is disabled
            min_area = request.min_area if request.min_area > 0 else self.min_area_default
            detection_result, leaf_data, bounding_boxes = self.detect_leaves_with_plantcv(
                self.current_frame,
                min_area=min_area
            )
            
            if leaf_data is None or leaf_data.get('num_leaves', 0) == 0:
                return {
                    'success': False,
                    'message': detection_result,
                    'coordinates': [],
                    'num_leaves': 0,
                    'has_yellow_tape': [],
                    'yellow_ratio': [],
                    'health_status': [],
                    'debug_info': json.dumps({'detection_result': detection_result, 'min_area': min_area})
                }
            
            # Convert to base frame coordinates
            coordinates = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
            has_yellow, yellow_ratio, health_status = self._extract_leaf_attributes(
                leaf_data.get('coordinates', [])
            )
            
            return {
                'success': True,
                'message': f"Detected {len(coordinates)} leaves",
                'coordinates': coordinates,
                'num_leaves': len(coordinates),
                'has_yellow_tape': has_yellow,
                'yellow_ratio': yellow_ratio,
                'health_status': health_status,
                'debug_info': json.dumps({
                    'num_leaves': leaf_data['num_leaves'],
                    'timestamp': leaf_data.get('timestamp', ''),
                    'has_base_coords': self.tf_handler is not None,
                    'min_area': min_area,
                    'source': 'on_demand_detection'
                })
            }
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
            import traceback
            return {
                'success': False,
                'message': f"Detection failed: {str(e)}",
                'coordinates': [],
                'num_leaves': 0,
                'has_yellow_tape': [],
                'yellow_ratio': [],
                'health_status': [],
                'debug_info': json.dumps({'error': str(e), 'traceback': traceback.format_exc()})
            }
    
    def measure_box_dimensions_3d(self, bbox_2d, depth_image, mask=None, sample_step=3):
        """
        Measure actual 3D dimensions of box using depth map (based on box_dimensioner method)
        
        This method references RealSense SDK's box_dimensioner example:
        1. Densely sample 3D points within detection region
        2. Use statistical methods to remove outliers
        3. Calculate 3D bounding box to get length/width/height
        
        Args:
            bbox_2d: 2D bounding box (x_min, y_min, x_max, y_max) in original image coordinates
            depth_image: Depth image (16-bit, unit: mm)
            mask: Optional mask (if provided, only sample points within mask)
            sample_step: Sampling step (pixels), default 3 (denser sampling for higher precision)
        
        Returns:
            dict: {
                'width': length (meters, X direction),
                'height': height (meters, Y direction),
                'depth': width (meters, Z direction),
                'points_3d': list of sampled 3D points,
                'min_point': minimum boundary point (x, y, z),
                'max_point': maximum boundary point (x, y, z),
                'num_samples': number of sample points
            } or None (if failed)
        """
        if depth_image is None or self.tf_handler is None or self.tf_handler.intrinsics is None:
            return None
        
        x_min, y_min, x_max, y_max = bbox_2d
        
        # Ensure coordinates are within image bounds
        x_min = max(0, int(x_min))
        y_min = max(0, int(y_min))
        x_max = min(depth_image.shape[1], int(x_max))
        y_max = min(depth_image.shape[0], int(y_max))
        
        if x_max <= x_min or y_max <= y_min:
            return None
        
        # Method 1: Dense sampling of 3D points (core method similar to box_dimensioner)
        points_3d = []
        valid_depths = []
        
        # Dense sampling within bounding box
        for y in range(y_min, y_max, sample_step):
            for x in range(x_min, x_max, sample_step):
                # If mask provided, check if point is within mask
                if mask is not None:
                    if (y < 0 or y >= mask.shape[0] or 
                        x < 0 or x >= mask.shape[1] or
                        mask[y, x] == 0):
                        continue
                
                # Get depth value
                depth_mm = depth_image[y, x]
                
                # Filter invalid depth (stricter filtering)
                if depth_mm <= 0 or depth_mm > 5000:
                    continue
                
                # Convert to 3D coordinates (using tf_handler method)
                point_3d = self.tf_handler.pixel_to_3d(x, y, depth_mm)
                if point_3d is not None:
                    points_3d.append(point_3d)
                    valid_depths.append(depth_mm)
        
        if len(points_3d) < 8:  # Need at least 8 points for accurate size estimation
            return None
        
        # Method 2: Use statistical methods to remove outliers (robustness like box_dimensioner)
        points_array = np.array(points_3d)
        depths_array = np.array(valid_depths)
        
        # Calculate depth median, remove outliers (large depth differences may be background or noise)
        median_depth = np.median(depths_array)
        depth_std = np.std(depths_array)
        depth_threshold = median_depth + 3 * depth_std  # 3 standard deviations
        
        # Filter outlier depth points
        valid_mask = depths_array <= depth_threshold
        filtered_points = points_array[valid_mask]
        
        if len(filtered_points) < 8:
            filtered_points = points_array  # If too few points after filtering, use original
        
        # Method 3: Calculate 3D bounding box (core of box_dimensioner)
        # Find min/max X, Y, Z values of all points
        min_point = filtered_points.min(axis=0)  # [min_x, min_y, min_z]
        max_point = filtered_points.max(axis=0)   # [max_x, max_y, max_z]
        
        # Calculate dimensions (in camera coordinate system)
        # X direction = length (left/right, usually longest edge)
        # Y direction = height (up/down)
        # Z direction = width (front/back, distance to camera, usually shortest edge)
        length = float(max_point[0] - min_point[0])   # X direction
        height = float(max_point[1] - min_point[1])    # Y direction
        width = float(max_point[2] - min_point[2])      # Z direction
        
        # Method 4: If width (Z direction) is too small, may only see one face
        # Use depth variation to estimate width (like box_dimensioner handling single view)
        if width < 0.01:  # Less than 1cm, may only see one face
            z_values = filtered_points[:, 2]
            z_std = float(np.std(z_values))
            if z_std > 0.003:  # If std > 3mm, there is depth variation
                # Use statistical estimation: 2x std + median range
                z_range = np.percentile(z_values, 95) - np.percentile(z_values, 5)
                width = max(z_range, z_std * 2.5)  # Use more conservative estimate
            else:
                # If depth variation is small, use pixel size estimation
                pixel_width = x_max - x_min
                avg_depth = float(np.median(z_values))
                if avg_depth > 0 and self.tf_handler.intrinsics:
                    pixel_size = avg_depth / self.tf_handler.intrinsics.fx
                    width = pixel_width * pixel_size * 0.3  # Conservative estimate at 30% of pixel width
                else:
                    width = 0.1  # Default 10cm
        
        # Ensure minimum dimensions are reasonable (at least 1cm)
        length = max(length, 0.01)
        height = max(height, 0.01)
        width = max(width, 0.01)
        
        return {
            'width': length,   # X direction (length)
            'height': height,  # Y direction (height)
            'depth': width,    # Z direction (width)
            'points_3d': filtered_points.tolist(),
            'min_point': tuple(min_point),
            'max_point': tuple(max_point),
            'num_samples': len(filtered_points),
            'median_depth': float(np.median(depths_array)) if len(depths_array) > 0 else 0.0
        }
    
    def detect_leaves_with_plantcv(self, cv_image, min_area=None):
        """
        Detect leaves using PlantCV library
        Returns: (detection_result_string, leaf_data_dict, bounding_boxes_list)
        """
        try:
            h, w = cv_image.shape[:2]
            
            # Minimal cropping
            crop_img = pcv.crop(img=cv_image, x=20, y=20, h=h-40, w=w-40)
            
            # HSV color space - detect green and optional yellow tape
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 60, 40])
            upper_green = np.array([80, 255, 255])
            thresh_green = cv2.inRange(hsv, lower_green, upper_green)
            thresh_green = (thresh_green / 255).astype(np.uint8)
            
            # Step 2.5: Detect blue boxes (independent of leaf detection)
            blue_box_coordinates = []
            if self.detect_blue_box:
                # Use HSV color space to detect blue
                lower_blue_hsv = np.array(self.blue_hsv_lower, dtype=np.uint8)
                upper_blue_hsv = np.array(self.blue_hsv_upper, dtype=np.uint8)
                thresh_blue_hsv = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
                
                # Morphological processing: remove small noise, connect nearby regions
                kernel_blue_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                kernel_blue_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
                
                # Close operation to connect nearby regions, then open operation to remove small noise
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_hsv, cv2.MORPH_CLOSE, kernel_blue_medium, iterations=2)
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_morph, cv2.MORPH_OPEN, kernel_blue_small, iterations=1)
                
                # Detect blue box contours (including all levels to detect multiple faces)
                blue_contours, blue_hierarchy = cv2.findContours(thresh_blue_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # Collect all valid blue regions (may be different faces of the box)
                valid_blue_regions = []
                
                # Filter blue box contours (detect all visible faces)
                for idx, blue_cnt in enumerate(blue_contours):
                    area = cv2.contourArea(blue_cnt)
                    
                    # Lower minimum area requirement for single face, as we detect multiple faces
                    min_face_area = max(500, self.blue_min_area * 0.3)  # Single face at least 30% of total area or 500 pixels
                    if area < min_face_area:
                        continue
                    
                    # Calculate bounding box
                    x, y, w_rect, h_rect = cv2.boundingRect(blue_cnt)
                    if w_rect == 0 or h_rect == 0:
                        continue
                    
                    # Calculate rectangularity (ratio of contour area to bounding box area)
                    bbox_area = w_rect * h_rect
                    if bbox_area == 0:
                        continue
                    rect_ratio = area / bbox_area
                    
                    # Calculate aspect ratio
                    aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                    
                    # Relaxed filtering conditions (may be box side faces, shape may not be perfectly regular)
                    if rect_ratio < 0.4:  # Lowered to 0.4
                        continue
                    
                    if aspect_ratio < 0.2 or aspect_ratio > 5.0:  # Relaxed range
                        continue
                    
                    # Calculate center point
                    M = cv2.moments(blue_cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00']) + 20
                        cy = int(M['m01'] / M['m00']) + 20
                    else:
                        cx = x + w_rect // 2 + 20
                        cy = y + h_rect // 2 + 20
                    
                    # Get depth value and 3D coordinates (sample multiple points for more accurate depth)
                    depth_value_mm = 0
                    point_3d = None
                    
                    if self.current_depth is not None and self.tf_handler:
                        try:
                            # Sample multiple points within contour region
                            mask = np.zeros((crop_img.shape[0], crop_img.shape[1]), dtype=np.uint8)
                            cv2.drawContours(mask, [blue_cnt], -1, 255, -1)
                            
                            # Convert to original image coordinates (depth image is original size)
                            depth_x = x + 20
                            depth_y = y + 20
                            depth_x_end = min(self.current_depth.shape[1], depth_x + w_rect)
                            depth_y_end = min(self.current_depth.shape[0], depth_y + h_rect)
                            depth_x_start = max(0, depth_x)
                            depth_y_start = max(0, depth_y)
                            
                            # Sample depth values within mask region
                            mask_roi = mask[y:y+h_rect, x:x+w_rect]
                            depth_roi = self.current_depth[depth_y_start:depth_y_end, depth_x_start:depth_x_end]
                            
                            # Resize mask ROI to match depth ROI
                            if mask_roi.shape != depth_roi.shape:
                                mask_roi_resized = cv2.resize(mask_roi, (depth_roi.shape[1], depth_roi.shape[0]))
                            else:
                                mask_roi_resized = mask_roi
                            
                            valid_depths = depth_roi[(mask_roi_resized > 0) & (depth_roi > 0)]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.median(valid_depths))  # Use median for stability
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    point_3d = self.tf_handler.pixel_to_3d(cx, cy, depth_value_mm)
                        except Exception as e:
                            self.get_logger().debug(f'Blue region depth retrieval error: {e}')
                            pass
                    
                    # Save blue region info (may be one face of the box)
                    region_info = {
                        'contour': blue_cnt,
                        'area': float(area),
                        'bbox': (x, y, w_rect, h_rect),
                        'center_2d': (cx, cy),
                        'depth_mm': float(depth_value_mm),
                        'point_3d': point_3d,
                        'rect_ratio': float(rect_ratio),
                        'aspect_ratio': float(aspect_ratio),
                    }
                    valid_blue_regions.append(region_info)
                
                # Group regions into the same box based on depth and position
                blue_box_groups = []
                depth_tolerance = 50  # Depth tolerance (mm)
                position_tolerance = 100  # Position tolerance (pixels)
                
                for region in valid_blue_regions:
                    if region['point_3d'] is None:
                        # If no depth info, each region becomes its own group
                        blue_box_groups.append([region])
                        continue
                    
                    # Try to find matching group
                    matched = False
                    for group in blue_box_groups:
                        # Check if there is depth info
                        group_depths = [r['depth_mm'] for r in group if r['point_3d'] is not None]
                        if len(group_depths) == 0:
                            continue
                        
                        avg_group_depth = np.mean(group_depths)
                        depth_diff = abs(region['depth_mm'] - avg_group_depth)
                        
                        # Check if positions are close
                        group_centers = [r['center_2d'] for r in group]
                        avg_center = (np.mean([c[0] for c in group_centers]), 
                                     np.mean([c[1] for c in group_centers]))
                        pos_diff = np.sqrt((region['center_2d'][0] - avg_center[0])**2 + 
                                          (region['center_2d'][1] - avg_center[1])**2)
                        
                        # If both depth and position are close, join this group
                        if depth_diff < depth_tolerance and pos_diff < position_tolerance:
                            group.append(region)
                            matched = True
                            break
                    
                    if not matched:
                        # Create new group
                        blue_box_groups.append([region])
                
                # Create a complete blue box for each group
                blue_box_idx = 1
                for group in blue_box_groups:
                    if len(group) == 0:
                        continue
                    
                    # Calculate merged bounding box (containing all faces)
                    all_x_min = min([r['bbox'][0] for r in group])
                    all_y_min = min([r['bbox'][1] for r in group])
                    all_x_max = max([r['bbox'][0] + r['bbox'][2] for r in group])
                    all_y_max = max([r['bbox'][1] + r['bbox'][3] for r in group])
                    
                    merged_w = all_x_max - all_x_min
                    merged_h = all_y_max - all_y_min
                    merged_area = sum([r['area'] for r in group])
                    
                    # Calculate merged center point
                    merged_cx = (all_x_min + all_x_max) // 2 + 20
                    merged_cy = (all_y_min + all_y_max) // 2 + 20
                    
                    # Calculate average depth and 3D coordinates
                    valid_depths = [r['depth_mm'] for r in group if r['depth_mm'] > 0]
                    avg_depth_mm = int(np.mean(valid_depths)) if len(valid_depths) > 0 else 0
                    
                    # Get 3D coordinates
                    merged_point_3d = None
                    if avg_depth_mm > 0 and self.current_depth is not None and self.tf_handler:
                        try:
                            merged_point_3d = self.tf_handler.pixel_to_3d(merged_cx, merged_cy, avg_depth_mm)
                        except:
                            pass
                    
                    # Calculate box 3D dimensions (using depth map sampling method, based on box_dimensioner algorithm)
                    box_3d_size = None
                    if self.current_depth is not None and merged_point_3d and self.tf_handler and self.tf_handler.intrinsics:
                        try:
                            # Create merged mask (in original image coordinates, to limit sampling area)
                            bbox_2d = (
                                all_x_min + 20,  # x_min
                                all_y_min + 20,  # y_min
                                all_x_max + 20,  # x_max
                                all_y_max + 20   # y_max
                            )
                            
                            # Create merged mask (in original image coordinates)
                            merged_mask = np.zeros((self.current_depth.shape[0], self.current_depth.shape[1]), dtype=np.uint8)
                            for region in group:
                                contour = region.get('contour')
                                if contour is not None:
                                    # Convert contour coordinates from cropped image to original image coordinates
                                    adjusted_contour = contour + np.array([20, 20])
                                    cv2.drawContours(merged_mask, [adjusted_contour], -1, 255, -1)
                            
                            # Use new 3D measurement method (based on box_dimensioner algorithm)
                            dimensions = self.measure_box_dimensions_3d(
                                bbox_2d, 
                                self.current_depth, 
                                mask=merged_mask,
                                sample_step=3  # Sample every 3 pixels (denser, higher precision)
                            )
                            
                            if dimensions:
                                box_3d_size = {
                                    'width': dimensions['width'],
                                    'height': dimensions['height'],
                                    'depth': dimensions['depth'],
                                    'num_samples': dimensions.get('num_samples', 0),
                                    'min_point': dimensions.get('min_point'),
                                    'max_point': dimensions.get('max_point')
                                }
                        except Exception as e:
                            self.get_logger().warn(f'Box 3D dimension measurement error: {e}')
                            # If new method fails, fallback to old method
                            if len(valid_depths) > 0:
                                depth_m = avg_depth_mm * 0.001
                                pixel_size = depth_m / self.tf_handler.intrinsics.fx if self.tf_handler.intrinsics.fx > 0 else 0.001
                                box_3d_size = {
                                    'width': merged_w * pixel_size,
                                    'height': merged_h * pixel_size,
                                    'depth': (max(valid_depths) - min(valid_depths)) * 0.001 if len(valid_depths) > 1 else 0.1
                                }
                    
                    # If new method didn't execute or failed, use old method as fallback
                    if box_3d_size is None and len(valid_depths) > 0 and merged_point_3d and self.tf_handler and self.tf_handler.intrinsics:
                        depth_m = avg_depth_mm * 0.001
                        pixel_size = depth_m / self.tf_handler.intrinsics.fx if self.tf_handler.intrinsics.fx > 0 else 0.001
                        box_3d_size = {
                            'width': merged_w * pixel_size,
                            'height': merged_h * pixel_size,
                            'depth': (max(valid_depths) - min(valid_depths)) * 0.001 if len(valid_depths) > 1 else 0.1
                        }
                    
                    # Save complete blue box info
                    blue_box_info = {
                        'id': blue_box_idx,
                        'type': 'blue_box',
                        'num_faces': len(group),  # Number of detected faces
                        'center': {'x': merged_cx, 'y': merged_cy},
                        'bounding_box': {
                            'x': all_x_min + 20,
                            'y': all_y_min + 20,
                            'width': merged_w,
                            'height': merged_h,
                            'x_min': all_x_min + 20,
                            'y_min': all_y_min + 20,
                            'x_max': all_x_max + 20,
                            'y_max': all_y_max + 20
                        },
                        'area': float(merged_area),
                        'depth_mm': float(avg_depth_mm),
                        'point_3d': merged_point_3d,
                        'size_3d': box_3d_size,
                        'faces': group  # Save all face info
                    }
                    blue_box_coordinates.append(blue_box_info)
                    
                    blue_box_idx += 1
                
                # Debug info
                if len(blue_box_coordinates) > 0:
                    self.get_logger().info(f'📦 Detected {len(blue_box_coordinates)} blue boxes')
            
            thresh_yellow_full = None
            if self.detect_yellow_tape:
                # Use configurable HSV range, stricter defaults for detecting real yellow tape
                lower_yellow = np.array(self.yellow_hsv_lower, dtype=np.uint8)
                upper_yellow = np.array(self.yellow_hsv_upper, dtype=np.uint8)
                thresh_yellow_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # Use stricter morphological operations to remove small noise
                # First open operation to remove small noise, then close operation to connect nearby regions
                kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_OPEN, kernel_small, iterations=1)
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_CLOSE, kernel_small, iterations=1)
                
                # Use LAB color space as supplement, but with stricter range
                # Only detect real bright yellow tape, avoid detecting yellow parts of leaves
                lab = cv2.cvtColor(crop_img, cv2.COLOR_BGR2LAB)
                # Yellow tape is usually bright yellow: high L (brightness), low a (greenish), high b (yellowish)
                # Use stricter range to avoid false detection of yellow leaf edges
                lab_yellow_mask = np.zeros_like(thresh_yellow_hsv, dtype=np.uint8)
                # Stricter conditions: L>110 (bright), a<140 (greenish), b>150 (very yellow)
                lab_yellow_mask[(lab[:,:,0] > 110) & (lab[:,:,1] < 140) & (lab[:,:,2] > 150)] = 255
                
                # Merge HSV and LAB detection results (use OR operation for more sensitivity)
                # Prioritize OR operation for more sensitivity (easier to detect yellow tape)
                thresh_yellow_full = cv2.bitwise_or(thresh_yellow_hsv, lab_yellow_mask)
                
                # Morphological processing to remove small noise
                kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_OPEN, kernel_medium, iterations=1)
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
            
            # Morphological processing
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh_green = cv2.morphologyEx(thresh_green, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh_green = cv2.morphologyEx(thresh_green, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # Contour detection
            contours, _ = cv2.findContours(thresh_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                # Even without leaves, return data if blue boxes are detected
                if len(blue_box_coordinates) > 0:
                    result = f"Detected {len(blue_box_coordinates)} blue boxes (no leaves)"
                    leaf_data = {
                        'num_leaves': 0,
                        'num_blue_boxes': len(blue_box_coordinates),
                        'timestamp': datetime.now().isoformat(),
                        'coordinates': [],
                        'blue_boxes': blue_box_coordinates
                    }
                    return result, leaf_data, []
                return "No leaves detected", None, None
            
            # Filtering parameters
            min_area_threshold = min_area if min_area and min_area > 0 else 2000
            max_area = 50000
            min_circularity = 0.2
            max_circularity = 0.9
            
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                if area < min_area_threshold or area > max_area:
                    continue
                
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < min_circularity or circularity > max_circularity:
                    continue
                
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                if w_rect == 0 or h_rect == 0:
                    continue
                
                aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                if aspect_ratio < 0.4 or aspect_ratio > 2.5:
                    continue
                
                min_dimension = min(w_rect, h_rect)
                if min_dimension < 30:
                    continue
                
                valid_contours.append(cnt)
            
            if len(valid_contours) == 0:
                # Even without valid leaves, return data if blue boxes are detected
                if len(blue_box_coordinates) > 0:
                    result = f"Detected {len(blue_box_coordinates)} blue boxes (no valid leaves)"
                    leaf_data = {
                        'num_leaves': 0,
                        'num_blue_boxes': len(blue_box_coordinates),
                        'timestamp': datetime.now().isoformat(),
                        'coordinates': [],
                        'blue_boxes': blue_box_coordinates
                    }
                    return result, leaf_data, []
                return "No valid leaves detected", None, None
            
            # Extract coordinates (with depth filtering)
            leaf_coordinates = []
            bounding_boxes = []
            leaf_idx = 1
            
            for cnt in valid_contours:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # Calculate center point
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00']) + 20
                    cy = int(M['m01'] / M['m00']) + 20
                else:
                    cx = x + w_rect // 2 + 20
                    cy = y + h_rect // 2 + 20
                
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # Get depth value and 3D coordinates
                # Use dense sampling within leaf region, select minimum depth value (leaf is above box, should be closer)
                depth_value_mm = 0
                point_3d = None
                has_valid_depth = False
                
                if self.current_depth is not None and self.tf_handler:
                    try:
                        # Method 1: Dense sampling of depth values within leaf contour region
                        # Create mask for leaf region
                        leaf_mask = np.zeros((self.current_depth.shape[0], self.current_depth.shape[1]), dtype=np.uint8)
                        # Note: cnt coordinates need adjustment because original image has cropped edges
                        adjusted_cnt = cnt.copy()
                        adjusted_cnt[:, :, 0] = adjusted_cnt[:, :, 0] + 20  # x coordinate offset
                        adjusted_cnt[:, :, 1] = adjusted_cnt[:, :, 1] + 20  # y coordinate offset
                        cv2.drawContours(leaf_mask, [adjusted_cnt], -1, 255, -1)
                        
                        # Sample depth values within mask region (every 3 pixels)
                        leaf_depths = []
                        for py in range(max(0, y + 20), min(self.current_depth.shape[0], y + h_rect + 20), 3):
                            for px in range(max(0, x + 20), min(self.current_depth.shape[1], x + w_rect + 20), 3):
                                if leaf_mask[py, px] > 0:  # Only sample within leaf region
                                    depth_val = self.current_depth[py, px]
                                    if depth_val > 0 and 100 <= depth_val <= 2000:
                                        leaf_depths.append(depth_val)
                        
                        if len(leaf_depths) > 0:
                            # Use minimum depth value (leaf is above box, should be closer)
                            # But to remove noise, use average of top 20% minimum values
                            sorted_depths = sorted(leaf_depths)
                            num_samples = max(1, int(len(sorted_depths) * 0.2))  # Top 20% minimum values
                            depth_value_mm = int(np.mean(sorted_depths[:num_samples]))
                            
                            if depth_value_mm > 0 and 100 <= depth_value_mm <= 2000:
                                has_valid_depth = True
                                point_3d = self.tf_handler.pixel_to_3d(cx, cy, depth_value_mm)
                        else:
                            # Fallback method: use region near center point
                            if cx < self.current_depth.shape[1] and cy < self.current_depth.shape[0]:
                                # Use 5x5 region, take minimum of valid depths
                                y_start = max(0, cy - 2)
                                y_end = min(self.current_depth.shape[0], cy + 3)
                                x_start = max(0, cx - 2)
                                x_end = min(self.current_depth.shape[1], cx + 3)
                                depth_region = self.current_depth[y_start:y_end, x_start:x_end]
                                valid_depths = depth_region[(depth_region > 0) & (depth_region >= 100) & (depth_region <= 2000)]
                                
                                if len(valid_depths) > 0:
                                    depth_value_mm = int(np.min(valid_depths))  # Use minimum value (shallowest depth)
                                    has_valid_depth = True
                                    point_3d = self.tf_handler.pixel_to_3d(cx, cy, depth_value_mm)
                    except Exception as e:
                        self.get_logger().debug(f'Leaf depth sampling error: {e}')
                        pass
                
                if not has_valid_depth:
                    continue
                
                has_yellow_tape = False
                yellow_ratio = 0.0
                if self.detect_yellow_tape and thresh_yellow_full is not None:
                    # Create leaf region mask (using full contour, not bounding box)
                    # This allows more accurate detection of yellow areas inside leaf
                    leaf_mask = np.zeros_like(thresh_green, dtype=np.uint8)
                    cv2.drawContours(leaf_mask, [cnt], -1, 255, -1)
                    
                    # Calculate directly on full mask, not bounding box region
                    # This avoids bounding box potentially including non-leaf areas
                    leaf_pixels = np.sum(leaf_mask > 0)
                    yellow_pixels = np.sum((leaf_mask > 0) & (thresh_yellow_full > 0))
                    
                    if leaf_pixels > 0:
                        yellow_ratio = yellow_pixels / leaf_pixels
                        
                        # Additional validation: check connectivity and size of yellow region
                        # Real yellow tape should be continuous with certain size
                        yellow_in_leaf = (leaf_mask > 0) & (thresh_yellow_full > 0)
                        yellow_contours, _ = cv2.findContours(
                            yellow_in_leaf.astype(np.uint8), 
                            cv2.RETR_EXTERNAL, 
                            cv2.CHAIN_APPROX_SIMPLE
                        )
                        
                        # Calculate area of largest yellow connected region
                        max_yellow_area = 0
                        if len(yellow_contours) > 0:
                            max_yellow_area = max(cv2.contourArea(c) for c in yellow_contours)
                        
                        # Lower minimum area requirement: at least 50 pixels or 0.5% of leaf area (easier detection)
                        min_yellow_area_threshold = max(50, leaf_pixels * 0.005)
                        
                        # Use OR condition: satisfy either ratio threshold or minimum area requirement
                        if (yellow_ratio >= self.yellow_ratio_threshold or 
                            max_yellow_area >= min_yellow_area_threshold):
                            has_yellow_tape = True
                    else:
                        pass  # Leaf region mask is empty, skip yellow tape detection
                
                # Save leaf info
                leaf_info = {
                    'id': leaf_idx,
                    'center': {'x': cx, 'y': cy},
                    'bounding_box': {
                        'x': x + 20,
                        'y': y + 20,
                        'width': w_rect,
                        'height': h_rect,
                        'x_min': x + 20,
                        'y_min': y + 20,
                        'x_max': x + 20 + w_rect,
                        'y_max': y + 20 + h_rect
                    },
                    'area': float(area),
                    'perimeter': float(perimeter),
                    'depth_mm': float(depth_value_mm),
                    'point_3d': point_3d,
                    'has_yellow_tape': has_yellow_tape,
                    'yellow_ratio': float(yellow_ratio),
                    'health_status': 'unhealthy' if has_yellow_tape else 'healthy'
                }
                leaf_coordinates.append(leaf_info)
                
                bounding_boxes.append({
                    'id': leaf_idx,
                    'center_x': cx,
                    'center_y': cy,
                    'width': w_rect,
                    'height': h_rect,
                    'x': x + 20,
                    'y': y + 20
                })
                
                leaf_idx += 1
            
            
            num_detected_leaves = len(leaf_coordinates)
            
            if num_detected_leaves == 0:
                # Even without leaves after depth filtering, return data if blue boxes detected
                if len(blue_box_coordinates) > 0:
                    result = f"Detected {len(blue_box_coordinates)} blue boxes (after leaf depth filtering)"
                    leaf_data = {
                        'num_leaves': 0,
                        'num_blue_boxes': len(blue_box_coordinates),
                        'timestamp': datetime.now().isoformat(),
                        'coordinates': [],
                        'blue_boxes': blue_box_coordinates
                    }
                    return result, leaf_data, []
                return "No valid leaves detected (after depth filtering)", None, None
            
            # Concise output of detection results (using base coordinates)
            healthy_leaves = []
            unhealthy_leaves = []
            
            for leaf in leaf_coordinates:
                # Prioritize base coordinates
                base_coords = leaf.get('base_coordinates')
                point_3d = leaf.get('point_3d')
                
                if base_coords and len(base_coords) >= 3:
                    coord_str = f"({base_coords[0]:.2f}, {base_coords[1]:.2f}, {base_coords[2]:.2f})"
                elif point_3d:
                    # Try to convert to base coordinates
                    if self.tf_handler:
                        base_pt = self.tf_handler.camera_to_base(point_3d)
                        if base_pt:
                            coord_str = f"({base_pt[0]:.2f}, {base_pt[1]:.2f}, {base_pt[2]:.2f})"
                        else:
                            coord_str = f"({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})"
                    else:
                        coord_str = f"({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})"
                else:
                    coord_str = "(N/A)"
                
                if leaf.get('has_yellow_tape', False):
                    unhealthy_leaves.append(f"Unhealthy{len(unhealthy_leaves)+1}: {coord_str}")
                else:
                    healthy_leaves.append(f"Healthy{len(healthy_leaves)+1}: {coord_str}")
            
            # Output concise detection results
            output_lines = []
            output_lines.extend(healthy_leaves)
            output_lines.extend(unhealthy_leaves)
            
            for i, box in enumerate(blue_box_coordinates):
                # Prioritize base coordinates
                base_coords = box.get('base_coordinates')
                point_3d = box.get('point_3d')
                
                if base_coords and len(base_coords) >= 3:
                    output_lines.append(f"Box{i+1}: ({base_coords[0]:.2f}, {base_coords[1]:.2f}, {base_coords[2]:.2f})")
                elif point_3d:
                    if self.tf_handler:
                        base_pt = self.tf_handler.camera_to_base(point_3d)
                        if base_pt:
                            output_lines.append(f"Box{i+1}: ({base_pt[0]:.2f}, {base_pt[1]:.2f}, {base_pt[2]:.2f})")
                        else:
                            output_lines.append(f"Box{i+1}: ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})")
            
            if output_lines:
                self.get_logger().info("📊 [base] " + " | ".join(output_lines))
            
            result = f"Detected {num_detected_leaves} leaves"
            if len(blue_box_coordinates) > 0:
                result += f", {len(blue_box_coordinates)} blue boxes"
            
            leaf_data = {
                'num_leaves': num_detected_leaves,
                'num_blue_boxes': len(blue_box_coordinates),
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates,
                'blue_boxes': blue_box_coordinates
            }
            
            return result, leaf_data, bounding_boxes
            
        except Exception as e:
            self.get_logger().error(f'✗ PlantCV detection error: {str(e)}')
            return f"Detection error: {str(e)}", None, None
    
    def draw_annotations(self, display_frame, leaf_coordinates, blue_box_coordinates=None):
        """Draw annotations on image"""
        if display_frame is None:
            return None
        
        # Always return a copy of the frame (with or without annotations)
        annotated = display_frame.copy()
        
        if blue_box_coordinates is None:
            blue_box_coordinates = []
        
        if not leaf_coordinates and not blue_box_coordinates:
            # No leaves or boxes detected, just return original frame
            return annotated
        
        # Draw annotations for detected leaves
        try:
            colors = [
                (255, 0, 0),      # Blue
                (0, 255, 0),      # Green
                (0, 0, 255),      # Red
                (255, 255, 0),    # Cyan
                (255, 0, 255),    # Magenta
                (0, 255, 255),    # Yellow
            ]
            
            for leaf in leaf_coordinates:
                obj_id = leaf['id']
                color = colors[(obj_id - 1) % len(colors)]
                
                bbox = leaf['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # Draw bounding box
                cv2.rectangle(annotated, (x, y), (x_max, y_max), color, 2)
                
                # Draw label - larger font and background
                label = f'Leaf {obj_id}'
                if leaf.get('has_yellow_tape', False):
                    label += ' [Tape]'
                font_scale_leaf = 0.8  # Larger font
                thickness_leaf = 2
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale_leaf, thickness_leaf)[0]
                label_y = max(y - 10, label_size[1] + 10)
                
                # Draw semi-transparent background (clearer)
                overlay = annotated.copy()
                cv2.rectangle(overlay, (x - 2, label_y - label_size[1] - 8),
                            (x + label_size[0] + 8, label_y + 8), color, -1)
                cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)
                cv2.rectangle(annotated, (x - 2, label_y - label_size[1] - 8),
                            (x + label_size[0] + 8, label_y + 8), color, 2)
                cv2.putText(annotated, label, (x + 3, label_y - 3),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale_leaf, (255, 255, 255), thickness_leaf)
                
                # Draw center point
                cx = leaf['center']['x']
                cy = leaf['center']['y']
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                cv2.circle(annotated, (cx, cy), 15, color, 2)
                
                # Draw 3D coordinates if available - larger font
                point_3d = leaf.get('point_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    font_scale_coord = 0.6
                    thickness_coord = 2
                    coord_text_size = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale_coord, thickness_coord)[0]
                    
                    # Draw background
                    text_bg_y = y_max - 5
                    overlay = annotated.copy()
                    cv2.rectangle(overlay, (x + 3, text_bg_y - coord_text_size[1] - 5),
                                (x + coord_text_size[0] + 8, text_bg_y + 5), (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)
                    
                    cv2.putText(annotated, coord_text,
                               (x + 5, text_bg_y),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale_coord, (255, 255, 255), thickness_coord)
            
            # Draw blue boxes (show all detected faces)
            blue_box_color = (255, 0, 0)  # Blue for blue boxes (BGR format)
            face_color = (200, 100, 100)  # Light blue for individual faces
            for blue_box in blue_box_coordinates:
                obj_id = blue_box['id']
                bbox = blue_box['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # Draw merged complete bounding box (thick line)
                cv2.rectangle(annotated, (x, y), (x_max, y_max), blue_box_color, 3)
                
                # Draw each detected face (thin line)
                num_faces = blue_box.get('num_faces', 1)
                faces = blue_box.get('faces', [])
                if len(faces) > 0:
                    # Draw contour of each face
                    for face_idx, face in enumerate(faces):
                        contour = face.get('contour')
                        if contour is not None:
                            # Adjust contour coordinates (add crop offset)
                            adjusted_contour = contour + np.array([20, 20])
                            cv2.drawContours(annotated, [adjusted_contour], -1, face_color, 1)
                
                # Draw label (show face count) - larger font and background
                label = f'Blue Box {obj_id}'
                if num_faces > 1:
                    label += f' ({num_faces} faces)'
                font_scale_label = 0.8  # Larger font
                thickness_label = 2
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale_label, thickness_label)[0]
                label_y = max(y - 10, label_size[1] + 10)
                
                # Draw semi-transparent background (clearer)
                overlay = annotated.copy()
                cv2.rectangle(overlay, (x - 2, label_y - label_size[1] - 8),
                            (x + label_size[0] + 8, label_y + 8), blue_box_color, -1)
                cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)
                cv2.rectangle(annotated, (x - 2, label_y - label_size[1] - 8),
                            (x + label_size[0] + 8, label_y + 8), blue_box_color, 2)
                cv2.putText(annotated, label, (x + 3, label_y - 3),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale_label, (255, 255, 255), thickness_label)
                
                # Draw center point
                cx = blue_box['center']['x']
                cy = blue_box['center']['y']
                cv2.circle(annotated, (cx, cy), 5, blue_box_color, -1)
                cv2.circle(annotated, (cx, cy), 15, blue_box_color, 2)
                
                # If available, draw 3D coordinates and dimensions - larger font, improved layout
                point_3d = blue_box.get('point_3d')
                size_3d = blue_box.get('size_3d')
                if point_3d:
                    font_scale_info = 0.7  # Larger info font
                    thickness_info = 2
                    line_height = 25  # Line spacing
                    
                    # Only show dimension info (length/width/height)
                    if size_3d:
                        size_text = f"L:{size_3d['width']*100:.1f}cm  H:{size_3d['height']*100:.1f}cm  W:{size_3d['depth']*100:.1f}cm"
                        size_text_size = cv2.getTextSize(size_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale_info, thickness_info)[0]
                        
                        # Draw background
                        text_bg_y = y_max - 5
                        overlay = annotated.copy()
                        cv2.rectangle(overlay, (x + 3, text_bg_y - size_text_size[1] - 5),
                                    (x + size_text_size[0] + 8, text_bg_y + 5), (0, 0, 0), -1)
                        cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)
                        
                        cv2.putText(annotated, size_text,
                                   (x + 5, text_bg_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, font_scale_info, (255, 255, 255), thickness_info)
            
            # Display total count - larger font and background
            total_text = f"Leaves: {len(leaf_coordinates)}"
            if len(blue_box_coordinates) > 0:
                total_text += f" | Blue Boxes: {len(blue_box_coordinates)}"
            font_scale_total = 1.0
            thickness_total = 3
            total_text_size = cv2.getTextSize(total_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale_total, thickness_total)[0]
            
            # Draw semi-transparent background
            overlay = annotated.copy()
            cv2.rectangle(overlay, (8, 8),
                        (total_text_size[0] + 12, total_text_size[1] + 20), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)
            
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale_total, (0, 255, 0), thickness_total)
            
            return annotated
            
        except Exception as e:
            self.get_logger().error(f'✗ Annotation drawing error: {str(e)}')
            return display_frame.copy()

