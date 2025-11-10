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
        import threading
        self.latest_data_lock = threading.Lock()
        
        # Detection mode: continuous or on-demand
        self.continuous_detection = node.declare_parameter('continuous_detection', True).value
        self.min_area_default = 2000.0  # Default min area
        
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
        
        self.get_logger().info('✓ DetectionHandler initialized')
        if self.continuous_detection:
            self.get_logger().info('📡 Continuous detection mode: images will be published continuously')
    
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
                    
                    # Log base frame coordinates periodically
                    if self._frame_count % 30 == 0 and self.latest_coordinates_base:
                        self.get_logger().info('📍 Latest base coordinates:')
                        for i, point in enumerate(self.latest_coordinates_base):
                            self.get_logger().info(f'  Leaf {i+1}: X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m')
                else:
                    self.latest_leaf_data = None
                    self.latest_coordinates_base = []
            
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
                
                results_json = {
                    'num_leaves': leaf_data['num_leaves'],
                    'timestamp': leaf_data.get('timestamp', ''),
                    'coordinates': leaf_data.get('coordinates', []),  # Camera frame coordinates
                    'base_coordinates': base_coords_list if base_coords_list else []  # Base frame coordinates
                }
                results_msg = String()
                results_msg.data = json.dumps(results_json)
                self.detection_results_pub.publish(results_msg)
            else:
                # Publish empty results if no leaves detected
                results_json = {
                    'num_leaves': 0,
                    'timestamp': datetime.now().isoformat(),
                    'coordinates': []
                }
                results_msg = String()
                results_msg.data = json.dumps(results_json)
                self.detection_results_pub.publish(results_msg)
            
            # Publish annotated image (with or without detections)
            annotated_image = self.draw_annotations(
                self.current_frame, 
                leaf_data.get('coordinates', []) if leaf_data else []
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
    
    async def handle_request(self, request):
        """Handle detection service request"""
        if request.command == "detect":
            return await self._detect_leaves(request)
        else:
            return {
                'success': False,
                'message': f"Unknown command: {request.command}"
            }
    
    async def _detect_leaves(self, request):
        """Handle detection service request - use latest results or re-detect with custom params"""
        if self.current_frame is None or self.current_depth is None:
            return {
                'success': False,
                'message': "No frame available",
                'coordinates': [],
                'num_leaves': 0,
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
                            return {
                                'success': True,
                                'message': f"Detected {num_leaves} leaves (using latest continuous detection)",
                                'coordinates': self.latest_coordinates_base.copy(),
                                'num_leaves': num_leaves,
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
                    'debug_info': json.dumps({'detection_result': detection_result, 'min_area': min_area})
                }
            
            # Convert to base frame coordinates
            coordinates = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
            
            return {
                'success': True,
                'message': f"Detected {len(coordinates)} leaves",
                'coordinates': coordinates,
                'num_leaves': len(coordinates),
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
                'debug_info': json.dumps({'error': str(e), 'traceback': traceback.format_exc()})
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
            
            # HSV color space - detect green
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 60, 40])
            upper_green = np.array([80, 255, 255])
            thresh = cv2.inRange(hsv, lower_green, upper_green)
            thresh = (thresh / 255).astype(np.uint8)
            
            # Morphological processing
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # Contour detection
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
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
                depth_value_mm = 0
                point_3d = None
                has_valid_depth = False
                
                if self.current_depth is not None and self.tf_handler:
                    try:
                        if cx < self.current_depth.shape[1] and cy < self.current_depth.shape[0]:
                            depth_value_mm = int(self.current_depth[cy, cx])
                            
                            if depth_value_mm > 0 and 100 <= depth_value_mm <= 2000:
                                has_valid_depth = True
                                point_3d = self.tf_handler.pixel_to_3d(cx, cy, depth_value_mm)
                    except:
                        pass
                
                if not has_valid_depth:
                    continue
                
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
                return "No valid leaves detected (after depth filtering)", None, None
            
            result = f"Detected {num_detected_leaves} leaves"
            leaf_data = {
                'num_leaves': num_detected_leaves,
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates
            }
            
            return result, leaf_data, bounding_boxes
            
        except Exception as e:
            self.get_logger().error(f'✗ PlantCV detection error: {str(e)}')
            return f"Detection error: {str(e)}", None, None
    
    def draw_annotations(self, display_frame, leaf_coordinates):
        """Draw annotations on image"""
        if display_frame is None:
            return None
        
        # Always return a copy of the frame (with or without annotations)
        annotated = display_frame.copy()
        
        if not leaf_coordinates:
            # No leaves detected, just return original frame
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
                
                # Draw label
                label = f'Leaf {obj_id}'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5),
                            (x + label_size[0] + 5, label_y + 5), color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Draw center point
                cx = leaf['center']['x']
                cy = leaf['center']['y']
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                cv2.circle(annotated, (cx, cy), 15, color, 2)
                
                # Draw 3D coordinates if available
                point_3d = leaf.get('point_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Display total count
            total_text = f"Leaves: {len(leaf_coordinates)}"
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return annotated
            
        except Exception as e:
            self.get_logger().error(f'✗ Annotation drawing error: {str(e)}')
            return display_frame.copy()

