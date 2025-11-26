#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standalone Leaf Detection and Display Script
No ROS2 dependency, reads images directly from RealSense camera for leaf detection
"""

import cv2
import numpy as np
from plantcv import plantcv as pcv
import pyrealsense2 as rs
from datetime import datetime
import argparse
import sys


class StandaloneLeafDetector:
    """Standalone leaf detector, no ROS2 dependency"""
    
    def __init__(self, min_area=2000, detect_yellow=True, yellow_ratio_threshold=0.05, 
                 yellow_hsv_lower=None, yellow_hsv_upper=None, detect_blue_box=True,
                 blue_min_area=3000, blue_hsv_lower=None, blue_hsv_upper=None):
        """
        Initialize detector
        
        Args:
            min_area: Minimum leaf area threshold
            detect_yellow: Whether to detect yellow regions (for detecting leaves with yellow tape)
            yellow_ratio_threshold: Yellow ratio threshold (default 0.05, i.e., 5%, lower for easier detection)
            yellow_hsv_lower: HSV color range lower bound (default [20, 100, 100], stricter for bright yellow tape)
            yellow_hsv_upper: HSV color range upper bound (default [30, 255, 255])
            detect_blue_box: Whether to detect blue boxes (default True)
            blue_min_area: Blue box minimum area threshold (default 3000)
            blue_hsv_lower: Blue HSV color range lower bound (default [100, 50, 50])
            blue_hsv_upper: Blue HSV color range upper bound (default [130, 255, 255])
        """
        self.min_area = min_area
        self.detect_yellow = detect_yellow
        self.yellow_ratio_threshold = yellow_ratio_threshold
        
        # HSV color range parameters (configurable, stricter defaults for detecting real yellow tape)
        # Yellow tape is usually bright yellow with high saturation and brightness
        self.yellow_hsv_lower = yellow_hsv_lower if yellow_hsv_lower is not None else [20, 100, 100]
        self.yellow_hsv_upper = yellow_hsv_upper if yellow_hsv_upper is not None else [30, 255, 255]
        
        # Blue box detection parameters
        self.detect_blue_box = detect_blue_box
        self.blue_min_area = blue_min_area
        # Blue in HSV: H value 100-130 (blue range), high S and V values
        # Using adjusted parameters: S Lower=147 to reduce false positives
        self.blue_hsv_lower = blue_hsv_lower if blue_hsv_lower is not None else [100, 147, 50]
        self.blue_hsv_upper = blue_hsv_upper if blue_hsv_upper is not None else [130, 255, 255]
        
        # PlantCV settings
        pcv.params.debug = None  # Disable debug output
        
        # Camera intrinsics (will be obtained from RealSense)
        self.intrinsics = None
        
        print('✓ Standalone leaf detector initialized')
        if self.detect_yellow:
            print(f'  ✓ Yellow tape detection enabled (threshold={self.yellow_ratio_threshold:.3f}, '
                  f'HSV range: {self.yellow_hsv_lower} - {self.yellow_hsv_upper})')
        if self.detect_blue_box:
            print(f'  ✓ Blue box detection enabled (min_area={self.blue_min_area}, '
                  f'HSV range: {self.blue_hsv_lower} - {self.blue_hsv_upper})')
    
    def setup_camera(self):
        """Setup RealSense camera"""
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Enable color and depth streams
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Create aligner (align depth to color)
        align_to = rs.align(rs.stream.color)
        self.align = align_to
        
        # Start streams
        profile = self.pipeline.start(config)
        
        # Get color stream intrinsics
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.intrinsics = color_profile.get_intrinsics()
        
        print(f'✓ RealSense camera initialized')
        print(f'  Resolution: {self.intrinsics.width}x{self.intrinsics.height}')
        print(f'  Intrinsics: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}')
        
        return True
    
    def get_frames(self):
        """Get synchronized color and depth frames"""
        try:
            frames = self.pipeline.wait_for_frames()
            
            # Align depth frame to color frame
            aligned_frames = self.align.process(frames)
            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                return None, None
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
        except Exception as e:
            print(f'✗ Failed to get frames: {e}')
            return None, None
    
    def pixel_to_3d(self, pixel_u, pixel_v, depth_value_mm):
        """
        Convert pixel coordinates and depth value to 3D coordinates (camera coordinate system)
        
        Args:
            pixel_u: Pixel u coordinate
            pixel_v: Pixel v coordinate
            depth_value_mm: Depth value (mm)
        
        Returns:
            3D coordinate tuple (x, y, z) in meters, or None if failed
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
            print(f'✗ 3D deprojection error: {e}')
            return None
    
    def measure_box_dimensions_3d(self, bbox_2d, depth_image, mask=None, sample_step=2):
        """
        Measure actual 3D dimensions of box using dense depth sampling (high precision method)
        
        This method obtains accurate dimensions by densely sampling 3D points and calculating 3D bounding box:
        1. Densely sample 3D points within detection region
        2. Use statistical methods to remove outliers
        3. Calculate 3D bounding box to get length/width/height
        
        Args:
            bbox_2d: 2D bounding box (x_min, y_min, x_max, y_max) in original image coordinates
            depth_image: Depth image (numpy array, unit: mm)
            mask: Optional mask (if provided, only sample points within mask)
            sample_step: Sampling step (pixels), default 2 (denser sampling for higher precision)
        
        Returns:
            dict: {
                'width': width (meters, X direction),
                'height': height (meters, Y direction),
                'depth': depth (meters, Z direction, distance to camera),
                'min_point': minimum boundary point (x, y, z),
                'max_point': maximum boundary point (x, y, z),
                'num_samples': number of sample points,
                'center_3d': 3D center point (x, y, z)
            } or None (if failed)
        """
        if depth_image is None or self.intrinsics is None:
            return None
        
        x_min, y_min, x_max, y_max = bbox_2d
        
        # Ensure coordinates are within image bounds
        x_min = max(0, int(x_min))
        y_min = max(0, int(y_min))
        x_max = min(depth_image.shape[1], int(x_max))
        y_max = min(depth_image.shape[0], int(y_max))
        
        if x_max <= x_min or y_max <= y_min:
            return None
        
        # Method 1: Dense sampling of 3D points
        points_3d = []
        valid_depths = []
        
        # Dense sampling within bounding box (sample_step=2 means sample every 2 pixels)
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
                
                # Filter invalid depth
                if depth_mm <= 0 or depth_mm > 5000:
                    continue
                
                # Convert to 3D coordinates
                point_3d = self.pixel_to_3d(x, y, depth_mm)
                if point_3d is not None:
                    points_3d.append(point_3d)
                    valid_depths.append(depth_mm)
        
        if len(points_3d) < 8:  # Need at least 8 points for accurate size estimation
            return None
        
        # Method 2: Use statistical methods to remove outliers (improve robustness)
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
        
        # Method 3: Calculate 3D bounding box
        # Find min/max X, Y, Z values of all points
        min_point = filtered_points.min(axis=0)  # [min_x, min_y, min_z]
        max_point = filtered_points.max(axis=0)   # [max_x, max_y, max_z]
        center_3d = (min_point + max_point) / 2
        
        # Calculate dimensions (in camera coordinate system)
        # X direction = left/right (width)
        # Y direction = up/down (height)
        # Z direction = front/back (depth, distance to camera)
        width = float(max_point[0] - min_point[0])   # X direction
        height = float(max_point[1] - min_point[1])   # Y direction
        depth = float(max_point[2] - min_point[2])    # Z direction
        
        # If depth (Z direction) is too small, may only see one face, use statistical estimation
        if depth < 0.01:  # Less than 1cm, may be planar view
            # Use depth standard deviation as depth estimate
            z_values = filtered_points[:, 2]
            depth = float(np.std(z_values)) * 2  # Use 2x std as depth estimate
            if depth < 0.01:
                depth = 0.1  # If still too small, use default
        
        return {
            'width': width,
            'height': height,
            'depth': depth,
            'min_point': tuple(min_point),
            'max_point': tuple(max_point),
            'num_samples': len(filtered_points),
            'center_3d': tuple(center_3d)
        }
    
    def detect_leaves(self, cv_image, depth_image, frame_count=0):
        """
        Use PlantCV to detect leaves
        
        Args:
            cv_image: BGR image
            depth_image: Depth image (16-bit, in mm)
            frame_count: Frame count (for debugging)
        
        Returns:
            (detection_result_string, leaf_data_dict, bounding_boxes_list, debug_images_dict)
        """
        debug_images = {}  # Store images from intermediate processing steps
        blue_box_coordinates = []  # Initialize blue box coordinate list
        
        try:
            h, w = cv_image.shape[:2]
            
            # Save original image
            debug_images['original'] = cv_image.copy()
            
            # Slight cropping
            crop_img = pcv.crop(img=cv_image, x=20, y=20, h=h-40, w=w-40)
            debug_images['cropped'] = crop_img.copy()
            
            # HSV color space - First detect green leaves
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            debug_images['hsv'] = hsv.copy()
            
            # Step 1: Detect green leaves
            lower_green = np.array([40, 60, 40])
            upper_green = np.array([80, 255, 255])
            thresh_green = cv2.inRange(hsv, lower_green, upper_green)
            
            # Step 2: Detect yellow tape region（Full image detection for later analysis within leaf regions）
            thresh_yellow_full = np.zeros_like(thresh_green)
            if self.detect_yellow:
                # Use configurable HSV range，Default range is wider to handle different lighting conditions
                lower_yellow = np.array(self.yellow_hsv_lower, dtype=np.uint8)
                upper_yellow = np.array(self.yellow_hsv_upper, dtype=np.uint8)
                thresh_yellow_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # Use stricter morphological operations，Remove small noise points
                # First open operation to remove small noise，Then close operation to connect nearby regions
                kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_OPEN, kernel_small, iterations=1)
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_CLOSE, kernel_small, iterations=1)
                
                # Use LAB color space as supplement，But use stricter range
                # Only detect real bright yellow tape, avoid detecting yellow parts of leaves themselves
                lab = cv2.cvtColor(crop_img, cv2.COLOR_BGR2LAB)
                # Yellow tape is usually bright yellow：L is high (brightness), a is low (greenish), b is high (yellowish)
                # Use stricter range，Avoid false detection of yellow at leaf edges
                lab_yellow_mask = np.zeros_like(thresh_yellow_hsv, dtype=np.uint8)
                # Stricter conditions：L>110 (bright), a<140 (greenish), b>150 (yellowish)
                lab_yellow_mask[(lab[:,:,0] > 110) & (lab[:,:,1] < 140) & (lab[:,:,2] > 150)] = 255
                
                # Merge HSV and LAB detection results（Prioritize OR operation for more sensitive detection）
                # Prioritize OR operation for more sensitive detection (easier to detect yellow tape)
                thresh_yellow_full = cv2.bitwise_or(thresh_yellow_hsv, lab_yellow_mask)
                
                # Morphological processing to remove small noise
                kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_OPEN, kernel_medium, iterations=1)
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
            
            # First use green detection for initial processing
            thresh = thresh_green.copy()
            thresh_binary = (thresh / 255).astype(np.uint8)
            
            # Step 2.5: Detect blue boxes（Independent of leaf detection）
            thresh_blue_full = np.zeros_like(thresh_green)
            blue_box_coordinates = []
            if self.detect_blue_box:
                # Use HSV color space to detect blue
                # Blue in HSV：H value between 100-130 (blue range), high S and V values
                lower_blue_hsv = np.array(self.blue_hsv_lower, dtype=np.uint8)
                upper_blue_hsv = np.array(self.blue_hsv_upper, dtype=np.uint8)
                thresh_blue_hsv = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
                
                # Morphological processing：Remove small noise，Connect nearby regions
                # Use smaller kernel to avoid destroying blue box shape
                kernel_blue_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                kernel_blue_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
                
                # First close operationConnect nearby regions，Then open operationRemove small noise
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_hsv, cv2.MORPH_CLOSE, kernel_blue_medium, iterations=2)
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_morph, cv2.MORPH_OPEN, kernel_blue_small, iterations=1)
                
                thresh_blue_full = thresh_blue_morph
                
                # Detect blue box contours（Including all levels to detect multiple faces）
                # Use RETR_TREE to get all contours including internal contours (may be different faces of box)
                blue_contours, blue_hierarchy = cv2.findContours(thresh_blue_full, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # Debug info：Show number of detected contours
                if frame_count <= 5:
                    print(f'  🔍 Blue detection: found {len(blue_contours)} Contours (HSV range: {self.blue_hsv_lower}-{self.blue_hsv_upper}, min area>{self.blue_min_area})')
                
                # Collect all valid blue regions（May be different faces of box）
                valid_blue_regions = []
                
                # Filter blue box contours（Detect all visible faces）
                for idx, blue_cnt in enumerate(blue_contours):
                    area = cv2.contourArea(blue_cnt)
                    
                    # Lower minimum area for single face，Because we want to detect multiple faces
                    min_face_area = max(500, self.blue_min_area * 0.3)  # Single face is at least 30% of total area or 500 pixels
                    if area < min_face_area:
                        continue
                    
                    # Calculate bounding box
                    x, y, w_rect, h_rect = cv2.boundingRect(blue_cnt)
                    if w_rect == 0 or h_rect == 0:
                        continue
                    
                    # Calculate rectangularity（Ratio of contour area to bounding box area）
                    bbox_area = w_rect * h_rect
                    if bbox_area == 0:
                        continue
                    rect_ratio = area / bbox_area
                    
                    # Calculate aspect ratio
                    aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                    
                    # Relaxed filtering conditions（Because it may be a side face of the box, shape may not be perfectly regular）
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
                    
                    # Get depth value and 3D coordinates（Sample multiple points for more accurate depth）
                    depth_value_mm = 0
                    point_3d = None
                    depth_samples = []
                    
                    if depth_image is not None:
                        try:
                            # Sample multiple points within contour region
                            # Note：Contour coordinates are relative to cropped image, need to convert to original image coordinates
                            mask = np.zeros((crop_img.shape[0], crop_img.shape[1]), dtype=np.uint8)
                            cv2.drawContours(mask, [blue_cnt], -1, 255, -1)
                            
                            # Convert to original image coordinates（Depth image is original size）
                            depth_x = x + 20
                            depth_y = y + 20
                            depth_x_end = min(depth_image.shape[1], depth_x + w_rect)
                            depth_y_end = min(depth_image.shape[0], depth_y + h_rect)
                            depth_x_start = max(0, depth_x)
                            depth_y_start = max(0, depth_y)
                            
                            # Sample depth values within mask region
                            mask_roi = mask[y:y+h_rect, x:x+w_rect]
                            depth_roi = depth_image[depth_y_start:depth_y_end, depth_x_start:depth_x_end]
                            
                            # Resize mask ROITo match depth ROI
                            if mask_roi.shape != depth_roi.shape:
                                mask_roi_resized = cv2.resize(mask_roi, (depth_roi.shape[1], depth_roi.shape[0]))
                            else:
                                mask_roi_resized = mask_roi
                            
                            valid_depths = depth_roi[(mask_roi_resized > 0) & (depth_roi > 0)]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.median(valid_depths))  # Use median for stability
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                                    depth_samples.append(depth_value_mm)
                        except Exception as e:
                            if frame_count <= 3:
                                print(f'  Blue region depth retrieval error: {e}')
                            pass
                    
                    # Save blue region info（May be one face of the box）
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
                
                # Group regions by depth and positionAs the same box
                # If multiple regions have similar depth and position, they may belong to the same box
                blue_box_groups = []
                depth_tolerance = 50  # Depth tolerance (mm)
                position_tolerance = 100  # Position tolerance（Pixels）
                
                for region in valid_blue_regions:
                    if region['point_3d'] is None:
                        # If no depth info，Each region becomes its own group
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
                        
                        # If both depth and position are close，Join this group
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
                    
                    # Calculate merged bounding box（Containing all faces）
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
                    if avg_depth_mm > 0 and depth_image is not None:
                        try:
                            merged_point_3d = self.pixel_to_3d(merged_cx, merged_cy, avg_depth_mm)
                        except:
                            pass
                    
                    # Calculate box 3D dimensions（Using high precision dense sampling method）
                    box_3d_size = None
                    measured_dimensions = None
                    if depth_image is not None and merged_point_3d:
                        # Use dense depth sampling method to calculate precise 3D dimensions
                        bbox_for_measurement = (
                            all_x_min,  # x_min
                            all_y_min,  # y_min
                            all_x_max,  # x_max
                            all_y_max   # y_max
                        )
                        
                        # Create blue region mask（Optional, for more precise sampling）
                        blue_mask = None
                        try:
                            # Create mask: within merged bounding box, only sample blue regions
                            blue_mask = np.zeros_like(depth_image, dtype=np.uint8)
                            for region in group:
                                x_reg, y_reg, w_reg, h_reg = region['bbox']
                                # Draw each face region（Simple rectangular mask）
                                # Note：Using simple rectangle here, use contours if more precision needed
                                cv2.rectangle(blue_mask, 
                                            (x_reg, y_reg), 
                                            (x_reg + w_reg, y_reg + h_reg), 
                                            255, -1)
                        except:
                            blue_mask = None
                        
                        # Use high precision method to measure
                        measured_dimensions = self.measure_box_dimensions_3d(
                            bbox_for_measurement,
                            depth_image,
                            mask=blue_mask,
                            sample_step=2  # Sample every 2 pixels, balancing precision and speed
                        )
                        
                        if measured_dimensions:
                            box_3d_size = {
                                'width': measured_dimensions['width'],
                                'height': measured_dimensions['height'],
                                'depth': measured_dimensions['depth']
                            }
                            # If measurement results available, use more precise center point
                            if 'center_3d' in measured_dimensions:
                                merged_point_3d = measured_dimensions['center_3d']
                        else:
                            # If high precision method fails，Fallback to simple estimation
                            depth_m = avg_depth_mm * 0.001
                            pixel_size = depth_m / self.intrinsics.fx if self.intrinsics else 0.001
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
                        'faces': group  # Save all face information
                    }
                    blue_box_coordinates.append(blue_box_info)
                    
                    # Debug info
                    if frame_count <= 5:
                        method_used = "Dense sampling (high precision)" if measured_dimensions else "Simple estimation (fallback)"
                        print(f'    ✓ Blue box {blue_box_idx}: {len(group)}faces, total area={merged_area:.0f}, '
                              f'merged size={merged_w}x{merged_h}, depth={avg_depth_mm}mm')
                        if merged_point_3d:
                            print(f'      3D position: X={merged_point_3d[0]:.3f}m, Y={merged_point_3d[1]:.3f}m, Z={merged_point_3d[2]:.3f}m')
                        if box_3d_size:
                            print(f'      3D dimensions: width={box_3d_size["width"]:.3f}m, '
                                  f'height={box_3d_size["height"]:.3f}m, '
                                  f'depth={box_3d_size["depth"]:.3f}m [{method_used}]')
                            if measured_dimensions and 'num_samples' in measured_dimensions:
                                print(f'      Sample points: {measured_dimensions["num_samples"]} (Step=2Pixels)')
                    
                    blue_box_idx += 1
                
                # Debug info
                if len(blue_box_coordinates) > 0:
                    if frame_count <= 5 or frame_count % 30 == 0:
                        print(f'  📦 Detected {len(blue_box_coordinates)} blue boxes')
                elif frame_count <= 5:
                    print(f'  ⚠️ No blue boxes detected (check HSV range and min area settings)')
            
            # Create debug image：Show green、yellow and blue detection results
            debug_thresh = np.zeros((crop_img.shape[0], crop_img.shape[1], 3), dtype=np.uint8)
            debug_thresh[:, :, 1] = thresh_green  # Green channelShow green detection
            debug_thresh[:, :, 0] = thresh_yellow_full  # Blue channelShow yellow detection
            debug_thresh[:, :, 2] = thresh_blue_full  # Red channelShow blue detection
            debug_images['threshold'] = debug_thresh
            
            # Morphological processing
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh_morph = cv2.morphologyEx(thresh_binary, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh_morph = cv2.morphologyEx(thresh_morph, cv2.MORPH_OPEN, kernel, iterations=1)
            debug_images['morphology'] = cv2.cvtColor(thresh_morph * 255, cv2.COLOR_GRAY2BGR)
            
            # Contour detection
            contours, _ = cv2.findContours(thresh_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw all contours (for debugging)
            contour_img = crop_img.copy()
            cv2.drawContours(contour_img, contours, -1, (0, 255, 255), 2)
            debug_images['contours_all'] = contour_img.copy()
            
            if len(contours) == 0:
                # Even without contours, also save empty valid contours image
                debug_images['contours_valid'] = crop_img.copy()
                # Even without leaves，May still have blue boxes
                if len(blue_box_coordinates) > 0:
                    result = f"Detected {len(blue_box_coordinates)} blue boxes（No leaves）"
                    moveit_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id="world")
                    leaf_data = {
                        'num_leaves': 0,
                        'num_blue_boxes': len(blue_box_coordinates),
                        'timestamp': datetime.now().isoformat(),
                        'coordinates': [],
                        'blue_boxes': blue_box_coordinates,
                        'moveit_collision_objects': moveit_objects
                    }
                    return result, leaf_data, [], debug_images
                return "No leaves detected", None, None, debug_images
            
            # Filter parameters
            min_area_threshold = self.min_area if self.min_area > 0 else 2000
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
            
            # Draw valid contours (after contour filtering)
            valid_contour_img = crop_img.copy()
            cv2.drawContours(valid_contour_img, valid_contours, -1, (0, 255, 0), 2)
            
            # Add text showing contour count (bottom right, non-overlapping)
            h_img, w_img = valid_contour_img.shape[:2]
            text = f'Valid: {len(valid_contours)}'
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            text_x = w_img - text_size[0] - 10
            text_y = h_img - 10
            cv2.rectangle(valid_contour_img, (text_x - 5, text_y - text_size[1] - 5),
                         (text_x + text_size[0] + 5, text_y + 5), (0, 0, 0), -1)
            cv2.putText(valid_contour_img, text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            debug_images['contours_valid'] = valid_contour_img.copy()
            
            if len(valid_contours) == 0:
                # Even without valid leaves，May still have blue boxes
                if len(blue_box_coordinates) > 0:
                    result = f"Detected {len(blue_box_coordinates)} blue boxes（No valid leaves）"
                    moveit_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id="world")
                    leaf_data = {
                        'num_leaves': 0,
                        'num_blue_boxes': len(blue_box_coordinates),
                        'timestamp': datetime.now().isoformat(),
                        'coordinates': [],
                        'blue_boxes': blue_box_coordinates,
                        'moveit_collision_objects': moveit_objects
                    }
                    return result, leaf_data, [], debug_images
                return "No valid leaves detected", None, None, debug_images
            
            # Step 3: For each detected green leaf, detect yellow tape within its region
            # Also detect independent yellow regions (if shape matches leaf characteristics)
            leaf_coordinates = []
            bounding_boxes = []
            leaf_idx = 1
            
            # Debug info
            if frame_count <= 5:
                print(f'  Start processing: {len(valid_contours)} Green contours')
            
            # Process green leaf contours
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
                
                # Get depth value and 3D coordinates（Use larger region for stability）
                depth_value_mm = 0
                point_3d = None
                has_valid_depth = False
                
                if depth_image is not None:
                    try:
                        if cx < depth_image.shape[1] and cy < depth_image.shape[0]:
                            # Use 5x5 regionAverage value for stability
                            y_start = max(0, cy - 2)
                            y_end = min(depth_image.shape[0], cy + 3)
                            x_start = max(0, cx - 2)
                            x_end = min(depth_image.shape[1], cx + 3)
                            
                            depth_region = depth_image[y_start:y_end, x_start:x_end]
                            # Filter out invalid depth values (0 values)
                            valid_depths = depth_region[depth_region > 0]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.mean(valid_depths))
                                
                                # Further relax depth range check
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    has_valid_depth = True
                                    point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                            else:
                                # If no valid depth, trying to use depth within contour region
                                # Sample multiple points within contour bounding box
                                bbox_x_start = max(0, x + 20 - 10)
                                bbox_x_end = min(depth_image.shape[1], x + 20 + w_rect + 10)
                                bbox_y_start = max(0, y + 20 - 10)
                                bbox_y_end = min(depth_image.shape[0], y + 20 + h_rect + 10)
                                
                                bbox_depth_region = depth_image[bbox_y_start:bbox_y_end, bbox_x_start:bbox_x_end]
                                bbox_valid_depths = bbox_depth_region[bbox_depth_region > 0]
                                
                                if len(bbox_valid_depths) > 0:
                                    depth_value_mm = int(np.mean(bbox_valid_depths))
                                    if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                        has_valid_depth = True
                                        point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                    except Exception as e:
                        # Debug info：Only print on first frame
                        if frame_count <= 3 and leaf_idx == 1:
                            print(f'  Depth retrieval error: {e}')
                        pass
                
                # If depth filtering fails, still keep contour (for debugging, at least can see detection box)
                # But mark as no depth info
                if not has_valid_depth:
                    # Debug info：Show why it was filtered
                    if frame_count <= 5:  # Print debug info for first 5 frames
                        print(f'  Contour {leaf_idx} depth invalid: cx={cx}, cy={cy}, depth={depth_value_mm}mm, trying to use contour region')
                    
                    # Even if depth invalid, keep contour (but mark depth as 0)
                    # So at least can see detection box
                    depth_value_mm = 0
                    point_3d = None
                    # Do not continue, keep processing this contour
                
                # Check if this leaf region contains yellow tape
                has_yellow_tape = False
                yellow_ratio = 0.0
                
                if self.detect_yellow and thresh_yellow_full is not None:
                    # Create leaf region mask（Using full contour，Not bounding box）
                    # This allows more accurate detection of yellow areas inside leaf
                    leaf_mask = np.zeros_like(thresh_green, dtype=np.uint8)
                    cv2.drawContours(leaf_mask, [cnt], -1, 255, -1)
                    
                    # Calculate directly on full mask，Not bounding box region
                    # This avoids bounding box potentially including non-leaf areas
                    leaf_pixels = np.sum(leaf_mask > 0)
                    yellow_pixels = np.sum((leaf_mask > 0) & (thresh_yellow_full > 0))
                    
                    if leaf_pixels > 0:
                        yellow_ratio = yellow_pixels / leaf_pixels
                        
                        # Additional validation：Check connectivity and size of yellow region
                        # Real yellow tape should be continuous、With certain size
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
                        
                        # Lower minimum area requirement：At least 50 pixels or 0.5% of leaf area (easier detection)
                        min_yellow_area_threshold = max(50, leaf_pixels * 0.005)
                        
                        # Use OR condition：Satisfy either ratio threshold or minimum area requirement（Easier detection）
                        if (yellow_ratio >= self.yellow_ratio_threshold or 
                            max_yellow_area >= min_yellow_area_threshold):
                            has_yellow_tape = True
                            
                            # Debug log：Record detected yellow tape info
                            if frame_count <= 5 or frame_count % 30 == 0:
                                print(f'  🎯 Leaf {leaf_idx}: Detected yellow tape - '
                                      f'yellow_ratio={yellow_ratio:.4f} '
                                      f'(Threshold={self.yellow_ratio_threshold:.4f}), '
                                      f'YellowPixels={yellow_pixels}/{leaf_pixels}, '
                                      f'Largest connected region={max_yellow_area:.0f}Pixels')
                        else:
                            # Debug log：Record validation failure
                            if yellow_ratio > 0.01 and (frame_count <= 5 or frame_count % 30 == 0):
                                reason = []
                                if yellow_ratio < self.yellow_ratio_threshold:
                                    reason.append(f'Ratio insufficient({yellow_ratio:.4f}<{self.yellow_ratio_threshold:.4f})')
                                if max_yellow_area < min_yellow_area_threshold:
                                    reason.append(f'Area too small({max_yellow_area:.0f}<{min_yellow_area_threshold:.0f})')
                                print(f'  ⚠️ Leaf {leaf_idx}: Detected yellow but validation failed - '
                                      f'yellow_ratio={yellow_ratio:.4f}, '
                                      f'Largest connected region={max_yellow_area:.0f}Pixels, '
                                      f'Reason: {", ".join(reason)}')
                    else:
                        # Debug: If leaf region is empty, record warning
                        if frame_count <= 5:
                            print(f'  ⚠️ Leaf {leaf_idx}: Leaf region mask is empty，Cannot detect yellow tape '
                                  f'(bbox: x={x}, y={y}, w={w_rect}, h={h_rect})')
                
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
            
            # Debug info
            if frame_count <= 5:
                print(f'  After depth filtering: {num_detected_leaves} Leaves（Original: {len(valid_contours)} Contours）')
            
            # Summarize yellow tape detection results
            yellow_tape_count = sum(1 for leaf in leaf_coordinates if leaf.get('has_yellow_tape', False))
            if yellow_tape_count > 0:
                print(f'  📊 Detection summary: Total{num_detected_leaves}Leaves, '
                      f'Of which {yellow_tape_count}leaves detected with yellow tape')
                # List all leaves with yellow tape
                for leaf in leaf_coordinates:
                    if leaf.get('has_yellow_tape', False):
                        print(f'    ✓ Leaf {leaf["id"]}: yellow_ratio={leaf.get("yellow_ratio", 0):.4f}')
            
            if num_detected_leaves == 0:
                # Even if depth filtering fails，Also show contours（For debugging）
                # Create result image showing all valid contours
                result_img = crop_img.copy()
                cv2.drawContours(result_img, valid_contours, -1, (0, 255, 255), 2)  # Yellow contours
                
                # Add debug info
                h_img, w_img = result_img.shape[:2]
                cv2.putText(result_img, f'No depth info, showing contours', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(result_img, f'Valid contours: {len(valid_contours)}', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Add number to each contour
                for idx, cnt in enumerate(valid_contours):
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00']) + 20
                        cy = int(M['m01'] / M['m00']) + 20
                        cv2.putText(result_img, f'{idx+1}', (cx-10, cy),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                debug_images['result'] = result_img
                # Even without valid leaves，May still have blue boxes
                if len(blue_box_coordinates) > 0:
                    result = f"Detected {len(blue_box_coordinates)} blue boxes（Leaves after depth filtering）"
                    moveit_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id="world")
                    leaf_data = {
                        'num_leaves': 0,
                        'num_blue_boxes': len(blue_box_coordinates),
                        'timestamp': datetime.now().isoformat(),
                        'coordinates': [],
                        'blue_boxes': blue_box_coordinates,
                        'moveit_collision_objects': moveit_objects
                    }
                    return result, leaf_data, [], debug_images
                return "No valid leaves detected（After depth filtering）", None, None, debug_images
            
            result = f"Detected {num_detected_leaves} Leaves"
            if len(blue_box_coordinates) > 0:
                result += f", {len(blue_box_coordinates)} blue boxes"
            
            # Generate MoveIt format coordinates
            moveit_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id="world")
            
            leaf_data = {
                'num_leaves': num_detected_leaves,
                'num_blue_boxes': len(blue_box_coordinates),
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates,
                'blue_boxes': blue_box_coordinates,
                'moveit_collision_objects': moveit_objects  # Add MoveIt format coordinates
            }
            
            # Create result image (will be updated with annotations in run() method)
            # For now, use valid_contour_img as placeholder
            debug_images['result'] = valid_contour_img.copy()
            
            return result, leaf_data, bounding_boxes, debug_images
            
        except Exception as e:
            print(f'✗ PlantCV detection error: {str(e)}')
            import traceback
            traceback.print_exc()
            return f"Detection error: {str(e)}", None, None, debug_images
    
    def draw_annotations(self, display_frame, leaf_coordinates, blue_box_coordinates=None):
        """
        Draw annotations on image
        
        Args:
            display_frame: Image to draw on
            leaf_coordinates: Leaf coordinate list
            blue_box_coordinates: Blue box coordinate list（Optional）
        
        Returns:
            Annotated image
        """
        if display_frame is None:
            return None
        
        # Always return frame copy（With or without annotations）
        annotated = display_frame.copy()
        
        if blue_box_coordinates is None:
            blue_box_coordinates = []
        
        if not leaf_coordinates and not blue_box_coordinates:
            # No objects detected, only return original frame
            return annotated
        
        # Draw annotations for detected leaves
        try:
            colors = [
                (255, 0, 0),      # Blue
                (0, 255, 0),       # Green
                (0, 0, 255),       # Red
                (255, 255, 0),     # Cyan
                (255, 0, 255),     # Magenta
                (0, 255, 255),     # Yellow
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
                
                # Draw label（If has tape，Add marker）
                label = f'Leaf {obj_id}'
                has_tape = leaf.get('has_yellow_tape', False)
                if has_tape:
                    label += ' [Tape]'
                
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
                
                # If available，Draw 3D coordinates
                point_3d = leaf.get('point_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
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
                
                # Draw merged complete bounding box（Thick line）
                cv2.rectangle(annotated, (x, y), (x_max, y_max), blue_box_color, 3)
                
                # Draw each detected face (thin line)
                num_faces = blue_box.get('num_faces', 1)
                faces = blue_box.get('faces', [])
                if len(faces) > 0:
                    # Draw contour of each face
                    for face_idx, face in enumerate(faces):
                        contour = face.get('contour')
                        if contour is not None:
                            # Adjust contour coordinates（Add crop offset）
                            adjusted_contour = contour + np.array([20, 20])
                            cv2.drawContours(annotated, [adjusted_contour], -1, face_color, 1)
                
                # Draw label（Show face count）
                label = f'Blue Box {obj_id}'
                if num_faces > 1:
                    label += f' ({num_faces} faces)'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5),
                            (x + label_size[0] + 5, label_y + 5), blue_box_color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Draw center point
                cx = blue_box['center']['x']
                cy = blue_box['center']['y']
                cv2.circle(annotated, (cx, cy), 5, blue_box_color, -1)
                cv2.circle(annotated, (cx, cy), 15, blue_box_color, 2)
                
                # If available, draw 3D coordinates and dimensions
                point_3d = blue_box.get('point_3d')
                size_3d = blue_box.get('size_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    if size_3d:
                        coord_text += f" ({size_3d['width']:.2f}x{size_3d['height']:.2f}x{size_3d['depth']:.2f}m)"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, blue_box_color, 1)
            
            # Display total count
            total_text = f"Leaves: {len(leaf_coordinates)}"
            if len(blue_box_coordinates) > 0:
                total_text += f" | Blue Boxes: {len(blue_box_coordinates)}"
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return annotated
            
        except Exception as e:
            print(f'✗ Annotation drawing error: {e}')
            import traceback
            traceback.print_exc()
            return display_frame.copy()
    
    def get_moveit_collision_objects(self, blue_box_coordinates, frame_id="world"):
        """
        Convert blue box coordinates to MoveIt CollisionObject format
        
        Args:
            blue_box_coordinates: Blue box coordinate list
            frame_id: Coordinate frame ID (default: "world"）
        
        Returns:
            List of dictionaries in MoveIt CollisionObject format
        """
        collision_objects = []
        
        for box in blue_box_coordinates:
            point_3d = box.get('point_3d')
            size_3d = box.get('size_3d')
            
            if point_3d is None:
                continue
            
            # Get 3D dimensions, use default if not available
            if size_3d:
                width = size_3d['width']
                height = size_3d['height']
                depth = size_3d['depth']
            else:
                # Use default dimensions (estimated from pixels)
                bbox = box.get('bounding_box', {})
                pixel_w = bbox.get('width', 100)
                pixel_h = bbox.get('height', 100)
                depth_mm = box.get('depth_mm', 1000)
                
                # Estimate 3D dimensions
                depth_m = depth_mm * 0.001
                if self.intrinsics:
                    pixel_size = depth_m / self.intrinsics.fx
                    width = pixel_w * pixel_size
                    height = pixel_h * pixel_size
                    depth = 0.1  # Default depth 10cm
                else:
                    width = 0.1
                    height = 0.1
                    depth = 0.1
            
            # Create CollisionObject format
            collision_obj = {
                'header': {
                    'frame_id': frame_id
                },
                'id': f"blue_box_{box['id']:03d}",
                'operation': [0],  # ADD operation
                'primitives': [{
                    'type': 1,  # BOX type
                    'dimensions': [width, height, depth]
                }],
                'primitive_poses': [{
                    'position': {
                        'x': float(point_3d[0]),
                        'y': float(point_3d[1]),
                        'z': float(point_3d[2])
                    },
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0
                    }
                }]
            }
            
            collision_objects.append(collision_obj)
        
        return collision_objects
    
    def print_moveit_format(self, blue_box_coordinates, frame_id="world"):
        """
        Print MoveIt format coordinates (for ROS2 topic pub command)
        
        Args:
            blue_box_coordinates: Blue box coordinate list
            frame_id: Coordinate frame ID (default: "world"）
        """
        collision_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id)
        
        if len(collision_objects) == 0:
            print("No blue boxes detected")
            return
        
        print("\n" + "="*80)
        print("MoveIt CollisionObject format coordinates:")
        print("="*80)
        
        for i, obj in enumerate(collision_objects):
            pos = obj['primitive_poses'][0]['position']
            dims = obj['primitives'][0]['dimensions']
            
            print(f"\nBlue box {i+1} ({obj['id']}):")
            print(f"  ros2 topic pub --once /obsFromImg moveit_msgs/msg/CollisionObject '{{")
            print(f"    header: {{frame_id: \"{obj['header']['frame_id']}\"}},")
            print(f"    id: \"{obj['id']}\",")
            print(f"    operation: [0],")
            print(f"    primitives: [{{type: 1, dimensions: [{dims[0]:.3f}, {dims[1]:.3f}, {dims[2]:.3f}]}}],")
            print(f"    primitive_poses: [{{position: {{x: {pos['x']:.3f}, y: {pos['y']:.3f}, z: {pos['z']:.3f}}}, orientation: {{w: 1.0}}}}]")
            print(f"  }}'")
        
        print("\n" + "="*80)
    
    def create_debug_mosaic(self, debug_images):
        """
        Create composite image with all processing steps
        
        Args:
            debug_images: Dictionary containing images from each processing step
        
        Returns:
            Composite large image
        """
        # Define image order and labels
        image_order = [
            ('original', '1. Original'),
            ('cropped', '2. Cropped'),
            ('hsv', '3. HSV'),
            ('threshold', '4. Threshold'),
            ('morphology', '5. Morphology'),
            ('contours_all', '6. All Contours'),
            ('contours_valid', '7. Valid Contours'),
            ('result', '8. Final Result')
        ]
        
        # Size of each small image
        tile_width = 320
        tile_height = 240
        
        # Create 3x3 grid (last row may only have 2)
        rows = 3
        cols = 3
        
        # Create large canvas
        mosaic_height = rows * tile_height
        mosaic_width = cols * tile_width
        mosaic = np.zeros((mosaic_height, mosaic_width, 3), dtype=np.uint8)
        
        # Fill each position
        for idx, (key, label) in enumerate(image_order):
            row = idx // cols
            col = idx % cols
            
            if key in debug_images and debug_images[key] is not None:
                img = debug_images[key]
                if img.size > 0:
                    # Resize image
                    resized = cv2.resize(img, (tile_width, tile_height))
                    
                    # Calculate position
                    y_start = row * tile_height
                    y_end = y_start + tile_height
                    x_start = col * tile_width
                    x_end = x_start + tile_width
                    
                    # Place image
                    mosaic[y_start:y_end, x_start:x_end] = resized
                    
                    # Add label
                    cv2.putText(mosaic, label, (x_start + 5, y_start + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(mosaic, label, (x_start + 5, y_start + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        return mosaic
    
    def run(self):
        """Run detection loop"""
        print('\nStarting detection loop...')
        print('Press \'q\' to exit\n')
        
        frame_count = 0
        window_name = 'Leaf Detection - All Processing Steps'
        
        # Create single large window
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 960, 720)  # 3x3Grid, each 320x240
            print('✓ Visualization window created')
            print('   All processing steps will be displayed in one window')
        except Exception as e:
            print(f'⚠ Window creation warning: {e}')
            print('  Continuing, but window may not display')
        
        try:
            while True:
                # Get frames
                color_image, depth_image = self.get_frames()
                
                if color_image is None or depth_image is None:
                    if frame_count == 0:
                        print('⚠ Waiting for camera data...')
                    continue
                
                frame_count += 1
                
                # Detect leaves (pass frame_count for debugging)
                detection_result, leaf_data, bounding_boxes, debug_images = self.detect_leaves(
                    color_image, depth_image, frame_count
                )
                
                # Draw annotations
                if leaf_data:
                    annotated_image = self.draw_annotations(
                        color_image,
                        leaf_data.get('coordinates', []),
                        leaf_data.get('blue_boxes', [])
                    )
                    # Safety check: if draw_annotations returns None, use original image
                    if annotated_image is None:
                        annotated_image = color_image.copy()
                    
                    # Print detection info
                    if frame_count % 30 == 0:  # Print every 30 frames
                        print(f'Frame {frame_count}: {detection_result}')
                        for i, leaf in enumerate(leaf_data.get('coordinates', [])):
                            point_3d = leaf.get('point_3d')
                            if point_3d:
                                print(f'  Leaf {i+1}: X={point_3d[0]:.3f}m, '
                                      f'Y={point_3d[1]:.3f}m, Z={point_3d[2]:.3f}m')
                        
                        # Print blue box details
                        blue_boxes = leaf_data.get('blue_boxes', [])
                        if len(blue_boxes) > 0:
                            print(f'\n{"="*80}')
                            print(f'Blue box detection info (Total {len(blue_boxes)} items):')
                            print(f'{"="*80}')
                            
                            for blue_box in blue_boxes:
                                box_id = blue_box["id"]
                                num_faces = blue_box.get('num_faces', 1)
                                area = blue_box.get('area', 0)
                                bbox = blue_box.get('bounding_box', {})
                                point_3d = blue_box.get('point_3d')
                                size_3d = blue_box.get('size_3d')
                                depth_mm = blue_box.get('depth_mm', 0)
                                
                                print(f'\n📦 Blue box {box_id}:')
                                print(f'  ├─ Basic info:')
                                print(f'  │   ├─ Number of detected faces: {num_faces}')
                                print(f'  │   ├─ total area: {area:.0f} Pixels²')
                                print(f'  │   └─ depth: {depth_mm} mm')
                                
                                if bbox:
                                    print(f'  ├─ 2D bounding box:')
                                    print(f'  │   ├─ Position: ({bbox.get("x", 0)}, {bbox.get("y", 0)})')
                                    print(f'  │   ├─ Size: {bbox.get("width", 0)} x {bbox.get("height", 0)} Pixels')
                                    print(f'  │   └─ Center: ({blue_box.get("center", {}).get("x", 0)}, {blue_box.get("center", {}).get("y", 0)})')
                                
                                if point_3d:
                                    print(f'  ├─ 3D position (Camera coordinate system):')
                                    print(f'  │   ├─ X: {point_3d[0]:.3f} m')
                                    print(f'  │   ├─ Y: {point_3d[1]:.3f} m')
                                    print(f'  │   └─ Z: {point_3d[2]:.3f} m')
                                
                                if size_3d:
                                    print(f'  ├─ 3D dimensions:')
                                    print(f'  │   ├─ width: {size_3d["width"]:.3f} m')
                                    print(f'  │   ├─ height: {size_3d["height"]:.3f} m')
                                    print(f'  │   └─ depth: {size_3d["depth"]:.3f} m')
                                
                                print(f'  └─ MoveIt format:')
                                if point_3d and size_3d:
                                    print(f'      position: {{x: {point_3d[0]:.3f}, y: {point_3d[1]:.3f}, z: {point_3d[2]:.3f}}}')
                                    print(f'      dimensions: [{size_3d["width"]:.3f}, {size_3d["height"]:.3f}, {size_3d["depth"]:.3f}]')
                            
                            print(f'\n{"="*80}')
                            
                            # Print MoveIt format coordinates (ROS2 command)
                            self.print_moveit_format(blue_boxes, frame_id="world")
                else:
                    annotated_image = color_image.copy()
                
                # Save final result to debug image
                debug_images['result'] = annotated_image.copy()
                
                # Check if image is valid
                if annotated_image is None or annotated_image.size == 0:
                    print('⚠ Image invalid, skipping display')
                    continue
                
                # Create composite image
                try:
                    mosaic = self.create_debug_mosaic(debug_images)
                    
                    # Display composite image
                    cv2.imshow(window_name, mosaic)
                    
                    # Print prompt on first display
                    if frame_count == 1:
                        print('✓ Image display started')
                        print('   All processing steps displayed in one window')
                        print('   If window not visible, check：')
                        print('   1. If window is blocked by other windows')
                        print('   2. If using remote SSH (needs X11 forwarding)')
                        print('   3. If DISPLAY environment variable is set correctly')
                except Exception as e:
                    print(f'✗ Failed to display image: {e}')
                    import traceback
                    traceback.print_exc()
                    print('   Tip: If using SSH, use X11 forwarding: ssh -X user@host')
                    break
                
                # Press q to exit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print('\nUser pressed \'q\' key, exiting program')
                    break
                elif key == 27:  # ESC key
                    print('\nUser pressed ESC, exiting program')
                    break
                
        except KeyboardInterrupt:
            print('\nUser interrupted (Ctrl+C)')
        except Exception as e:
            print(f'\n✗ Runtime error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup
            try:
                cv2.destroyAllWindows()
            except:
                pass
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
            print('\n✓ Program exited')


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Standalone leaf detection and display script (no ROS2 dependency)'
    )
    parser.add_argument(
        '--min-area',
        type=float,
        default=2000,
        help='Minimum leaf area threshold (default: 2000)'
    )
    parser.add_argument(
        '--no-yellow',
        action='store_true',
        help='Disable yellow tape detection (only detect green leaves)'
    )
    parser.add_argument(
        '--yellow-threshold',
        type=float,
        default=0.05,
        help='Yellow ratio threshold (default: 0.05, i.e. 5%%, lower for easier yellow tape detection)'
    )
    parser.add_argument(
        '--yellow-hsv-lower',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[20, 100, 100],
        help='HSV color range lower bound (default: 20 100 100, stricter for detecting bright yellow tape)'
    )
    parser.add_argument(
        '--yellow-hsv-upper',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[30, 255, 255],
        help='HSV color range upper bound (default: 30 255 255)'
    )
    parser.add_argument(
        '--no-blue-box',
        action='store_true',
        help='Disable blue box detection (only detect leaves)'
    )
    parser.add_argument(
        '--blue-min-area',
        type=float,
        default=3000,
        help='Blue box minimum area threshold (default: 3000)'
    )
    parser.add_argument(
        '--blue-hsv-lower',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[100, 147, 50],
        help='Blue HSV color range lower bound (default: 100 147 50, higher S value to reduce false positives)'
    )
    parser.add_argument(
        '--blue-hsv-upper',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[130, 255, 255],
        help='Blue HSV color range upper bound (default: 130 255 255)'
    )
    
    args = parser.parse_args()
    
    # Create detector
    detector = StandaloneLeafDetector(
        min_area=args.min_area,
        detect_yellow=not args.no_yellow,
        yellow_ratio_threshold=args.yellow_threshold,
        yellow_hsv_lower=args.yellow_hsv_lower,
        yellow_hsv_upper=args.yellow_hsv_upper,
        detect_blue_box=not args.no_blue_box,
        blue_min_area=args.blue_min_area,
        blue_hsv_lower=args.blue_hsv_lower,
        blue_hsv_upper=args.blue_hsv_upper
    )
    
    # Set up camera
    try:
        detector.setup_camera()
    except Exception as e:
        print(f'✗ Camera initialization failed: {e}')
        print('Please ensure RealSense camera is connected and properly configured')
        sys.exit(1)
    
    # Run detection loop
    detector.run()


if __name__ == '__main__':
    main()

