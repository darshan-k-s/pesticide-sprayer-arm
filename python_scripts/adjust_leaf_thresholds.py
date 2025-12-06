#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Detection Threshold Manual Adjustment Tool
Use sliders to adjust HSV color range and other filter parameters in real-time
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import subprocess
import time


class LeafThresholdAdjuster:
    """Leaf detection threshold adjuster"""
    
    def __init__(self):
        """Initialize parameters"""
        # Green leaf HSV color range (default values) - only keep key parameters
        self.green_h_lower = 40
        self.green_h_upper = 80
        self.green_s_lower = 60
        self.green_s_upper = 255
        self.green_v_lower = 40
        self.green_v_upper = 255
        
        # Yellow ratio threshold (0.0 to 1.0)
        self.yellow_ratio_threshold = 50  # Trackbar value (0-1000, represents 0.0-1.0)
        
        # Area threshold
        self.min_area = 2000
        
        # Fixed parameters (no longer adjustable, using reasonable defaults)
        # Yellow tape HSV range (fixed, using defaults from main script)
        self.yellow_hsv_lower = [20, 100, 100]
        self.yellow_hsv_upper = [30, 255, 255]
        self.morph_close_iter = 2
        self.morph_open_iter = 1
        
        # Enable/disable yellow detection
        self.detect_yellow = True
        
        # Camera and image
        self.pipeline = None
        self.align = None
        self.color_image = None
        self.depth_image = None
        
        # Window names
        self.window_name = 'Leaf Threshold Adjustment'
        self.debug_window = 'Debug: HSV Threshold'
        
        # Frame count for console output
        self.frame_count = 0
        self.last_print_frame = 0
    
    def check_camera_usage(self):
        """Check if camera is being used by other processes"""
        try:
            # Check for ROS2 realsense2_camera processes
            result = subprocess.run(['pgrep', '-f', 'realsense2_camera'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                return True, [pid for pid in pids if pid]
            return False, []
        except Exception:
            return False, []
    
    def setup_camera(self):
        """Setup RealSense camera"""
        try:
            # Create context to check for available devices
            ctx = rs.context()
            devices = ctx.query_devices()
            
            if len(devices) == 0:
                print(' No RealSense devices found')
                print('  Please ensure the camera is connected via USB')
                return False
            
            print(f' Found {len(devices)} RealSense device(s)')
            
            # Create new pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Try to find and use the first available device
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            self.align = rs.align(rs.stream.color)
            
            # Start pipeline with timeout handling
            print('  Starting camera streams...')
            profile = self.pipeline.start(config)
            
            print(' Camera initialized successfully')
            return True
            
        except RuntimeError as e:
            error_msg = str(e)
            print(f' Camera initialization failed: {e}')
            if 'busy' in error_msg.lower() or 'errno=16' in error_msg:
                print('\n  Possible causes:')
                print('    1. Another program is using the camera (e.g., ROS2 realsense2_camera node)')
                print('    2. A previous instance didn\'t close properly')
                print('\n  Solutions:')
                print('    1. Stop ROS2 camera node: pkill -f realsense2_camera')
                print('    2. Check for other processes: ps aux | grep realsense')
                print('    3. Unplug and replug the USB cable')
                print('    4. Wait a few seconds and try again')
            return False
        except Exception as e:
            print(f' Camera initialization failed: {e}')
            print('  Please ensure RealSense camera is connected and not in use by another program')
            return False
    
    def get_frames(self):
        """Get camera frames"""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                return None, None
            
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
        except Exception as e:
            print(f' Failed to get frames: {e}')
            return None, None
    
    def detect_leaves(self, color_image):
        """
        Detect leaves using current thresholds
        
        Returns:
            (result_image, debug_image, valid_leaves_list)
        """
        if color_image is None:
            return None, None, []
        
        # Copy image for drawing
        result_img = color_image.copy()
        
        # Slight cropping (consistent with main script)
        h, w = color_image.shape[:2]
        crop_img = color_image[20:h-20, 20:w-20]
        
        # Convert to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Step 1: Detect green leaves
        lower_green = np.array([
            self.green_h_lower,
            self.green_s_lower,
            self.green_v_lower
        ], dtype=np.uint8)
        upper_green = np.array([
            self.green_h_upper,
            self.green_s_upper,
            self.green_v_upper
        ], dtype=np.uint8)
        
        thresh_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # Morphological processing
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        thresh_morph = cv2.morphologyEx(
            thresh_green, 
            cv2.MORPH_CLOSE, 
            kernel_medium, 
            iterations=self.morph_close_iter
        )
        thresh_morph = cv2.morphologyEx(
            thresh_morph, 
            cv2.MORPH_OPEN, 
            kernel_small, 
            iterations=self.morph_open_iter
        )
        
        # Step 2: Detect yellow tape (if enabled)
        thresh_yellow_full = np.zeros_like(thresh_green)
        if self.detect_yellow:
            lower_yellow = np.array(self.yellow_hsv_lower, dtype=np.uint8)
            upper_yellow = np.array(self.yellow_hsv_upper, dtype=np.uint8)
            
            thresh_yellow_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # Morphological processing for yellow
            thresh_yellow_hsv = cv2.morphologyEx(
                thresh_yellow_hsv, 
                cv2.MORPH_OPEN, 
                kernel_small, 
                iterations=1
            )
            thresh_yellow_hsv = cv2.morphologyEx(
                thresh_yellow_hsv, 
                cv2.MORPH_CLOSE, 
                kernel_small, 
                iterations=1
            )
            
            # LAB color space as supplement
            lab = cv2.cvtColor(crop_img, cv2.COLOR_BGR2LAB)
            lab_yellow_mask = np.zeros_like(thresh_yellow_hsv, dtype=np.uint8)
            lab_yellow_mask[(lab[:,:,0] > 110) & (lab[:,:,1] < 140) & (lab[:,:,2] > 150)] = 255
            
            # Merge HSV and LAB detection results
            thresh_yellow_full = cv2.bitwise_or(thresh_yellow_hsv, lab_yellow_mask)
            
            # Additional morphological processing
            thresh_yellow_full = cv2.morphologyEx(
                thresh_yellow_full, 
                cv2.MORPH_OPEN, 
                kernel_medium, 
                iterations=1
            )
            thresh_yellow_full = cv2.morphologyEx(
                thresh_yellow_full, 
                cv2.MORPH_CLOSE, 
                kernel_medium, 
                iterations=1
            )
        
        # Detect contours
        contours, _ = cv2.findContours(
            thresh_morph, 
            cv2.RETR_TREE, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filter contours
        valid_leaves = []
        yellow_ratio_threshold = self.yellow_ratio_threshold / 1000.0  # Convert trackbar value to ratio
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Area filtering
            if area < self.min_area:
                continue
            
            # Bounding box
            x, y, w_rect, h_rect = cv2.boundingRect(cnt)
            if w_rect == 0 or h_rect == 0:
                continue
            
            # Calculate center point
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00']) + 20
                cy = int(M['m01'] / M['m00']) + 20
            else:
                cx = x + w_rect // 2 + 20
                cy = y + h_rect // 2 + 20
            
            # Check for yellow tape within leaf region (if enabled)
            has_yellow = False
            yellow_ratio = 0.0
            
            if self.detect_yellow:
                # Create mask for this leaf
                leaf_mask = np.zeros_like(thresh_morph)
                cv2.drawContours(leaf_mask, [cnt], -1, 255, -1)
                
                # Count yellow pixels within leaf region
                yellow_in_leaf = cv2.bitwise_and(thresh_yellow_full, leaf_mask)
                yellow_pixels = np.sum(yellow_in_leaf > 0)
                leaf_pixels = np.sum(leaf_mask > 0)
                
                if leaf_pixels > 0:
                    yellow_ratio = yellow_pixels / leaf_pixels
                    if yellow_ratio >= yellow_ratio_threshold:
                        has_yellow = True
            
            valid_leaves.append({
                'id': len(valid_leaves) + 1,
                'contour': cnt,
                'area': area,
                'bbox': (x, y, w_rect, h_rect),
                'center': (cx, cy),
                'has_yellow': has_yellow,
                'yellow_ratio': yellow_ratio
            })
        
        # Draw results and detailed info
        y_offset = 30
        line_height = 25
        
        # Display detected leaf count
        yellow_count = sum(1 for leaf in valid_leaves if leaf['has_yellow'])
        info_text = f"Detected: {len(valid_leaves)} leaves ({yellow_count} with yellow tape)"
        cv2.putText(result_img, info_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        y_offset += line_height + 5
        
        # Draw each leaf and display info
        for leaf in valid_leaves:
            x, y, w_rect, h_rect = leaf['bbox']
            # Adjust coordinates (add crop offset)
            x_abs = x + 20
            y_abs = y + 20
            cx, cy = leaf['center']
            
            # Choose color based on yellow tape detection
            if leaf['has_yellow']:
                color = (0, 255, 255)  # Yellow (BGR) for leaves with yellow tape
            else:
                color = (0, 255, 0)  # Green (BGR) for normal leaves
            
            # Draw bounding box
            cv2.rectangle(result_img, (x_abs, y_abs), 
                         (x_abs + w_rect, y_abs + h_rect), color, 2)
            
            # Draw contour
            adjusted_contour = leaf['contour'] + np.array([20, 20])
            cv2.drawContours(result_img, [adjusted_contour], -1, color, 1)
            
            # Draw center point
            cv2.circle(result_img, (cx, cy), 5, color, -1)
            cv2.circle(result_img, (cx, cy), 15, color, 2)
            
            # Draw label
            label = f'Leaf {leaf["id"]}'
            if leaf['has_yellow']:
                label += ' [Y]'
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            label_y = max(y_abs - 5, label_size[1] + 5)
            
            cv2.rectangle(result_img, (x_abs, label_y - label_size[1] - 5),
                         (x_abs + label_size[0] + 5, label_y + 5), color, -1)
            cv2.putText(result_img, label, (x_abs + 2, label_y - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # Display leaf details on left side of image
            if y_offset < h - 50:
                yellow_info = f" Y:{leaf['yellow_ratio']:.3f}" if self.detect_yellow else ""
                leaf_info = f"Leaf{leaf['id']}: pos({cx},{cy}) size({w_rect}x{h_rect}) area={int(leaf['area'])}{yellow_info}"
                cv2.putText(result_img, leaf_info, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                y_offset += line_height
        
        # Create debug image (show HSV threshold result)
        debug_img = cv2.cvtColor(thresh_morph, cv2.COLOR_GRAY2BGR)
        
        # Overlay original image on debug image (semi-transparent)
        overlay = crop_img.copy()
        mask_colored = cv2.cvtColor(thresh_morph, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 0] = 0  # Remove blue channel
        mask_colored[:, :, 2] = 0  # Remove red channel
        # Only keep green channel, show detected regions
        
        debug_img = cv2.addWeighted(overlay, 0.7, mask_colored, 0.3, 0)
        
        # Add HSV range info
        green_hsv_text = f"Green H:[{self.green_h_lower}-{self.green_h_upper}] S:[{self.green_s_lower}-{self.green_s_upper}] V:[{self.green_v_lower}-{self.green_v_upper}]"
        cv2.putText(debug_img, green_hsv_text, (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if self.detect_yellow:
            yellow_thresh_text = f"Yellow Ratio Threshold: {yellow_ratio_threshold:.4f}"
            cv2.putText(debug_img, yellow_thresh_text, (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return result_img, debug_img, valid_leaves
    
    def create_trackbars(self):
        """Create trackbars (only keep most important parameters)"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        # Only keep most important green leaf HSV range trackbars
        cv2.createTrackbar('H Lower', self.window_name, self.green_h_lower, 179, self.on_trackbar)
        cv2.createTrackbar('H Upper', self.window_name, self.green_h_upper, 179, self.on_trackbar)
        cv2.createTrackbar('S Lower', self.window_name, self.green_s_lower, 255, self.on_trackbar)
        cv2.createTrackbar('S Upper', self.window_name, self.green_s_upper, 255, self.on_trackbar)
        cv2.createTrackbar('V Lower', self.window_name, self.green_v_lower, 255, self.on_trackbar)
        cv2.createTrackbar('V Upper', self.window_name, self.green_v_upper, 255, self.on_trackbar)
        
        # Yellow ratio threshold (0-1000 represents 0.0-1.0)
        cv2.createTrackbar('Y Ratio*10', self.window_name, self.yellow_ratio_threshold, 1000, self.on_trackbar)
        
        # Area threshold
        cv2.createTrackbar('Min Area', self.window_name, self.min_area, 10000, self.on_trackbar)
        
        # Create debug window
        cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.debug_window, 640, 480)
    
    def on_trackbar(self, val):
        """Trackbar callback function (placeholder, actual update in update_params)"""
        pass
    
    def update_params(self):
        """Update parameters from trackbars"""
        self.green_h_lower = cv2.getTrackbarPos('H Lower', self.window_name)
        self.green_h_upper = cv2.getTrackbarPos('H Upper', self.window_name)
        self.green_s_lower = cv2.getTrackbarPos('S Lower', self.window_name)
        self.green_s_upper = cv2.getTrackbarPos('S Upper', self.window_name)
        self.green_v_lower = cv2.getTrackbarPos('V Lower', self.window_name)
        self.green_v_upper = cv2.getTrackbarPos('V Upper', self.window_name)
        
        self.yellow_ratio_threshold = cv2.getTrackbarPos('Y Ratio*10', self.window_name)
        
        self.min_area = cv2.getTrackbarPos('Min Area', self.window_name)
    
    def print_detected_leaves(self, valid_leaves):
        """Print detected leaf info to console"""
        if len(valid_leaves) == 0:
            return
        
        print(f"\n{'='*80}")
        print(f"Detected {len(valid_leaves)} leaves (frame {self.frame_count}):")
        print(f"{'='*80}")
        
        for leaf in valid_leaves:
            x, y, w, h = leaf['bbox']
            cx, cy = leaf['center']
            yellow_info = f" (Yellow ratio: {leaf['yellow_ratio']:.4f})" if leaf['has_yellow'] else ""
            print(f"  Leaf {leaf['id']}:")
            print(f"    ├─ Position: ({cx}, {cy}) pixels")
            print(f"    ├─ Bounding box: ({x+20}, {y+20}) Size: {w} x {h} pixels")
            print(f"    ├─ Area: {int(leaf['area'])} pixels²")
            if self.detect_yellow:
                print(f"    ├─ Has yellow tape: {leaf['has_yellow']}{yellow_info}")
            print(f"    └─ Status: {'With yellow tape' if leaf['has_yellow'] else 'Normal leaf'}")
        
        print(f"{'='*80}\n")
    
    def print_current_params(self):
        """Print current parameters (for copying to main script)"""
        yellow_ratio_threshold = self.yellow_ratio_threshold / 1000.0
        print("\n" + "="*80)
        print("Current threshold parameters (copy to main script):")
        print("="*80)
        print(f"# Green leaf HSV range")
        print(f"--green-hsv-lower {self.green_h_lower} {self.green_s_lower} {self.green_v_lower}")
        print(f"--green-hsv-upper {self.green_h_upper} {self.green_s_upper} {self.green_v_upper}")
        print(f"")
        if self.detect_yellow:
            print(f"# Yellow tape HSV range")
            print(f"--yellow-hsv-lower {self.yellow_h_lower} {self.yellow_s_lower} {self.yellow_v_lower}")
            print(f"--yellow-hsv-upper {self.yellow_h_upper} {self.yellow_s_upper} {self.yellow_v_upper}")
            print(f"--yellow-threshold {yellow_ratio_threshold:.4f}")
            print(f"")
        print(f"# Area threshold")
        print(f"--min-area {self.min_area}")
        print(f"\nValues for code:")
        print(f"green_hsv_lower = [{self.green_h_lower}, {self.green_s_lower}, {self.green_v_lower}]")
        print(f"green_hsv_upper = [{self.green_h_upper}, {self.green_s_upper}, {self.green_v_upper}]")
        if self.detect_yellow:
            print(f"yellow_ratio_threshold = {yellow_ratio_threshold:.4f}")
        print(f"min_area = {self.min_area}")
        print("="*80 + "\n")
    
    def run(self):
        """Run adjustment tool"""
        print('\nLeaf Detection Threshold Adjustment Tool')
        print('='*80)
        print('Instructions:')
        print('  1. Adjust sliders to change detection parameters (HSV range, yellow ratio, min area)')
        print('  2. View detection results and leaf info in real-time')
        print('  3. Detected leaf info will be printed to console (every 30 frames)')
        print('  4. Press \'p\' to print current parameters')
        print('  5. Press \'s\' to save parameters to file')
        print('  6. Press \'q\' or ESC to exit')
        print('='*80 + '\n')
        
        # Check for other processes using camera
        in_use, pids = self.check_camera_usage()
        if in_use:
            print(f'\n  Warning: Found other processes using camera (PIDs: {", ".join(pids)})')
            print('  Attempting to continue anyway...')
            print('  If camera initialization fails, try: pkill -f realsense2_camera\n')
        
        # Setup camera
        if not self.setup_camera():
            print('\nPlease ensure RealSense camera is connected and available')
            sys.exit(1)
        
        # Create trackbars
        self.create_trackbars()
        
        try:
            while True:
                # Get frames
                color_image, depth_image = self.get_frames()
                
                if color_image is None:
                    continue
                
                self.frame_count += 1
                
                # Update parameters
                self.update_params()
                
                # Detect leaves
                result_img, debug_img, valid_leaves = self.detect_leaves(color_image)
                
                # Print detected leaf info to console every 30 frames
                if len(valid_leaves) > 0 and self.frame_count - self.last_print_frame >= 30:
                    self.print_detected_leaves(valid_leaves)
                    self.last_print_frame = self.frame_count
                
                if result_img is not None:
                    # Display result
                    cv2.imshow(self.window_name, result_img)
                    
                if debug_img is not None:
                    # Display debug image
                    cv2.imshow(self.debug_window, debug_img)
                
                # Keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord('p'):  # Print parameters
                    self.print_current_params()
                    if len(valid_leaves) > 0:
                        self.print_detected_leaves(valid_leaves)
                elif key == ord('s'):  # Save parameters
                    self.save_params()
        
        except KeyboardInterrupt:
            print('\nUser interrupt (Ctrl+C)')
        except Exception as e:
            print(f'\n Runtime error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup
            cv2.destroyAllWindows()
            if self.pipeline:
                self.pipeline.stop()
            print('\n Program exited')
            # Print parameters one last time
            self.print_current_params()
    
    def save_params(self):
        """Save parameters to file"""
        from datetime import datetime
        yellow_ratio_threshold = self.yellow_ratio_threshold / 1000.0
        filename = f"leaf_thresholds_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("# Leaf detection threshold parameters\n")
            f.write(f"# Generated: {datetime.now().isoformat()}\n\n")
            f.write("# Command line arguments:\n")
            f.write(f"--green-hsv-lower {self.green_h_lower} {self.green_s_lower} {self.green_v_lower}\n")
            f.write(f"--green-hsv-upper {self.green_h_upper} {self.green_s_upper} {self.green_v_upper}\n")
            if self.detect_yellow:
                f.write(f"--yellow-threshold {yellow_ratio_threshold:.4f}\n")
            f.write(f"--min-area {self.min_area}\n\n")
            f.write("# Values for Python code:\n")
            f.write(f"green_hsv_lower = [{self.green_h_lower}, {self.green_s_lower}, {self.green_v_lower}]\n")
            f.write(f"green_hsv_upper = [{self.green_h_upper}, {self.green_s_upper}, {self.green_v_upper}]\n")
            if self.detect_yellow:
                f.write(f"yellow_ratio_threshold = {yellow_ratio_threshold:.4f}\n")
            f.write(f"min_area = {self.min_area}\n")
        
        print(f'\n Parameters saved to: {filename}')


def main():
    """Main function"""
    adjuster = LeafThresholdAdjuster()
    adjuster.run()


if __name__ == '__main__':
    main()

