#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Blue Box Detection Threshold Manual Adjustment Tool
Use sliders to adjust HSV color range and other filter parameters in real-time
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import sys


class BlueBoxThresholdAdjuster:
    """Blue box detection threshold adjuster"""
    
    def __init__(self):
        """Initialize parameters"""
        # HSV color range (default values) - only keep these adjustable parameters
        # Using adjusted parameters: S Lower=147 to reduce false positives
        self.blue_h_lower = 100
        self.blue_h_upper = 130
        self.blue_s_lower = 147
        self.blue_s_upper = 255
        self.blue_v_lower = 50
        self.blue_v_upper = 255
        
        # Area threshold
        self.blue_min_area = 3000
        
        # Fixed parameters (no longer adjustable, using reasonable defaults)
        self.min_face_area = 500
        self.rect_ratio_min = 0.4
        self.aspect_ratio_min = 0.2
        self.aspect_ratio_max = 5.0
        self.morph_close_iter = 2
        self.morph_open_iter = 1
        
        # Camera and image
        self.pipeline = None
        self.align = None
        self.color_image = None
        self.depth_image = None
        
        # Window names
        self.window_name = 'Blue Box Threshold Adjustment'
        self.debug_window = 'Debug: HSV Threshold'
        
        # Frame count for console output
        self.frame_count = 0
        self.last_print_frame = 0
        
    def setup_camera(self):
        """Setup RealSense camera"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            self.align = rs.align(rs.stream.color)
            profile = self.pipeline.start(config)
            
            print('✓ Camera initialized')
            return True
        except Exception as e:
            print(f'✗ Camera initialization failed: {e}')
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
            print(f'✗ Failed to get frames: {e}')
            return None, None
    
    def detect_blue_boxes(self, color_image):
        """
        Detect blue boxes using current thresholds
        
        Returns:
            (result_image, debug_image, valid_boxes_list)
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
        
        # Create mask using current thresholds
        lower_blue = np.array([
            self.blue_h_lower,
            self.blue_s_lower,
            self.blue_v_lower
        ], dtype=np.uint8)
        upper_blue = np.array([
            self.blue_h_upper,
            self.blue_s_upper,
            self.blue_v_upper
        ], dtype=np.uint8)
        
        thresh_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Morphological processing
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        thresh_morph = cv2.morphologyEx(
            thresh_blue, 
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
        
        # Detect contours
        contours, _ = cv2.findContours(
            thresh_morph, 
            cv2.RETR_TREE, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filter contours
        valid_boxes = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Area filtering
            min_area_thresh = max(self.min_face_area, self.blue_min_area * 0.3)
            if area < min_area_thresh:
                continue
            
            # Bounding box
            x, y, w_rect, h_rect = cv2.boundingRect(cnt)
            if w_rect == 0 or h_rect == 0:
                continue
            
            # Rectangularity
            bbox_area = w_rect * h_rect
            if bbox_area == 0:
                continue
            rect_ratio = area / bbox_area
            if rect_ratio < self.rect_ratio_min:
                continue
            
            # Aspect ratio
            aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
            if aspect_ratio < self.aspect_ratio_min or aspect_ratio > self.aspect_ratio_max:
                continue
            
            # Calculate center point
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00']) + 20
                cy = int(M['m01'] / M['m00']) + 20
            else:
                cx = x + w_rect // 2 + 20
                cy = y + h_rect // 2 + 20
            
            valid_boxes.append({
                'id': len(valid_boxes) + 1,
                'contour': cnt,
                'area': area,
                'bbox': (x, y, w_rect, h_rect),
                'center': (cx, cy),
                'rect_ratio': rect_ratio,
                'aspect_ratio': aspect_ratio
            })
        
        # Draw results and detailed info
        y_offset = 30
        line_height = 25
        
        # Display detected box count
        info_text = f"Detected: {len(valid_boxes)} blue boxes"
        cv2.putText(result_img, info_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        y_offset += line_height + 5
        
        # Draw each box and display info
        for box in valid_boxes:
            x, y, w_rect, h_rect = box['bbox']
            # Adjust coordinates (add crop offset)
            x_abs = x + 20
            y_abs = y + 20
            cx, cy = box['center']
            
            # Draw bounding box
            color = (255, 0, 0)  # Blue (BGR)
            cv2.rectangle(result_img, (x_abs, y_abs), 
                         (x_abs + w_rect, y_abs + h_rect), color, 2)
            
            # Draw contour
            adjusted_contour = box['contour'] + np.array([20, 20])
            cv2.drawContours(result_img, [adjusted_contour], -1, color, 1)
            
            # Draw center point
            cv2.circle(result_img, (cx, cy), 5, color, -1)
            cv2.circle(result_img, (cx, cy), 15, color, 2)
            
            # Draw label
            label = f'Box {box["id"]}'
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            label_y = max(y_abs - 5, label_size[1] + 5)
            
            cv2.rectangle(result_img, (x_abs, label_y - label_size[1] - 5),
                         (x_abs + label_size[0] + 5, label_y + 5), color, -1)
            cv2.putText(result_img, label, (x_abs + 2, label_y - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display box details on left side of image
            if y_offset < h - 50:
                box_info = f"Box{box['id']}: pos({cx},{cy}) size({w_rect}x{h_rect}) area={int(box['area'])}"
                cv2.putText(result_img, box_info, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                y_offset += line_height
        
        # Create debug image (show HSV threshold result)
        debug_img = cv2.cvtColor(thresh_morph, cv2.COLOR_GRAY2BGR)
        
        # Overlay original image on debug image (semi-transparent)
        overlay = crop_img.copy()
        mask_colored = cv2.cvtColor(thresh_morph, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 0] = 0  # Remove blue channel
        mask_colored[:, :, 1] = 0  # Remove green channel
        # Only keep red channel, show detected regions
        
        debug_img = cv2.addWeighted(overlay, 0.7, mask_colored, 0.3, 0)
        
        # Add HSV range info
        hsv_text = f"H:[{self.blue_h_lower}-{self.blue_h_upper}] S:[{self.blue_s_lower}-{self.blue_s_upper}] V:[{self.blue_v_lower}-{self.blue_v_upper}]"
        cv2.putText(debug_img, hsv_text, (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return result_img, debug_img, valid_boxes
    
    def create_trackbars(self):
        """Create trackbars (only keep most important parameters)"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        # Only keep most important HSV range trackbars
        cv2.createTrackbar('H Lower', self.window_name, self.blue_h_lower, 179, self.on_trackbar)
        cv2.createTrackbar('H Upper', self.window_name, self.blue_h_upper, 179, self.on_trackbar)
        cv2.createTrackbar('S Lower', self.window_name, self.blue_s_lower, 255, self.on_trackbar)
        cv2.createTrackbar('S Upper', self.window_name, self.blue_s_upper, 255, self.on_trackbar)
        cv2.createTrackbar('V Lower', self.window_name, self.blue_v_lower, 255, self.on_trackbar)
        cv2.createTrackbar('V Upper', self.window_name, self.blue_v_upper, 255, self.on_trackbar)
        
        # Area threshold
        cv2.createTrackbar('Min Area', self.window_name, self.blue_min_area, 10000, self.on_trackbar)
        
        # Create debug window
        cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.debug_window, 640, 480)
    
    def on_trackbar(self, val):
        """Trackbar callback function (placeholder, actual update in update_params)"""
        pass
    
    def update_params(self):
        """Update parameters from trackbars"""
        self.blue_h_lower = cv2.getTrackbarPos('H Lower', self.window_name)
        self.blue_h_upper = cv2.getTrackbarPos('H Upper', self.window_name)
        self.blue_s_lower = cv2.getTrackbarPos('S Lower', self.window_name)
        self.blue_s_upper = cv2.getTrackbarPos('S Upper', self.window_name)
        self.blue_v_lower = cv2.getTrackbarPos('V Lower', self.window_name)
        self.blue_v_upper = cv2.getTrackbarPos('V Upper', self.window_name)
        
        self.blue_min_area = cv2.getTrackbarPos('Min Area', self.window_name)
    
    def print_detected_boxes(self, valid_boxes):
        """Print detected box info to console"""
        if len(valid_boxes) == 0:
            return
        
        print(f"\n{'='*80}")
        print(f"Detected {len(valid_boxes)} blue boxes (frame {self.frame_count}):")
        print(f"{'='*80}")
        
        for box in valid_boxes:
            x, y, w, h = box['bbox']
            cx, cy = box['center']
            print(f"  Box {box['id']}:")
            print(f"    ├─ Position: ({cx}, {cy}) pixels")
            print(f"    ├─ Bounding box: ({x+20}, {y+20}) Size: {w} x {h} pixels")
            print(f"    ├─ Area: {int(box['area'])} pixels²")
            print(f"    ├─ Rectangularity: {box['rect_ratio']:.2f}")
            print(f"    └─ Aspect ratio: {box['aspect_ratio']:.2f}")
        
        print(f"{'='*80}\n")
    
    def print_current_params(self):
        """Print current parameters (for copying to main script)"""
        print("\n" + "="*80)
        print("Current threshold parameters (copy to main script):")
        print("="*80)
        print(f"--blue-hsv-lower {self.blue_h_lower} {self.blue_s_lower} {self.blue_v_lower}")
        print(f"--blue-hsv-upper {self.blue_h_upper} {self.blue_s_upper} {self.blue_v_upper}")
        print(f"--blue-min-area {self.blue_min_area}")
        print(f"\nValues for code:")
        print(f"blue_hsv_lower = [{self.blue_h_lower}, {self.blue_s_lower}, {self.blue_v_lower}]")
        print(f"blue_hsv_upper = [{self.blue_h_upper}, {self.blue_s_upper}, {self.blue_v_upper}]")
        print(f"blue_min_area = {self.blue_min_area}")
        print("="*80 + "\n")
    
    def run(self):
        """Run adjustment tool"""
        print('\nBlue Box Detection Threshold Adjustment Tool')
        print('='*80)
        print('Instructions:')
        print('  1. Adjust sliders to change detection parameters (HSV range and min area)')
        print('  2. View detection results and box info in real-time')
        print('  3. Detected box info will be printed to console (every 30 frames)')
        print('  4. Press \'p\' to print current parameters')
        print('  5. Press \'s\' to save parameters to file')
        print('  6. Press \'q\' or ESC to exit')
        print('='*80 + '\n')
        
        # Setup camera
        if not self.setup_camera():
            print('Please ensure RealSense camera is connected')
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
                
                # Detect blue boxes
                result_img, debug_img, valid_boxes = self.detect_blue_boxes(color_image)
                
                # Print detected box info to console every 30 frames
                if len(valid_boxes) > 0 and self.frame_count - self.last_print_frame >= 30:
                    self.print_detected_boxes(valid_boxes)
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
                    if len(valid_boxes) > 0:
                        self.print_detected_boxes(valid_boxes)
                elif key == ord('s'):  # Save parameters
                    self.save_params()
        
        except KeyboardInterrupt:
            print('\nUser interrupt (Ctrl+C)')
        except Exception as e:
            print(f'\n✗ Runtime error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup
            cv2.destroyAllWindows()
            if self.pipeline:
                self.pipeline.stop()
            print('\n✓ Program exited')
            # Print parameters one last time
            self.print_current_params()
    
    def save_params(self):
        """Save parameters to file"""
        from datetime import datetime
        filename = f"blue_box_thresholds_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("# Blue box detection threshold parameters\n")
            f.write(f"# Generated: {datetime.now().isoformat()}\n\n")
            f.write("# Command line arguments:\n")
            f.write(f"--blue-hsv-lower {self.blue_h_lower} {self.blue_s_lower} {self.blue_v_lower}\n")
            f.write(f"--blue-hsv-upper {self.blue_h_upper} {self.blue_s_upper} {self.blue_v_upper}\n")
            f.write(f"--blue-min-area {self.blue_min_area}\n\n")
            f.write("# Values for Python code:\n")
            f.write(f"blue_hsv_lower = [{self.blue_h_lower}, {self.blue_s_lower}, {self.blue_v_lower}]\n")
            f.write(f"blue_hsv_upper = [{self.blue_h_upper}, {self.blue_s_upper}, {self.blue_v_upper}]\n")
            f.write(f"blue_min_area = {self.blue_min_area}\n")
        
        print(f'\n✓ Parameters saved to: {filename}')


def main():
    """Main function"""
    adjuster = BlueBoxThresholdAdjuster()
    adjuster.run()


if __name__ == '__main__':
    main()

