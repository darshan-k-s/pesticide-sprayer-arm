#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝色盒子检测阈值手动调整工具
使用滑动条实时调整HSV颜色范围和其他过滤参数
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import sys


class BlueBoxThresholdAdjuster:
    """蓝色盒子检测阈值调整器"""
    
    def __init__(self):
        """初始化参数"""
        # HSV颜色范围（默认值）- 只保留这些可调参数
        # 使用调整后的参数：S Lower=147 以减少误检
        self.blue_h_lower = 100
        self.blue_h_upper = 130
        self.blue_s_lower = 147
        self.blue_s_upper = 255
        self.blue_v_lower = 50
        self.blue_v_upper = 255
        
        # 面积阈值
        self.blue_min_area = 3000
        
        # 固定参数（不再可调，使用合理默认值）
        self.min_face_area = 500
        self.rect_ratio_min = 0.4
        self.aspect_ratio_min = 0.2
        self.aspect_ratio_max = 5.0
        self.morph_close_iter = 2
        self.morph_open_iter = 1
        
        # 相机和图像
        self.pipeline = None
        self.align = None
        self.color_image = None
        self.depth_image = None
        
        # 窗口名称
        self.window_name = 'Blue Box Threshold Adjustment'
        self.debug_window = 'Debug: HSV Threshold'
        
        # 用于控制台输出的帧计数
        self.frame_count = 0
        self.last_print_frame = 0
        
    def setup_camera(self):
        """设置RealSense相机"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            self.align = rs.align(rs.stream.color)
            profile = self.pipeline.start(config)
            
            print('✓ 相机初始化完成')
            return True
        except Exception as e:
            print(f'✗ 相机初始化失败: {e}')
            return False
    
    def get_frames(self):
        """获取相机帧"""
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
            print(f'✗ 获取帧失败: {e}')
            return None, None
    
    def detect_blue_boxes(self, color_image):
        """
        使用当前阈值检测蓝色盒子
        
        Returns:
            (result_image, debug_image, valid_boxes_list)
        """
        if color_image is None:
            return None, None, []
        
        # 复制图像用于绘制
        result_img = color_image.copy()
        
        # 轻微裁剪（与主脚本一致）
        h, w = color_image.shape[:2]
        crop_img = color_image[20:h-20, 20:w-20]
        
        # 转换为HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # 使用当前阈值创建掩码
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
        
        # 形态学处理
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
        
        # 检测轮廓
        contours, _ = cv2.findContours(
            thresh_morph, 
            cv2.RETR_TREE, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # 过滤轮廓
        valid_boxes = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # 面积过滤
            min_area_thresh = max(self.min_face_area, self.blue_min_area * 0.3)
            if area < min_area_thresh:
                continue
            
            # 边界框
            x, y, w_rect, h_rect = cv2.boundingRect(cnt)
            if w_rect == 0 or h_rect == 0:
                continue
            
            # 矩形度
            bbox_area = w_rect * h_rect
            if bbox_area == 0:
                continue
            rect_ratio = area / bbox_area
            if rect_ratio < self.rect_ratio_min:
                continue
            
            # 宽高比
            aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
            if aspect_ratio < self.aspect_ratio_min or aspect_ratio > self.aspect_ratio_max:
                continue
            
            # 计算中心点
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
        
        # 绘制结果和详细信息
        y_offset = 30
        line_height = 25
        
        # 显示检测到的盒子数量
        info_text = f"检测到: {len(valid_boxes)} 个蓝色盒子"
        cv2.putText(result_img, info_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        y_offset += line_height + 5
        
        # 绘制每个盒子并显示信息
        for box in valid_boxes:
            x, y, w_rect, h_rect = box['bbox']
            # 调整坐标（加上裁剪偏移）
            x_abs = x + 20
            y_abs = y + 20
            cx, cy = box['center']
            
            # 绘制边界框
            color = (255, 0, 0)  # 蓝色 (BGR)
            cv2.rectangle(result_img, (x_abs, y_abs), 
                         (x_abs + w_rect, y_abs + h_rect), color, 2)
            
            # 绘制轮廓
            adjusted_contour = box['contour'] + np.array([20, 20])
            cv2.drawContours(result_img, [adjusted_contour], -1, color, 1)
            
            # 绘制中心点
            cv2.circle(result_img, (cx, cy), 5, color, -1)
            cv2.circle(result_img, (cx, cy), 15, color, 2)
            
            # 绘制标签
            label = f'Box {box["id"]}'
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            label_y = max(y_abs - 5, label_size[1] + 5)
            
            cv2.rectangle(result_img, (x_abs, label_y - label_size[1] - 5),
                         (x_abs + label_size[0] + 5, label_y + 5), color, -1)
            cv2.putText(result_img, label, (x_abs + 2, label_y - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 在图像左侧显示盒子详细信息
            if y_offset < h - 50:
                box_info = f"Box{box['id']}: 位置({cx},{cy}) 大小({w_rect}x{h_rect}) 面积={int(box['area'])}"
                cv2.putText(result_img, box_info, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                y_offset += line_height
        
        # 创建调试图像（显示HSV阈值结果）
        debug_img = cv2.cvtColor(thresh_morph, cv2.COLOR_GRAY2BGR)
        
        # 在调试图像上叠加原始图像（半透明）
        overlay = crop_img.copy()
        mask_colored = cv2.cvtColor(thresh_morph, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 0] = 0  # 移除蓝色通道
        mask_colored[:, :, 1] = 0  # 移除绿色通道
        # 只保留红色通道，显示检测到的区域
        
        debug_img = cv2.addWeighted(overlay, 0.7, mask_colored, 0.3, 0)
        
        # 添加HSV范围信息
        hsv_text = f"H:[{self.blue_h_lower}-{self.blue_h_upper}] S:[{self.blue_s_lower}-{self.blue_s_upper}] V:[{self.blue_v_lower}-{self.blue_v_upper}]"
        cv2.putText(debug_img, hsv_text, (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return result_img, debug_img, valid_boxes
    
    def create_trackbars(self):
        """创建滑动条（只保留最重要的参数）"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        # 只保留最重要的HSV范围滑动条
        cv2.createTrackbar('H Lower', self.window_name, self.blue_h_lower, 179, self.on_trackbar)
        cv2.createTrackbar('H Upper', self.window_name, self.blue_h_upper, 179, self.on_trackbar)
        cv2.createTrackbar('S Lower', self.window_name, self.blue_s_lower, 255, self.on_trackbar)
        cv2.createTrackbar('S Upper', self.window_name, self.blue_s_upper, 255, self.on_trackbar)
        cv2.createTrackbar('V Lower', self.window_name, self.blue_v_lower, 255, self.on_trackbar)
        cv2.createTrackbar('V Upper', self.window_name, self.blue_v_upper, 255, self.on_trackbar)
        
        # 面积阈值
        cv2.createTrackbar('Min Area', self.window_name, self.blue_min_area, 10000, self.on_trackbar)
        
        # 创建调试窗口
        cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.debug_window, 640, 480)
    
    def on_trackbar(self, val):
        """滑动条回调函数（占位符，实际更新在update_params中）"""
        pass
    
    def update_params(self):
        """从滑动条更新参数"""
        self.blue_h_lower = cv2.getTrackbarPos('H Lower', self.window_name)
        self.blue_h_upper = cv2.getTrackbarPos('H Upper', self.window_name)
        self.blue_s_lower = cv2.getTrackbarPos('S Lower', self.window_name)
        self.blue_s_upper = cv2.getTrackbarPos('S Upper', self.window_name)
        self.blue_v_lower = cv2.getTrackbarPos('V Lower', self.window_name)
        self.blue_v_upper = cv2.getTrackbarPos('V Upper', self.window_name)
        
        self.blue_min_area = cv2.getTrackbarPos('Min Area', self.window_name)
    
    def print_detected_boxes(self, valid_boxes):
        """在控制台打印检测到的盒子信息"""
        if len(valid_boxes) == 0:
            return
        
        print(f"\n{'='*80}")
        print(f"检测到 {len(valid_boxes)} 个蓝色盒子 (帧 {self.frame_count}):")
        print(f"{'='*80}")
        
        for box in valid_boxes:
            x, y, w, h = box['bbox']
            cx, cy = box['center']
            print(f"  Box {box['id']}:")
            print(f"    ├─ 位置: ({cx}, {cy}) 像素")
            print(f"    ├─ 边界框: ({x+20}, {y+20}) 尺寸: {w} x {h} 像素")
            print(f"    ├─ 面积: {int(box['area'])} 像素²")
            print(f"    ├─ 矩形度: {box['rect_ratio']:.2f}")
            print(f"    └─ 宽高比: {box['aspect_ratio']:.2f}")
        
        print(f"{'='*80}\n")
    
    def print_current_params(self):
        """打印当前参数（用于复制到主脚本）"""
        print("\n" + "="*80)
        print("当前阈值参数（复制到主脚本使用）:")
        print("="*80)
        print(f"--blue-hsv-lower {self.blue_h_lower} {self.blue_s_lower} {self.blue_v_lower}")
        print(f"--blue-hsv-upper {self.blue_h_upper} {self.blue_s_upper} {self.blue_v_upper}")
        print(f"--blue-min-area {self.blue_min_area}")
        print(f"\n代码中使用的值:")
        print(f"blue_hsv_lower = [{self.blue_h_lower}, {self.blue_s_lower}, {self.blue_v_lower}]")
        print(f"blue_hsv_upper = [{self.blue_h_upper}, {self.blue_s_upper}, {self.blue_v_upper}]")
        print(f"blue_min_area = {self.blue_min_area}")
        print("="*80 + "\n")
    
    def run(self):
        """运行调整工具"""
        print('\n蓝色盒子检测阈值调整工具')
        print('='*80)
        print('使用说明:')
        print('  1. 调整滑动条来改变检测参数（HSV范围和最小面积）')
        print('  2. 实时查看检测结果和盒子信息')
        print('  3. 检测到的盒子信息会自动在控制台输出（每30帧）')
        print('  4. 按 \'p\' 键打印当前参数')
        print('  5. 按 \'s\' 键保存参数到文件')
        print('  6. 按 \'q\' 或 ESC 键退出')
        print('='*80 + '\n')
        
        # 设置相机
        if not self.setup_camera():
            print('请确保RealSense相机已连接')
            sys.exit(1)
        
        # 创建滑动条
        self.create_trackbars()
        
        try:
            while True:
                # 获取帧
                color_image, depth_image = self.get_frames()
                
                if color_image is None:
                    continue
                
                self.frame_count += 1
                
                # 更新参数
                self.update_params()
                
                # 检测蓝色盒子
                result_img, debug_img, valid_boxes = self.detect_blue_boxes(color_image)
                
                # 每30帧在控制台输出检测到的盒子信息
                if len(valid_boxes) > 0 and self.frame_count - self.last_print_frame >= 30:
                    self.print_detected_boxes(valid_boxes)
                    self.last_print_frame = self.frame_count
                
                if result_img is not None:
                    # 显示结果
                    cv2.imshow(self.window_name, result_img)
                    
                if debug_img is not None:
                    # 显示调试图像
                    cv2.imshow(self.debug_window, debug_img)
                
                # 键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' 或 ESC
                    break
                elif key == ord('p'):  # 打印参数
                    self.print_current_params()
                    if len(valid_boxes) > 0:
                        self.print_detected_boxes(valid_boxes)
                elif key == ord('s'):  # 保存参数
                    self.save_params()
        
        except KeyboardInterrupt:
            print('\n用户中断 (Ctrl+C)')
        except Exception as e:
            print(f'\n✗ 运行错误: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # 清理
            cv2.destroyAllWindows()
            if self.pipeline:
                self.pipeline.stop()
            print('\n✓ 程序退出')
            # 最后打印一次参数
            self.print_current_params()
    
    def save_params(self):
        """保存参数到文件"""
        from datetime import datetime
        filename = f"blue_box_thresholds_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("# 蓝色盒子检测阈值参数\n")
            f.write(f"# 生成时间: {datetime.now().isoformat()}\n\n")
            f.write("# 命令行参数:\n")
            f.write(f"--blue-hsv-lower {self.blue_h_lower} {self.blue_s_lower} {self.blue_v_lower}\n")
            f.write(f"--blue-hsv-upper {self.blue_h_upper} {self.blue_s_upper} {self.blue_v_upper}\n")
            f.write(f"--blue-min-area {self.blue_min_area}\n\n")
            f.write("# Python代码中的值:\n")
            f.write(f"blue_hsv_lower = [{self.blue_h_lower}, {self.blue_s_lower}, {self.blue_v_lower}]\n")
            f.write(f"blue_hsv_upper = [{self.blue_h_upper}, {self.blue_s_upper}, {self.blue_v_upper}]\n")
            f.write(f"blue_min_area = {self.blue_min_area}\n")
        
        print(f'\n✓ 参数已保存到: {filename}')


def main():
    """主函数"""
    adjuster = BlueBoxThresholdAdjuster()
    adjuster.run()


if __name__ == '__main__':
    main()

