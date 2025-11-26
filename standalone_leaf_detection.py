#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立叶子检测和显示脚本
不依赖ROS2，直接从RealSense相机读取图像并检测叶子
"""

import cv2
import numpy as np
from plantcv import plantcv as pcv
import pyrealsense2 as rs
from datetime import datetime
import argparse
import sys


class StandaloneLeafDetector:
    """独立的叶子检测器，不依赖ROS2"""
    
    def __init__(self, min_area=2000, detect_yellow=True, yellow_ratio_threshold=0.05, 
                 yellow_hsv_lower=None, yellow_hsv_upper=None, detect_blue_box=True,
                 blue_min_area=3000, blue_hsv_lower=None, blue_hsv_upper=None):
        """
        初始化检测器
        
        Args:
            min_area: 最小叶子面积阈值
            detect_yellow: 是否检测黄色区域（用于检测贴有黄色胶布的叶子）
            yellow_ratio_threshold: 黄色比例阈值（默认0.05，即5%，降低以更容易检测黄色胶布）
            yellow_hsv_lower: HSV颜色范围下限（默认[20, 100, 100]，更严格以检测亮黄色胶布）
            yellow_hsv_upper: HSV颜色范围上限（默认[30, 255, 255]）
            detect_blue_box: 是否检测蓝色盒子（默认True）
            blue_min_area: 蓝色盒子最小面积阈值（默认3000）
            blue_hsv_lower: 蓝色HSV颜色范围下限（默认[100, 50, 50]）
            blue_hsv_upper: 蓝色HSV颜色范围上限（默认[130, 255, 255]）
        """
        self.min_area = min_area
        self.detect_yellow = detect_yellow
        self.yellow_ratio_threshold = yellow_ratio_threshold
        
        # HSV颜色范围参数（可配置，默认更严格以检测真正的黄色胶布）
        # 黄色胶布通常是亮黄色，饱和度和亮度都较高
        self.yellow_hsv_lower = yellow_hsv_lower if yellow_hsv_lower is not None else [20, 100, 100]
        self.yellow_hsv_upper = yellow_hsv_upper if yellow_hsv_upper is not None else [30, 255, 255]
        
        # 蓝色盒子检测参数
        self.detect_blue_box = detect_blue_box
        self.blue_min_area = blue_min_area
        # 蓝色在HSV中：H值在100-130之间（蓝色范围），S和V值较高
        # 使用调整后的参数：S Lower=147 以减少误检
        self.blue_hsv_lower = blue_hsv_lower if blue_hsv_lower is not None else [100, 147, 50]
        self.blue_hsv_upper = blue_hsv_upper if blue_hsv_upper is not None else [130, 255, 255]
        
        # PlantCV设置
        pcv.params.debug = None  # 禁用调试输出
        
        # 相机内参（将从RealSense获取）
        self.intrinsics = None
        
        print('✓ 独立叶子检测器初始化完成')
        if self.detect_yellow:
            print(f'  ✓ 黄色胶布检测已启用 (阈值={self.yellow_ratio_threshold:.3f}, '
                  f'HSV范围: {self.yellow_hsv_lower} - {self.yellow_hsv_upper})')
        if self.detect_blue_box:
            print(f'  ✓ 蓝色盒子检测已启用 (最小面积={self.blue_min_area}, '
                  f'HSV范围: {self.blue_hsv_lower} - {self.blue_hsv_upper})')
    
    def setup_camera(self):
        """设置RealSense相机"""
        # 配置深度和颜色流
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 启用颜色和深度流
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 创建对齐器（将深度对齐到颜色）
        align_to = rs.align(rs.stream.color)
        self.align = align_to
        
        # 启动流
        profile = self.pipeline.start(config)
        
        # 获取颜色流的内参
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.intrinsics = color_profile.get_intrinsics()
        
        print(f'✓ RealSense相机初始化完成')
        print(f'  分辨率: {self.intrinsics.width}x{self.intrinsics.height}')
        print(f'  内参: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}')
        
        return True
    
    def get_frames(self):
        """获取同步的颜色和深度帧"""
        try:
            frames = self.pipeline.wait_for_frames()
            
            # 对齐深度帧到颜色帧
            aligned_frames = self.align.process(frames)
            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                return None, None
            
            # 转换为numpy数组
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
        except Exception as e:
            print(f'✗ 获取帧失败: {e}')
            return None, None
    
    def pixel_to_3d(self, pixel_u, pixel_v, depth_value_mm):
        """
        将像素坐标和深度值转换为3D坐标（相机坐标系）
        
        Args:
            pixel_u: 像素u坐标
            pixel_v: 像素v坐标
            depth_value_mm: 深度值（毫米）
        
        Returns:
            3D坐标元组 (x, y, z) 单位：米，如果失败返回None
        """
        if self.intrinsics is None or depth_value_mm == 0:
            return None
        
        try:
            depth_m = depth_value_mm * 0.001  # 转换为米
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                [pixel_u, pixel_v],
                depth_m
            )
            return tuple(point_3d)
        except Exception as e:
            print(f'✗ 3D反投影错误: {e}')
            return None
    
    def measure_box_dimensions_3d(self, bbox_2d, depth_image, mask=None, sample_step=2):
        """
        使用密集深度采样测量盒子的实际3D尺寸（高精度方法）
        
        该方法通过密集采样3D点并计算3D边界框来获得准确的尺寸：
        1. 在检测区域内密集采样3D点
        2. 使用统计方法去除异常值（离群点）
        3. 计算3D边界框得到长宽高
        
        Args:
            bbox_2d: 2D边界框 (x_min, y_min, x_max, y_max) 在原始图像坐标系
            depth_image: 深度图像（numpy数组，单位：毫米）
            mask: 可选的掩码（如果提供，只采样掩码内的点）
            sample_step: 采样步长（像素），默认2（更密集采样以提高精度）
        
        Returns:
            dict: {
                'width': 宽度（米，X方向）,
                'height': 高度（米，Y方向）,
                'depth': 深度（米，Z方向，距离相机）,
                'min_point': 最小边界点 (x, y, z),
                'max_point': 最大边界点 (x, y, z),
                'num_samples': 采样点数,
                'center_3d': 3D中心点 (x, y, z)
            } 或 None（如果失败）
        """
        if depth_image is None or self.intrinsics is None:
            return None
        
        x_min, y_min, x_max, y_max = bbox_2d
        
        # 确保坐标在图像范围内
        x_min = max(0, int(x_min))
        y_min = max(0, int(y_min))
        x_max = min(depth_image.shape[1], int(x_max))
        y_max = min(depth_image.shape[0], int(y_max))
        
        if x_max <= x_min or y_max <= y_min:
            return None
        
        # 方法1：密集采样3D点
        points_3d = []
        valid_depths = []
        
        # 在边界框内密集采样（sample_step=2表示每2个像素采样一次）
        for y in range(y_min, y_max, sample_step):
            for x in range(x_min, x_max, sample_step):
                # 如果提供了掩码，检查该点是否在掩码内
                if mask is not None:
                    if (y < 0 or y >= mask.shape[0] or 
                        x < 0 or x >= mask.shape[1] or
                        mask[y, x] == 0):
                        continue
                
                # 获取深度值
                depth_mm = depth_image[y, x]
                
                # 过滤无效深度
                if depth_mm <= 0 or depth_mm > 5000:
                    continue
                
                # 转换为3D坐标
                point_3d = self.pixel_to_3d(x, y, depth_mm)
                if point_3d is not None:
                    points_3d.append(point_3d)
                    valid_depths.append(depth_mm)
        
        if len(points_3d) < 8:  # 至少需要8个点才能准确估算尺寸
            return None
        
        # 方法2：使用统计方法去除异常值（提高鲁棒性）
        points_array = np.array(points_3d)
        depths_array = np.array(valid_depths)
        
        # 计算深度中位数，去除离群点（深度差异过大可能是背景或噪声）
        median_depth = np.median(depths_array)
        depth_std = np.std(depths_array)
        depth_threshold = median_depth + 3 * depth_std  # 3倍标准差
        
        # 过滤异常深度点
        valid_mask = depths_array <= depth_threshold
        filtered_points = points_array[valid_mask]
        
        if len(filtered_points) < 8:
            filtered_points = points_array  # 如果过滤后点数太少，使用原始点
        
        # 方法3：计算3D边界框
        # 找到所有点的最小/最大X, Y, Z值
        min_point = filtered_points.min(axis=0)  # [min_x, min_y, min_z]
        max_point = filtered_points.max(axis=0)   # [max_x, max_y, max_z]
        center_3d = (min_point + max_point) / 2
        
        # 计算尺寸（在相机坐标系中）
        # X方向 = 左右（宽度）
        # Y方向 = 上下（高度）
        # Z方向 = 前后（深度，距离相机）
        width = float(max_point[0] - min_point[0])   # X方向
        height = float(max_point[1] - min_point[1])   # Y方向
        depth = float(max_point[2] - min_point[2])    # Z方向
        
        # 如果深度（Z方向）太小，可能是只看到了一个面，使用统计方法估算
        if depth < 0.01:  # 小于1cm，可能是平面视图
            # 使用深度标准差作为深度的估算
            z_values = filtered_points[:, 2]
            depth = float(np.std(z_values)) * 2  # 使用2倍标准差作为深度估算
            if depth < 0.01:
                depth = 0.1  # 如果还是太小，使用默认值
        
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
        使用PlantCV检测叶子
        
        Args:
            cv_image: BGR图像
            depth_image: 深度图像（16位，单位：毫米）
            frame_count: 帧计数（用于调试）
        
        Returns:
            (detection_result_string, leaf_data_dict, bounding_boxes_list, debug_images_dict)
        """
        debug_images = {}  # 存储中间处理步骤的图像
        blue_box_coordinates = []  # 初始化蓝色盒子坐标列表
        
        try:
            h, w = cv_image.shape[:2]
            
            # 保存原始图像
            debug_images['original'] = cv_image.copy()
            
            # 轻微裁剪
            crop_img = pcv.crop(img=cv_image, x=20, y=20, h=h-40, w=w-40)
            debug_images['cropped'] = crop_img.copy()
            
            # HSV颜色空间 - 先检测绿色叶子
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            debug_images['hsv'] = hsv.copy()
            
            # 步骤1: 检测绿色叶子
            lower_green = np.array([40, 60, 40])
            upper_green = np.array([80, 255, 255])
            thresh_green = cv2.inRange(hsv, lower_green, upper_green)
            
            # 步骤2: 检测黄色胶布区域（全图检测，用于后续在叶子区域内分析）
            thresh_yellow_full = np.zeros_like(thresh_green)
            if self.detect_yellow:
                # 使用可配置的HSV范围，默认范围更宽以应对不同光照条件
                lower_yellow = np.array(self.yellow_hsv_lower, dtype=np.uint8)
                upper_yellow = np.array(self.yellow_hsv_upper, dtype=np.uint8)
                thresh_yellow_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # 使用更严格的形态学操作，去除小的噪声点
                # 先进行开运算去除小噪声，然后进行闭运算连接相近区域
                kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_OPEN, kernel_small, iterations=1)
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_CLOSE, kernel_small, iterations=1)
                
                # 使用LAB颜色空间作为补充，但使用更严格的范围
                # 只检测真正的亮黄色胶布，避免检测到叶子本身的黄色部分
                lab = cv2.cvtColor(crop_img, cv2.COLOR_BGR2LAB)
                # 黄色胶布通常是亮黄色：L较高（亮度高），a较低（偏绿），b很高（偏黄）
                # 使用更严格的范围，避免误检叶子边缘的黄色
                lab_yellow_mask = np.zeros_like(thresh_yellow_hsv, dtype=np.uint8)
                # 更严格的条件：L>110（较亮），a<140（偏绿），b>150（很黄）
                lab_yellow_mask[(lab[:,:,0] > 110) & (lab[:,:,1] < 140) & (lab[:,:,2] > 150)] = 255
                
                # 合并HSV和LAB的检测结果（优先使用OR操作，使检测更敏感）
                # 优先使用OR操作，使检测更敏感（更容易检测到黄色胶布）
                thresh_yellow_full = cv2.bitwise_or(thresh_yellow_hsv, lab_yellow_mask)
                
                # 形态学处理去除小噪声
                kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_OPEN, kernel_medium, iterations=1)
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
            
            # 先使用绿色检测进行初步处理
            thresh = thresh_green.copy()
            thresh_binary = (thresh / 255).astype(np.uint8)
            
            # 步骤2.5: 检测蓝色盒子（独立于叶子检测）
            thresh_blue_full = np.zeros_like(thresh_green)
            blue_box_coordinates = []
            if self.detect_blue_box:
                # 使用HSV颜色空间检测蓝色
                # 蓝色在HSV中：H值在100-130之间（蓝色范围），S和V值较高
                lower_blue_hsv = np.array(self.blue_hsv_lower, dtype=np.uint8)
                upper_blue_hsv = np.array(self.blue_hsv_upper, dtype=np.uint8)
                thresh_blue_hsv = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
                
                # 形态学处理：去除小噪声，连接相近区域
                # 使用较小的kernel，避免破坏蓝色盒子的形状
                kernel_blue_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                kernel_blue_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
                
                # 先闭运算连接相近区域，再开运算去除小噪声
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_hsv, cv2.MORPH_CLOSE, kernel_blue_medium, iterations=2)
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_morph, cv2.MORPH_OPEN, kernel_blue_small, iterations=1)
                
                thresh_blue_full = thresh_blue_morph
                
                # 检测蓝色盒子的轮廓（包括所有层级，以检测多个面）
                # 使用RETR_TREE来获取所有轮廓，包括内部轮廓（可能是盒子的不同面）
                blue_contours, blue_hierarchy = cv2.findContours(thresh_blue_full, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # 调试信息：显示检测到的轮廓数量
                if frame_count <= 5:
                    print(f'  🔍 蓝色检测: 找到 {len(blue_contours)} 个轮廓 (HSV范围: {self.blue_hsv_lower}-{self.blue_hsv_upper}, 最小面积>{self.blue_min_area})')
                
                # 收集所有有效的蓝色区域（可能是盒子的不同面）
                valid_blue_regions = []
                
                # 过滤蓝色盒子轮廓（检测所有可见面）
                for idx, blue_cnt in enumerate(blue_contours):
                    area = cv2.contourArea(blue_cnt)
                    
                    # 降低单个面的最小面积要求，因为我们要检测多个面
                    min_face_area = max(500, self.blue_min_area * 0.3)  # 单个面至少是总面积的30%或500像素
                    if area < min_face_area:
                        continue
                    
                    # 计算边界框
                    x, y, w_rect, h_rect = cv2.boundingRect(blue_cnt)
                    if w_rect == 0 or h_rect == 0:
                        continue
                    
                    # 计算矩形度（轮廓面积与边界框面积的比值）
                    bbox_area = w_rect * h_rect
                    if bbox_area == 0:
                        continue
                    rect_ratio = area / bbox_area
                    
                    # 计算宽高比
                    aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                    
                    # 更宽松的过滤条件（因为可能是盒子的侧面，形状可能不完全规则）
                    if rect_ratio < 0.4:  # 降低到0.4
                        continue
                    
                    if aspect_ratio < 0.2 or aspect_ratio > 5.0:  # 放宽范围
                        continue
                    
                    # 计算中心点
                    M = cv2.moments(blue_cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00']) + 20
                        cy = int(M['m01'] / M['m00']) + 20
                    else:
                        cx = x + w_rect // 2 + 20
                        cy = y + h_rect // 2 + 20
                    
                    # 获取深度值和3D坐标（采样多个点以获得更准确的深度）
                    depth_value_mm = 0
                    point_3d = None
                    depth_samples = []
                    
                    if depth_image is not None:
                        try:
                            # 在轮廓区域内采样多个点
                            # 注意：轮廓坐标是相对于裁剪图像的，需要转换到原始图像坐标
                            mask = np.zeros((crop_img.shape[0], crop_img.shape[1]), dtype=np.uint8)
                            cv2.drawContours(mask, [blue_cnt], -1, 255, -1)
                            
                            # 转换到原始图像坐标（深度图像是原始尺寸）
                            depth_x = x + 20
                            depth_y = y + 20
                            depth_x_end = min(depth_image.shape[1], depth_x + w_rect)
                            depth_y_end = min(depth_image.shape[0], depth_y + h_rect)
                            depth_x_start = max(0, depth_x)
                            depth_y_start = max(0, depth_y)
                            
                            # 在掩码区域内采样深度值
                            mask_roi = mask[y:y+h_rect, x:x+w_rect]
                            depth_roi = depth_image[depth_y_start:depth_y_end, depth_x_start:depth_x_end]
                            
                            # 调整mask ROI的大小以匹配depth ROI
                            if mask_roi.shape != depth_roi.shape:
                                mask_roi_resized = cv2.resize(mask_roi, (depth_roi.shape[1], depth_roi.shape[0]))
                            else:
                                mask_roi_resized = mask_roi
                            
                            valid_depths = depth_roi[(mask_roi_resized > 0) & (depth_roi > 0)]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.median(valid_depths))  # 使用中位数更稳定
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                                    depth_samples.append(depth_value_mm)
                        except Exception as e:
                            if frame_count <= 3:
                                print(f'  蓝色区域深度获取错误: {e}')
                            pass
                    
                    # 保存蓝色区域信息（可能是盒子的一个面）
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
                
                # 根据深度和位置将区域分组为同一个盒子
                # 如果多个区域深度相近且位置相近，它们可能属于同一个盒子
                blue_box_groups = []
                depth_tolerance = 50  # 深度容差（毫米）
                position_tolerance = 100  # 位置容差（像素）
                
                for region in valid_blue_regions:
                    if region['point_3d'] is None:
                        # 如果没有深度信息，每个区域单独成组
                        blue_box_groups.append([region])
                        continue
                    
                    # 尝试找到匹配的组
                    matched = False
                    for group in blue_box_groups:
                        # 检查是否有深度信息
                        group_depths = [r['depth_mm'] for r in group if r['point_3d'] is not None]
                        if len(group_depths) == 0:
                            continue
                        
                        avg_group_depth = np.mean(group_depths)
                        depth_diff = abs(region['depth_mm'] - avg_group_depth)
                        
                        # 检查位置是否相近
                        group_centers = [r['center_2d'] for r in group]
                        avg_center = (np.mean([c[0] for c in group_centers]), 
                                     np.mean([c[1] for c in group_centers]))
                        pos_diff = np.sqrt((region['center_2d'][0] - avg_center[0])**2 + 
                                          (region['center_2d'][1] - avg_center[1])**2)
                        
                        # 如果深度和位置都相近，加入该组
                        if depth_diff < depth_tolerance and pos_diff < position_tolerance:
                            group.append(region)
                            matched = True
                            break
                    
                    if not matched:
                        # 创建新组
                        blue_box_groups.append([region])
                
                # 为每个组创建一个完整的蓝色盒子
                blue_box_idx = 1
                for group in blue_box_groups:
                    if len(group) == 0:
                        continue
                    
                    # 计算合并后的边界框（包含所有面）
                    all_x_min = min([r['bbox'][0] for r in group])
                    all_y_min = min([r['bbox'][1] for r in group])
                    all_x_max = max([r['bbox'][0] + r['bbox'][2] for r in group])
                    all_y_max = max([r['bbox'][1] + r['bbox'][3] for r in group])
                    
                    merged_w = all_x_max - all_x_min
                    merged_h = all_y_max - all_y_min
                    merged_area = sum([r['area'] for r in group])
                    
                    # 计算合并后的中心点
                    merged_cx = (all_x_min + all_x_max) // 2 + 20
                    merged_cy = (all_y_min + all_y_max) // 2 + 20
                    
                    # 计算平均深度和3D坐标
                    valid_depths = [r['depth_mm'] for r in group if r['depth_mm'] > 0]
                    avg_depth_mm = int(np.mean(valid_depths)) if len(valid_depths) > 0 else 0
                    
                    # 获取3D坐标
                    merged_point_3d = None
                    if avg_depth_mm > 0 and depth_image is not None:
                        try:
                            merged_point_3d = self.pixel_to_3d(merged_cx, merged_cy, avg_depth_mm)
                        except:
                            pass
                    
                    # 计算盒子的3D尺寸（使用高精度密集采样方法）
                    box_3d_size = None
                    measured_dimensions = None
                    if depth_image is not None and merged_point_3d:
                        # 使用密集深度采样方法计算精确的3D尺寸
                        bbox_for_measurement = (
                            all_x_min,  # x_min
                            all_y_min,  # y_min
                            all_x_max,  # x_max
                            all_y_max   # y_max
                        )
                        
                        # 创建蓝色区域的掩码（可选，用于更精确的采样）
                        blue_mask = None
                        try:
                            # 创建掩码：在合并后的边界框内，只采样蓝色区域
                            blue_mask = np.zeros_like(depth_image, dtype=np.uint8)
                            for region in group:
                                x_reg, y_reg, w_reg, h_reg = region['bbox']
                                # 绘制每个面的区域（简单矩形掩码）
                                # 注意：这里用简单的矩形，如果需要更精确可以用轮廓
                                cv2.rectangle(blue_mask, 
                                            (x_reg, y_reg), 
                                            (x_reg + w_reg, y_reg + h_reg), 
                                            255, -1)
                        except:
                            blue_mask = None
                        
                        # 使用高精度方法测量
                        measured_dimensions = self.measure_box_dimensions_3d(
                            bbox_for_measurement,
                            depth_image,
                            mask=blue_mask,
                            sample_step=2  # 每2个像素采样一次，平衡精度和速度
                        )
                        
                        if measured_dimensions:
                            box_3d_size = {
                                'width': measured_dimensions['width'],
                                'height': measured_dimensions['height'],
                                'depth': measured_dimensions['depth']
                            }
                            # 如果有测量结果，使用更精确的中心点
                            if 'center_3d' in measured_dimensions:
                                merged_point_3d = measured_dimensions['center_3d']
                        else:
                            # 如果高精度方法失败，回退到简单估算
                            depth_m = avg_depth_mm * 0.001
                            pixel_size = depth_m / self.intrinsics.fx if self.intrinsics else 0.001
                            box_3d_size = {
                                'width': merged_w * pixel_size,
                                'height': merged_h * pixel_size,
                                'depth': (max(valid_depths) - min(valid_depths)) * 0.001 if len(valid_depths) > 1 else 0.1
                            }
                    
                    # 保存完整的蓝色盒子信息
                    blue_box_info = {
                        'id': blue_box_idx,
                        'type': 'blue_box',
                        'num_faces': len(group),  # 检测到的面数
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
                        'faces': group  # 保存所有面的信息
                    }
                    blue_box_coordinates.append(blue_box_info)
                    
                    # 调试信息
                    if frame_count <= 5:
                        method_used = "密集采样（高精度）" if measured_dimensions else "简单估算（回退）"
                        print(f'    ✓ 蓝色盒子 {blue_box_idx}: {len(group)}个面, 总面积={merged_area:.0f}, '
                              f'合并尺寸={merged_w}x{merged_h}, 深度={avg_depth_mm}mm')
                        if merged_point_3d:
                            print(f'      3D位置: X={merged_point_3d[0]:.3f}m, Y={merged_point_3d[1]:.3f}m, Z={merged_point_3d[2]:.3f}m')
                        if box_3d_size:
                            print(f'      3D尺寸: 宽度={box_3d_size["width"]:.3f}m, '
                                  f'高度={box_3d_size["height"]:.3f}m, '
                                  f'深度={box_3d_size["depth"]:.3f}m [{method_used}]')
                            if measured_dimensions and 'num_samples' in measured_dimensions:
                                print(f'      采样点数: {measured_dimensions["num_samples"]} (步长=2像素)')
                    
                    blue_box_idx += 1
                
                # 调试信息
                if len(blue_box_coordinates) > 0:
                    if frame_count <= 5 or frame_count % 30 == 0:
                        print(f'  📦 检测到 {len(blue_box_coordinates)} 个蓝色盒子')
                elif frame_count <= 5:
                    print(f'  ⚠️ 未检测到蓝色盒子（检查HSV范围和最小面积设置）')
            
            # 创建调试图像：显示绿色、黄色和蓝色检测结果
            debug_thresh = np.zeros((crop_img.shape[0], crop_img.shape[1], 3), dtype=np.uint8)
            debug_thresh[:, :, 1] = thresh_green  # 绿色通道显示绿色检测
            debug_thresh[:, :, 0] = thresh_yellow_full  # 蓝色通道显示黄色检测
            debug_thresh[:, :, 2] = thresh_blue_full  # 红色通道显示蓝色检测
            debug_images['threshold'] = debug_thresh
            
            # 形态学处理
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh_morph = cv2.morphologyEx(thresh_binary, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh_morph = cv2.morphologyEx(thresh_morph, cv2.MORPH_OPEN, kernel, iterations=1)
            debug_images['morphology'] = cv2.cvtColor(thresh_morph * 255, cv2.COLOR_GRAY2BGR)
            
            # 轮廓检测
            contours, _ = cv2.findContours(thresh_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 绘制所有轮廓（用于调试）
            contour_img = crop_img.copy()
            cv2.drawContours(contour_img, contours, -1, (0, 255, 255), 2)
            debug_images['contours_all'] = contour_img.copy()
            
            if len(contours) == 0:
                # 即使没有轮廓，也保存空的valid contours图像
                debug_images['contours_valid'] = crop_img.copy()
                # 即使没有叶子，也可能有蓝色盒子
                if len(blue_box_coordinates) > 0:
                    result = f"检测到 {len(blue_box_coordinates)} 个蓝色盒子（无叶子）"
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
                return "未检测到叶子", None, None, debug_images
            
            # 过滤参数
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
            
            # 绘制有效轮廓（在轮廓过滤后）
            valid_contour_img = crop_img.copy()
            cv2.drawContours(valid_contour_img, valid_contours, -1, (0, 255, 0), 2)
            
            # 添加文本显示轮廓数量（放在右下角，不遮挡）
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
                # 即使没有有效叶子，也可能有蓝色盒子
                if len(blue_box_coordinates) > 0:
                    result = f"检测到 {len(blue_box_coordinates)} 个蓝色盒子（无有效叶子）"
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
                return "未检测到有效叶子", None, None, debug_images
            
            # 步骤3: 对于每个检测到的绿色叶子，在其区域内检测黄色胶布
            # 同时检测独立的黄色区域（如果形状符合叶子特征）
            leaf_coordinates = []
            bounding_boxes = []
            leaf_idx = 1
            
            # 调试信息
            if frame_count <= 5:
                print(f'  开始处理: {len(valid_contours)} 个绿色轮廓')
            
            # 处理绿色叶子轮廓
            for cnt in valid_contours:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # 计算中心点
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00']) + 20
                    cy = int(M['m01'] / M['m00']) + 20
                else:
                    cx = x + w_rect // 2 + 20
                    cy = y + h_rect // 2 + 20
                
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # 获取深度值和3D坐标（使用更大的区域以提高稳定性）
                depth_value_mm = 0
                point_3d = None
                has_valid_depth = False
                
                if depth_image is not None:
                    try:
                        if cx < depth_image.shape[1] and cy < depth_image.shape[0]:
                            # 使用5x5区域的平均值，提高稳定性
                            y_start = max(0, cy - 2)
                            y_end = min(depth_image.shape[0], cy + 3)
                            x_start = max(0, cx - 2)
                            x_end = min(depth_image.shape[1], cx + 3)
                            
                            depth_region = depth_image[y_start:y_end, x_start:x_end]
                            # 过滤掉无效深度值（0值）
                            valid_depths = depth_region[depth_region > 0]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.mean(valid_depths))
                                
                                # 进一步放宽深度范围检查
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    has_valid_depth = True
                                    point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                            else:
                                # 如果没有有效深度，尝试使用轮廓区域内的深度
                                # 在轮廓边界框内采样多个点
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
                        # 调试信息：只在第一帧打印
                        if frame_count <= 3 and leaf_idx == 1:
                            print(f'  深度获取错误: {e}')
                        pass
                
                # 如果深度过滤失败，但仍然保留轮廓（用于调试，至少能看到检测框）
                # 但标记为无深度信息
                if not has_valid_depth:
                    # 调试信息：显示为什么被过滤
                    if frame_count <= 5:  # 前5帧打印调试信息
                        print(f'  轮廓 {leaf_idx} 深度无效: cx={cx}, cy={cy}, depth={depth_value_mm}mm, 尝试使用轮廓区域')
                    
                    # 即使深度无效，也保留轮廓（但标记深度为0）
                    # 这样至少能看到检测框
                    depth_value_mm = 0
                    point_3d = None
                    # 不continue，继续处理这个轮廓
                
                # 检查这个叶子区域内是否包含黄色胶布
                has_yellow_tape = False
                yellow_ratio = 0.0
                
                if self.detect_yellow and thresh_yellow_full is not None:
                    # 创建叶子区域的掩码（使用完整的轮廓，而不是边界框）
                    # 这样可以更准确地检测叶子内部的黄色区域
                    leaf_mask = np.zeros_like(thresh_green, dtype=np.uint8)
                    cv2.drawContours(leaf_mask, [cnt], -1, 255, -1)
                    
                    # 直接在完整掩码上计算，而不是使用边界框区域
                    # 这样可以避免边界框可能包含非叶子区域的问题
                    leaf_pixels = np.sum(leaf_mask > 0)
                    yellow_pixels = np.sum((leaf_mask > 0) & (thresh_yellow_full > 0))
                    
                    if leaf_pixels > 0:
                        yellow_ratio = yellow_pixels / leaf_pixels
                        
                        # 额外的验证：检查黄色区域的连通性和大小
                        # 真正的黄色胶布应该是连续的、有一定大小的区域
                        yellow_in_leaf = (leaf_mask > 0) & (thresh_yellow_full > 0)
                        yellow_contours, _ = cv2.findContours(
                            yellow_in_leaf.astype(np.uint8), 
                            cv2.RETR_EXTERNAL, 
                            cv2.CHAIN_APPROX_SIMPLE
                        )
                        
                        # 计算最大黄色连通区域的面积
                        max_yellow_area = 0
                        if len(yellow_contours) > 0:
                            max_yellow_area = max(cv2.contourArea(c) for c in yellow_contours)
                        
                        # 降低最小区域要求：至少50像素或叶子面积的0.5%（更容易检测）
                        min_yellow_area_threshold = max(50, leaf_pixels * 0.005)
                        
                        # 使用OR条件：满足比例阈值或最小区域要求之一即可（更容易检测）
                        if (yellow_ratio >= self.yellow_ratio_threshold or 
                            max_yellow_area >= min_yellow_area_threshold):
                            has_yellow_tape = True
                            
                            # 调试日志：记录检测到的黄色标签信息
                            if frame_count <= 5 or frame_count % 30 == 0:
                                print(f'  🎯 Leaf {leaf_idx}: 检测到黄色标签 - '
                                      f'yellow_ratio={yellow_ratio:.4f} '
                                      f'(阈值={self.yellow_ratio_threshold:.4f}), '
                                      f'黄色像素={yellow_pixels}/{leaf_pixels}, '
                                      f'最大连通区域={max_yellow_area:.0f}像素')
                        else:
                            # 调试日志：记录未通过验证的情况
                            if yellow_ratio > 0.01 and (frame_count <= 5 or frame_count % 30 == 0):
                                reason = []
                                if yellow_ratio < self.yellow_ratio_threshold:
                                    reason.append(f'比例不足({yellow_ratio:.4f}<{self.yellow_ratio_threshold:.4f})')
                                if max_yellow_area < min_yellow_area_threshold:
                                    reason.append(f'区域太小({max_yellow_area:.0f}<{min_yellow_area_threshold:.0f})')
                                print(f'  ⚠️ Leaf {leaf_idx}: 检测到黄色但未通过验证 - '
                                      f'yellow_ratio={yellow_ratio:.4f}, '
                                      f'最大连通区域={max_yellow_area:.0f}像素, '
                                      f'原因: {", ".join(reason)}')
                    else:
                        # 调试：如果叶子区域为空，记录警告
                        if frame_count <= 5:
                            print(f'  ⚠️ Leaf {leaf_idx}: 叶子区域掩码为空，无法检测黄色标签 '
                                  f'(bbox: x={x}, y={y}, w={w_rect}, h={h_rect})')
                
                # 保存叶子信息
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
            
            # 调试信息
            if frame_count <= 5:
                print(f'  深度过滤后: {num_detected_leaves} 片叶子（原始: {len(valid_contours)} 个轮廓）')
            
            # 统计黄色标签检测结果
            yellow_tape_count = sum(1 for leaf in leaf_coordinates if leaf.get('has_yellow_tape', False))
            if yellow_tape_count > 0:
                print(f'  📊 检测总结: 共{num_detected_leaves}片叶子, '
                      f'其中{yellow_tape_count}片检测到黄色标签')
                # 列出所有有黄色标签的叶子
                for leaf in leaf_coordinates:
                    if leaf.get('has_yellow_tape', False):
                        print(f'    ✓ Leaf {leaf["id"]}: yellow_ratio={leaf.get("yellow_ratio", 0):.4f}')
            
            if num_detected_leaves == 0:
                # 即使深度过滤失败，也显示轮廓（用于调试）
                # 创建一个显示所有有效轮廓的结果图像
                result_img = crop_img.copy()
                cv2.drawContours(result_img, valid_contours, -1, (0, 255, 255), 2)  # 黄色轮廓
                
                # 添加调试信息
                h_img, w_img = result_img.shape[:2]
                cv2.putText(result_img, f'No depth info, showing contours', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(result_img, f'Valid contours: {len(valid_contours)}', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 为每个轮廓添加编号
                for idx, cnt in enumerate(valid_contours):
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00']) + 20
                        cy = int(M['m01'] / M['m00']) + 20
                        cv2.putText(result_img, f'{idx+1}', (cx-10, cy),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                debug_images['result'] = result_img
                # 即使没有有效叶子，也可能有蓝色盒子
                if len(blue_box_coordinates) > 0:
                    result = f"检测到 {len(blue_box_coordinates)} 个蓝色盒子（叶子深度过滤后）"
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
                return "未检测到有效叶子（深度过滤后）", None, None, debug_images
            
            result = f"检测到 {num_detected_leaves} 片叶子"
            if len(blue_box_coordinates) > 0:
                result += f", {len(blue_box_coordinates)} 个蓝色盒子"
            
            # 生成MoveIt格式的坐标
            moveit_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id="world")
            
            leaf_data = {
                'num_leaves': num_detected_leaves,
                'num_blue_boxes': len(blue_box_coordinates),
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates,
                'blue_boxes': blue_box_coordinates,
                'moveit_collision_objects': moveit_objects  # 添加MoveIt格式坐标
            }
            
            return result, leaf_data, bounding_boxes, debug_images
            
        except Exception as e:
            print(f'✗ PlantCV检测错误: {str(e)}')
            import traceback
            traceback.print_exc()
            return f"检测错误: {str(e)}", None, None, debug_images
    
    def draw_annotations(self, display_frame, leaf_coordinates, blue_box_coordinates=None):
        """
        在图像上绘制标注
        
        Args:
            display_frame: 要绘制的图像
            leaf_coordinates: 叶子坐标列表
            blue_box_coordinates: 蓝色盒子坐标列表（可选）
        
        Returns:
            标注后的图像
        """
        if display_frame is None:
            return None
        
        # 始终返回帧的副本（带或不带标注）
        annotated = display_frame.copy()
        
        if blue_box_coordinates is None:
            blue_box_coordinates = []
        
        if not leaf_coordinates and not blue_box_coordinates:
            # 未检测到任何对象，只返回原始帧
            return annotated
        
        # 为检测到的叶子绘制标注
        try:
            colors = [
                (255, 0, 0),      # 蓝色
                (0, 255, 0),       # 绿色
                (0, 0, 255),       # 红色
                (255, 255, 0),     # 青色
                (255, 0, 255),     # 洋红色
                (0, 255, 255),     # 黄色
            ]
            
            for leaf in leaf_coordinates:
                obj_id = leaf['id']
                color = colors[(obj_id - 1) % len(colors)]
                
                bbox = leaf['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # 绘制边界框
                cv2.rectangle(annotated, (x, y), (x_max, y_max), color, 2)
                
                # 绘制标签（如果有胶布，添加标记）
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
                
                # 绘制中心点
                cx = leaf['center']['x']
                cy = leaf['center']['y']
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                cv2.circle(annotated, (cx, cy), 15, color, 2)
                
                # 如果可用，绘制3D坐标
                point_3d = leaf.get('point_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # 绘制蓝色盒子（显示所有检测到的面）
            blue_box_color = (255, 0, 0)  # 蓝色用于蓝色盒子 (BGR格式)
            face_color = (200, 100, 100)  # 浅蓝色用于单个面
            for blue_box in blue_box_coordinates:
                obj_id = blue_box['id']
                bbox = blue_box['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # 绘制合并后的完整边界框（粗线）
                cv2.rectangle(annotated, (x, y), (x_max, y_max), blue_box_color, 3)
                
                # 绘制每个检测到的面（细线）
                num_faces = blue_box.get('num_faces', 1)
                faces = blue_box.get('faces', [])
                if len(faces) > 0:
                    # 绘制每个面的轮廓
                    for face_idx, face in enumerate(faces):
                        contour = face.get('contour')
                        if contour is not None:
                            # 调整轮廓坐标（加上裁剪偏移）
                            adjusted_contour = contour + np.array([20, 20])
                            cv2.drawContours(annotated, [adjusted_contour], -1, face_color, 1)
                
                # 绘制标签（显示面数）
                label = f'Blue Box {obj_id}'
                if num_faces > 1:
                    label += f' ({num_faces} faces)'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5),
                            (x + label_size[0] + 5, label_y + 5), blue_box_color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 绘制中心点
                cx = blue_box['center']['x']
                cy = blue_box['center']['y']
                cv2.circle(annotated, (cx, cy), 5, blue_box_color, -1)
                cv2.circle(annotated, (cx, cy), 15, blue_box_color, 2)
                
                # 如果可用，绘制3D坐标和尺寸
                point_3d = blue_box.get('point_3d')
                size_3d = blue_box.get('size_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    if size_3d:
                        coord_text += f" ({size_3d['width']:.2f}x{size_3d['height']:.2f}x{size_3d['depth']:.2f}m)"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, blue_box_color, 1)
            
            # 显示总数
            total_text = f"Leaves: {len(leaf_coordinates)}"
            if len(blue_box_coordinates) > 0:
                total_text += f" | Blue Boxes: {len(blue_box_coordinates)}"
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return annotated
            
        except Exception as e:
            print(f'✗ 标注绘制错误: {e}')
            import traceback
            traceback.print_exc()
            return display_frame.copy()
    
    def get_moveit_collision_objects(self, blue_box_coordinates, frame_id="world"):
        """
        将蓝色盒子坐标转换为MoveIt CollisionObject格式
        
        Args:
            blue_box_coordinates: 蓝色盒子坐标列表
            frame_id: 坐标系ID（默认: "world"）
        
        Returns:
            MoveIt CollisionObject格式的字典列表
        """
        collision_objects = []
        
        for box in blue_box_coordinates:
            point_3d = box.get('point_3d')
            size_3d = box.get('size_3d')
            
            if point_3d is None:
                continue
            
            # 获取3D尺寸，如果没有则使用默认值
            if size_3d:
                width = size_3d['width']
                height = size_3d['height']
                depth = size_3d['depth']
            else:
                # 使用默认尺寸（基于像素估算）
                bbox = box.get('bounding_box', {})
                pixel_w = bbox.get('width', 100)
                pixel_h = bbox.get('height', 100)
                depth_mm = box.get('depth_mm', 1000)
                
                # 估算3D尺寸
                depth_m = depth_mm * 0.001
                if self.intrinsics:
                    pixel_size = depth_m / self.intrinsics.fx
                    width = pixel_w * pixel_size
                    height = pixel_h * pixel_size
                    depth = 0.1  # 默认深度10cm
                else:
                    width = 0.1
                    height = 0.1
                    depth = 0.1
            
            # 创建CollisionObject格式
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
        打印MoveIt格式的坐标（用于ROS2 topic pub命令）
        
        Args:
            blue_box_coordinates: 蓝色盒子坐标列表
            frame_id: 坐标系ID（默认: "world"）
        """
        collision_objects = self.get_moveit_collision_objects(blue_box_coordinates, frame_id)
        
        if len(collision_objects) == 0:
            print("未检测到蓝色盒子")
            return
        
        print("\n" + "="*80)
        print("MoveIt CollisionObject 格式坐标:")
        print("="*80)
        
        for i, obj in enumerate(collision_objects):
            pos = obj['primitive_poses'][0]['position']
            dims = obj['primitives'][0]['dimensions']
            
            print(f"\n蓝色盒子 {i+1} ({obj['id']}):")
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
        创建包含所有处理步骤的拼接图像
        
        Args:
            debug_images: 包含各个处理步骤图像的字典
        
        Returns:
            拼接后的大图像
        """
        # 定义图像顺序和标签
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
        
        # 每个小图像的大小
        tile_width = 320
        tile_height = 240
        
        # 创建3x3网格（最后一行可能只有2个）
        rows = 3
        cols = 3
        
        # 创建大画布
        mosaic_height = rows * tile_height
        mosaic_width = cols * tile_width
        mosaic = np.zeros((mosaic_height, mosaic_width, 3), dtype=np.uint8)
        
        # 填充每个位置
        for idx, (key, label) in enumerate(image_order):
            row = idx // cols
            col = idx % cols
            
            if key in debug_images and debug_images[key] is not None:
                img = debug_images[key]
                if img.size > 0:
                    # 调整图像大小
                    resized = cv2.resize(img, (tile_width, tile_height))
                    
                    # 计算位置
                    y_start = row * tile_height
                    y_end = y_start + tile_height
                    x_start = col * tile_width
                    x_end = x_start + tile_width
                    
                    # 放置图像
                    mosaic[y_start:y_end, x_start:x_end] = resized
                    
                    # 添加标签
                    cv2.putText(mosaic, label, (x_start + 5, y_start + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(mosaic, label, (x_start + 5, y_start + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        return mosaic
    
    def run(self):
        """运行检测循环"""
        print('\n开始检测循环...')
        print('按 \'q\' 键退出\n')
        
        frame_count = 0
        window_name = 'Leaf Detection - All Processing Steps'
        
        # 创建单个大窗口
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 960, 720)  # 3x3网格，每个320x240
            print('✓ 可视化窗口已创建')
            print('   所有处理步骤将显示在一个窗口中')
        except Exception as e:
            print(f'⚠ 窗口创建警告: {e}')
            print('  继续运行，但窗口可能无法显示')
        
        try:
            while True:
                # 获取帧
                color_image, depth_image = self.get_frames()
                
                if color_image is None or depth_image is None:
                    if frame_count == 0:
                        print('⚠ 等待相机数据...')
                    continue
                
                frame_count += 1
                
                # 检测叶子（传递frame_count用于调试）
                detection_result, leaf_data, bounding_boxes, debug_images = self.detect_leaves(
                    color_image, depth_image, frame_count
                )
                
                # 绘制标注
                if leaf_data:
                    annotated_image = self.draw_annotations(
                        color_image,
                        leaf_data.get('coordinates', []),
                        leaf_data.get('blue_boxes', [])
                    )
                    
                    # 打印检测信息
                    if frame_count % 30 == 0:  # 每30帧打印一次
                        print(f'帧 {frame_count}: {detection_result}')
                        for i, leaf in enumerate(leaf_data.get('coordinates', [])):
                            point_3d = leaf.get('point_3d')
                            if point_3d:
                                print(f'  Leaf {i+1}: X={point_3d[0]:.3f}m, '
                                      f'Y={point_3d[1]:.3f}m, Z={point_3d[2]:.3f}m')
                        
                        # 打印蓝色盒子详细信息
                        blue_boxes = leaf_data.get('blue_boxes', [])
                        if len(blue_boxes) > 0:
                            print(f'\n{"="*80}')
                            print(f'蓝色盒子检测信息 (共 {len(blue_boxes)} 个):')
                            print(f'{"="*80}')
                            
                            for blue_box in blue_boxes:
                                box_id = blue_box["id"]
                                num_faces = blue_box.get('num_faces', 1)
                                area = blue_box.get('area', 0)
                                bbox = blue_box.get('bounding_box', {})
                                point_3d = blue_box.get('point_3d')
                                size_3d = blue_box.get('size_3d')
                                depth_mm = blue_box.get('depth_mm', 0)
                                
                                print(f'\n📦 蓝色盒子 {box_id}:')
                                print(f'  ├─ 基本信息:')
                                print(f'  │   ├─ 检测到的面数: {num_faces}')
                                print(f'  │   ├─ 总面积: {area:.0f} 像素²')
                                print(f'  │   └─ 深度: {depth_mm} mm')
                                
                                if bbox:
                                    print(f'  ├─ 2D边界框:')
                                    print(f'  │   ├─ 位置: ({bbox.get("x", 0)}, {bbox.get("y", 0)})')
                                    print(f'  │   ├─ 尺寸: {bbox.get("width", 0)} x {bbox.get("height", 0)} 像素')
                                    print(f'  │   └─ 中心: ({blue_box.get("center", {}).get("x", 0)}, {blue_box.get("center", {}).get("y", 0)})')
                                
                                if point_3d:
                                    print(f'  ├─ 3D位置 (相机坐标系):')
                                    print(f'  │   ├─ X: {point_3d[0]:.3f} m')
                                    print(f'  │   ├─ Y: {point_3d[1]:.3f} m')
                                    print(f'  │   └─ Z: {point_3d[2]:.3f} m')
                                
                                if size_3d:
                                    print(f'  ├─ 3D尺寸:')
                                    print(f'  │   ├─ 宽度: {size_3d["width"]:.3f} m')
                                    print(f'  │   ├─ 高度: {size_3d["height"]:.3f} m')
                                    print(f'  │   └─ 深度: {size_3d["depth"]:.3f} m')
                                
                                print(f'  └─ MoveIt格式:')
                                if point_3d and size_3d:
                                    print(f'      position: {{x: {point_3d[0]:.3f}, y: {point_3d[1]:.3f}, z: {point_3d[2]:.3f}}}')
                                    print(f'      dimensions: [{size_3d["width"]:.3f}, {size_3d["height"]:.3f}, {size_3d["depth"]:.3f}]')
                            
                            print(f'\n{"="*80}')
                            
                            # 打印MoveIt格式的坐标（ROS2命令）
                            self.print_moveit_format(blue_boxes, frame_id="world")
                else:
                    annotated_image = color_image.copy()
                
                # 保存最终结果到调试图像
                debug_images['result'] = annotated_image.copy()
                
                # 检查图像是否有效
                if annotated_image is None or annotated_image.size == 0:
                    print('⚠ 图像无效，跳过显示')
                    continue
                
                # 创建拼接图像
                try:
                    mosaic = self.create_debug_mosaic(debug_images)
                    
                    # 显示拼接后的图像
                    cv2.imshow(window_name, mosaic)
                    
                    # 首次显示时打印提示
                    if frame_count == 1:
                        print('✓ 图像显示已启动')
                        print('   所有处理步骤显示在一个窗口中')
                        print('   如果看不到窗口，请检查：')
                        print('   1. 窗口是否被其他窗口遮挡')
                        print('   2. 是否在远程SSH连接（需要X11转发）')
                        print('   3. DISPLAY环境变量是否正确设置')
                except Exception as e:
                    print(f'✗ 显示图像失败: {e}')
                    import traceback
                    traceback.print_exc()
                    print('   提示: 如果使用SSH，请使用 X11 转发: ssh -X user@host')
                    break
                
                # 按'q'退出
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print('\n用户按下 \'q\' 键，退出程序')
                    break
                elif key == 27:  # ESC键
                    print('\n用户按下 ESC 键，退出程序')
                    break
                
        except KeyboardInterrupt:
            print('\n用户中断 (Ctrl+C)')
        except Exception as e:
            print(f'\n✗ 运行错误: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # 清理
            try:
                cv2.destroyAllWindows()
            except:
                pass
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
            print('\n✓ 程序退出')


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='独立叶子检测和显示脚本（不依赖ROS2）'
    )
    parser.add_argument(
        '--min-area',
        type=float,
        default=2000,
        help='最小叶子面积阈值（默认: 2000）'
    )
    parser.add_argument(
        '--no-yellow',
        action='store_true',
        help='禁用黄色胶布检测（只检测绿色叶子）'
    )
    parser.add_argument(
        '--yellow-threshold',
        type=float,
        default=0.05,
        help='黄色比例阈值（默认: 0.05，即5%%，降低以更容易检测黄色胶布）'
    )
    parser.add_argument(
        '--yellow-hsv-lower',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[20, 100, 100],
        help='HSV颜色范围下限（默认: 20 100 100，更严格以检测亮黄色胶布）'
    )
    parser.add_argument(
        '--yellow-hsv-upper',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[30, 255, 255],
        help='HSV颜色范围上限（默认: 30 255 255）'
    )
    parser.add_argument(
        '--no-blue-box',
        action='store_true',
        help='禁用蓝色盒子检测（只检测叶子）'
    )
    parser.add_argument(
        '--blue-min-area',
        type=float,
        default=3000,
        help='蓝色盒子最小面积阈值（默认: 3000）'
    )
    parser.add_argument(
        '--blue-hsv-lower',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[100, 147, 50],
        help='蓝色HSV颜色范围下限（默认: 100 147 50，S值提高以减少误检）'
    )
    parser.add_argument(
        '--blue-hsv-upper',
        type=int,
        nargs=3,
        metavar=('H', 'S', 'V'),
        default=[130, 255, 255],
        help='蓝色HSV颜色范围上限（默认: 130 255 255）'
    )
    
    args = parser.parse_args()
    
    # 创建检测器
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
    
    # 设置相机
    try:
        detector.setup_camera()
    except Exception as e:
        print(f'✗ 相机初始化失败: {e}')
        print('请确保RealSense相机已连接并正确配置')
        sys.exit(1)
    
    # 运行检测循环
    detector.run()


if __name__ == '__main__':
    main()

