# RViz2 泵状态可视化说明

## 功能说明

`leafServerNode` 现在会自动发布泵的开关状态到 RViz2，以文字形式显示在 3D 视图中。

## 显示内容

- **Vacuum Pump（真空泵）**: 显示在 z=1.5m 位置
  - 绿色文字 = ON（开启）
  - 红色文字 = OFF（关闭）

- **Spray Pump（喷雾泵）**: 显示在 z=1.35m 位置
  - 绿色文字 = ON（开启）
  - 红色文字 = OFF（关闭）

## 在 RViz2 中启用可视化

1. **启动系统**（确保 `leafServerNode` 正在运行）

2. **打开 RViz2**（通常通过 MoveIt launch 文件自动启动）

3. **添加 Marker 显示**：
   - 点击左侧面板的 "Add" 按钮
   - 选择 "By topic"
   - 找到并选择 `/pump_status_marker` topic
   - 点击 "OK"

4. **查看状态**：
   - 文字会显示在 3D 视图中的机器人上方
   - 状态会实时更新（每 100ms 更新一次）
   - 当泵开关状态改变时，文字颜色会立即更新

## 文字位置

- 文字显示在 `base_link` 坐标系中
- 位置：x=0.0, y=0.0, z=1.5m（真空泵）和 z=1.35m（喷雾泵）
- 文字始终面向相机（TEXT_VIEW_FACING 类型）

## 自定义位置

如果需要调整文字显示位置，可以修改 `leafServer.cpp` 中的 `update_visualization()` 方法：

```cpp
vacuum_marker.pose.position.x = 0.0;  // X 坐标
vacuum_marker.pose.position.y = 0.0;  // Y 坐标
vacuum_marker.pose.position.z = 1.5;  // Z 坐标（高度）
```

## 注意事项

- 确保 RViz2 的 Fixed Frame 设置为 `base_link` 或相应的坐标系
- 如果看不到文字，检查 RViz2 的 Marker 显示是否已启用
- 文字大小可以通过修改 `scale.z` 参数调整（当前为 0.15）

