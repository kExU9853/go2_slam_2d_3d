# Lidar Time Adapter

这个ROS2节点用于处理激光雷达点云数据，添加相对时间字段。

## 功能

- 订阅 `/lidar_points` 话题
- **NaN点清洗**：自动检测并移除包含NaN或Inf值的点，确保点云是dense
- 读取现有的 `timestamp` 字段（支持多种数据类型）
- 计算相对于本帧首点的时间（秒）
- 添加新的 `time` 字段（float32类型）
- 发布到 `/lidar_points_ready` 话题
- **可选**：发布清洗后的点云到 `/lidar_points_clean` 话题

## 支持的timestamp数据类型

- UINT64
- UINT32  
- FLOAT64
- FLOAT32

## 编译

```bash
# 在工作空间根目录
colcon build --packages-select lidar_time_adapter
```

## 运行

### 方法1：直接运行节点
```bash
source install/setup.bash
ros2 run lidar_time_adapter lidar_time_adapter_node
```

### 方法2：使用启动文件
```bash
source install/setup.bash
ros2 launch lidar_time_adapter lidar_time_adapter.launch.py
```

## 话题

### 订阅话题
- `/lidar_points` (sensor_msgs/msg/PointCloud2)
  - 输入点云数据，必须包含 `timestamp` 字段

### 发布话题
- `/lidar_points_ready` (sensor_msgs/msg/PointCloud2)
  - 输出点云数据，包含原始字段 + 新的 `time` 字段，已清洗NaN点
- `/lidar_points_clean` (sensor_msgs/msg/PointCloud2) [可选]
  - 仅清洗NaN点后的点云数据，不包含time字段

## 输出格式

输出的点云将包含以下字段：
- x, y, z: 坐标
- intensity: 强度
- ring: 激光线束编号
- timestamp: 原始时间戳
- time: 相对时间（秒，相对于本帧首点）

## 测试

### 运行测试
```bash
# 方法1：使用测试启动文件
ros2 launch lidar_time_adapter test_lidar_time_adapter.launch.py

# 方法2：手动运行
# 终端1：运行主节点
ros2 run lidar_time_adapter lidar_time_adapter_node

# 终端2：运行测试脚本
python3 src/lidar_time_adapter/test/test_lidar_time_adapter.py
```

### 验证输出
测试脚本会：
1. 发布包含timestamp字段的测试点云到 `/lidar_points`
2. 订阅 `/lidar_points_ready` 话题
3. 验证输出点云是否包含新的 `time` 字段

## 参数配置

- `remove_nan_points` (bool, default: true)
  - 是否启用NaN点清洗功能
- `publish_clean_topic` (bool, default: false)
  - 是否发布清洗后的点云到额外话题
- `clean_topic_name` (string, default: "/lidar_points_clean")
  - 清洗后点云的话题名称

## 注意事项

1. 输入点云必须包含 `timestamp` 字段
2. 时间戳假设为纳秒单位，会自动转换为秒
3. 如果找不到有效的时间戳，节点会跳过该帧并输出警告
4. 支持的timestamp数据类型：UINT32, FLOAT64, FLOAT32
5. **NaN清洗功能**：自动检测并移除x、y、z坐标中的NaN和Inf值
6. **Dense点云**：输出点云确保是dense（is_dense = true），无无效点
