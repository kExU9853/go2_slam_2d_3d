# Lidar Time Adapter

This ROS2 node processes lidar point cloud data and adds relative time fields.

## Features

- Subscribes to `/lidar_points` topic
- **NaN Point Filtering**: Automatically detects and removes points containing NaN or Inf values, ensuring the point cloud is dense
- Reads existing `timestamp` field (supports multiple data types)
- Calculates relative time (seconds) from the first point in the frame
- Adds new `time` field (float32 type)
- Publishes to `/lidar_points_ready` topic
- **Optional**: Publishes cleaned point cloud to `/lidar_points_clean` topic

## Supported Timestamp Data Types

- UINT32
- FLOAT64
- FLOAT32

## Build

```bash
# In workspace root directory
colcon build --packages-select lidar_time_adapter
```

## Run

### Method 1: Direct node execution
```bash
source install/setup.bash
ros2 run lidar_time_adapter lidar_time_adapter_node
```

### Method 2: Using launch file
```bash
source install/setup.bash
ros2 launch lidar_time_adapter lidar_time_adapter.launch.py
```

## Topics

### Subscribed Topics
- `/lidar_points` (sensor_msgs/msg/PointCloud2)
  - Input point cloud data, must contain `timestamp` field

### Published Topics
- `/lidar_points_ready` (sensor_msgs/msg/PointCloud2)
  - Output point cloud data with original fields + new `time` field, NaN points filtered
- `/lidar_points_clean` (sensor_msgs/msg/PointCloud2) [Optional]
  - Point cloud data after NaN filtering only, without time field

## Output Format

The output point cloud will contain the following fields:
- x, y, z: coordinates
- intensity: intensity values
- ring: laser ring number
- timestamp: original timestamp
- time: relative time (seconds, relative to first point in frame)

## Test

### Run Tests
```bash
# Method 1: Using test launch file
ros2 launch lidar_time_adapter test_lidar_time_adapter.launch.py

# Method 2: Manual execution
# Terminal 1: Run main node
ros2 run lidar_time_adapter lidar_time_adapter_node

# Terminal 2: Run test script
python3 src/lidar_time_adapter/test/test_lidar_time_adapter.py
```

### Verify Output
The test script will:
1. Publish test point cloud with timestamp field to `/lidar_points`
2. Subscribe to `/lidar_points_ready` topic
3. Verify that output point cloud contains new `time` field

## Parameter Configuration

- `remove_nan_points` (bool, default: true)
  - Whether to enable NaN point filtering
- `publish_clean_topic` (bool, default: false)
  - Whether to publish cleaned point cloud to additional topic
- `clean_topic_name` (string, default: "/lidar_points_clean")
  - Topic name for cleaned point cloud

## Notes

1. Input point cloud must contain `timestamp` field
2. Timestamp is assumed to be in nanoseconds and will be converted to seconds
3. If no valid timestamp is found, the node will skip the frame and output a warning
4. Supported timestamp data types: UINT32, FLOAT64, FLOAT32
5. **NaN Filtering**: Automatically detects and removes NaN and Inf values in x, y, z coordinates
6. **Dense Point Cloud**: Output point cloud is guaranteed to be dense (is_dense = true) with no invalid points
