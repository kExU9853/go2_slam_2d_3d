#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <cmath>
#include <vector>

class LidarTimeAdapter : public rclcpp::Node
{
public:
    LidarTimeAdapter() : Node("lidar_time_adapter")
    {
        // 声明参数
        this->declare_parameter("remove_nan_points", true);
        this->declare_parameter("publish_clean_topic", false);
        this->declare_parameter("clean_topic_name", "/lidar_points_clean");
        
        // 获取参数
        remove_nan_points_ = this->get_parameter("remove_nan_points").as_bool();
        publish_clean_topic_ = this->get_parameter("publish_clean_topic").as_bool();
        clean_topic_name_ = this->get_parameter("clean_topic_name").as_string();
        
        // 创建订阅者和发布者
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10, std::bind(&LidarTimeAdapter::pointcloud_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points_ready", 10);
        
        // 如果启用清洗话题，创建额外的发布者
        if (publish_clean_topic_)
        {
            clean_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(clean_topic_name_, 10);
        }
        
        RCLCPP_INFO(this->get_logger(), "Lidar Time Adapter node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /lidar_points");
        RCLCPP_INFO(this->get_logger(), "Publishing to /lidar_points_ready");
        RCLCPP_INFO(this->get_logger(), "Remove NaN points: %s", remove_nan_points_ ? "enabled" : "disabled");
        if (publish_clean_topic_)
        {
            RCLCPP_INFO(this->get_logger(), "Clean topic enabled: %s", clean_topic_name_.c_str());
        }
    }

private:
    // 检查点是否为NaN
    bool is_point_nan(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t point_index, 
                      int x_offset, int y_offset, int z_offset)
    {
        size_t data_index = point_index * msg->point_step;
        
        float x = *reinterpret_cast<const float*>(&msg->data[data_index + x_offset]);
        float y = *reinterpret_cast<const float*>(&msg->data[data_index + y_offset]);
        float z = *reinterpret_cast<const float*>(&msg->data[data_index + z_offset]);
        
        return std::isnan(x) || std::isnan(y) || std::isnan(z) || 
               std::isinf(x) || std::isinf(y) || std::isinf(z);
    }
    
    // 清洗NaN点并返回有效点的索引
    std::vector<size_t> filter_nan_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 找到x, y, z字段的偏移
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : msg->fields)
        {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }
        
        if (x_offset == -1 || y_offset == -1 || z_offset == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find x, y, z fields, skipping NaN removal");
            std::vector<size_t> all_indices;
            for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
            {
                all_indices.push_back(i);
            }
            return all_indices;
        }
        
        // 统计有效点
        std::vector<size_t> valid_indices;
        for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
        {
            size_t point_index = i / msg->point_step;
            if (!is_point_nan(msg, point_index, x_offset, y_offset, z_offset))
            {
                valid_indices.push_back(i);
            }
        }
        
        if (valid_indices.size() != msg->data.size() / msg->point_step)
        {
            RCLCPP_INFO(this->get_logger(), "Removed %zu NaN points, kept %zu valid points", 
                       (msg->data.size() / msg->point_step) - valid_indices.size(), valid_indices.size());
        }
        
        return valid_indices;
    }
    
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 首先进行NaN清洗
        std::vector<size_t> valid_indices;
        if (remove_nan_points_)
        {
            valid_indices = filter_nan_points(msg);
        }
        else
        {
            // 如果不清洗NaN，使用所有点
            for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
            {
                valid_indices.push_back(i);
            }
        }
        
        if (valid_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid points after NaN filtering, skipping...");
            return;
        }
        
        // 检查是否有timestamp字段
        bool has_timestamp = false;
        int timestamp_offset = -1;
        int timestamp_datatype = -1;
        
        for (const auto& field : msg->fields)
        {
            if (field.name == "timestamp")
            {
                has_timestamp = true;
                timestamp_offset = field.offset;
                timestamp_datatype = field.datatype;
                break;
            }
        }
        
        if (!has_timestamp)
        {
            RCLCPP_WARN(this->get_logger(), "No timestamp field found in point cloud, skipping...");
            return;
        }
        
        // 创建新的点云消息
        sensor_msgs::msg::PointCloud2 output_msg = *msg;
        
        // 更新点云属性
        output_msg.width = valid_indices.size();
        output_msg.height = 1;
        output_msg.is_dense = true;  // 确保是dense
        
        // 添加time字段
        sensor_msgs::msg::PointField time_field;
        time_field.name = "time";
        time_field.offset = msg->point_step;  // 在现有字段后添加
        time_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        time_field.count = 1;
        
        output_msg.fields.push_back(time_field);
        output_msg.point_step += 4;  // float32 = 4 bytes
        output_msg.row_step = output_msg.point_step * output_msg.width;
        
        // 重新分配数据数组
        output_msg.data.resize(valid_indices.size() * output_msg.point_step);
        
        // 处理每个点
        uint64_t first_timestamp = 0;
        bool first_timestamp_set = false;
        
        // 首先找到第一个有效的时间戳
        for (size_t valid_idx : valid_indices)
        {
            uint64_t timestamp = 0;
            if (timestamp_datatype == sensor_msgs::msg::PointField::UINT32)
            {
                timestamp = *reinterpret_cast<const uint32_t*>(&msg->data[valid_idx + timestamp_offset]);
            }
            else if (timestamp_datatype == sensor_msgs::msg::PointField::FLOAT64)
            {
                timestamp = static_cast<uint64_t>(*reinterpret_cast<const double*>(&msg->data[valid_idx + timestamp_offset]));
            }
            else if (timestamp_datatype == sensor_msgs::msg::PointField::FLOAT32)
            {
                timestamp = static_cast<uint64_t>(*reinterpret_cast<const float*>(&msg->data[valid_idx + timestamp_offset]));
            }
            
            if (timestamp > 0)
            {
                first_timestamp = timestamp;
                first_timestamp_set = true;
                break;
            }
        }
        
        if (!first_timestamp_set)
        {
            RCLCPP_WARN(this->get_logger(), "No valid timestamp found, skipping...");
            return;
        }
        
        // 复制数据并添加时间字段
        for (size_t i = 0; i < valid_indices.size(); ++i)
        {
            size_t valid_idx = valid_indices[i];
            size_t output_idx = i * output_msg.point_step;
            
            // 复制原始数据
            std::memcpy(&output_msg.data[output_idx], &msg->data[valid_idx], msg->point_step);
            
            // 计算相对时间
            uint64_t timestamp = 0;
            if (timestamp_datatype == sensor_msgs::msg::PointField::UINT32)
            {
                timestamp = *reinterpret_cast<const uint32_t*>(&msg->data[valid_idx + timestamp_offset]);
            }
            else if (timestamp_datatype == sensor_msgs::msg::PointField::FLOAT64)
            {
                timestamp = static_cast<uint64_t>(*reinterpret_cast<const double*>(&msg->data[valid_idx + timestamp_offset]));
            }
            else if (timestamp_datatype == sensor_msgs::msg::PointField::FLOAT32)
            {
                timestamp = static_cast<uint64_t>(*reinterpret_cast<const float*>(&msg->data[valid_idx + timestamp_offset]));
            }
            
            // 计算相对时间（秒）
            float relative_time = 0.0f;
            if (timestamp > first_timestamp)
            {
                // 假设时间戳是纳秒，转换为秒
                relative_time = static_cast<float>(timestamp - first_timestamp) / 1e9f;
            }
            
            // 添加时间字段
            float* time_ptr = reinterpret_cast<float*>(&output_msg.data[output_idx + msg->point_step]);
            *time_ptr = relative_time;
        }
        
        // 如果启用了清洗话题，发布清洗后的点云（不包含time字段）
        if (publish_clean_topic_ && clean_publisher_)
        {
            sensor_msgs::msg::PointCloud2 clean_msg = *msg;
            clean_msg.width = valid_indices.size();
            clean_msg.height = 1;
            clean_msg.is_dense = true;
            clean_msg.row_step = clean_msg.point_step * clean_msg.width;
            clean_msg.data.resize(valid_indices.size() * clean_msg.point_step);
            
            for (size_t i = 0; i < valid_indices.size(); ++i)
            {
                std::memcpy(&clean_msg.data[i * clean_msg.point_step], 
                           &msg->data[valid_indices[i]], clean_msg.point_step);
            }
            
            clean_publisher_->publish(clean_msg);
        }
        
        // 发布处理后的点云
        publisher_->publish(output_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Processed point cloud with %zu points, first timestamp: %lu", 
                     valid_indices.size(), first_timestamp);
    }
    
    // 成员变量
    bool remove_nan_points_;
    bool publish_clean_topic_;
    std::string clean_topic_name_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clean_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarTimeAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
