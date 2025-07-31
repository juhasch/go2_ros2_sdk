#include <vector>
#include <memory>
#include <deque>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class ImprovedCloudAccumulation : public rclcpp::Node
{
public:
    ImprovedCloudAccumulation() : Node("cloud_accumulation_improved")
    {
        // Parameters
        this->declare_parameter("max_clouds", 30);
        this->declare_parameter("max_age_seconds", 2.0);
        this->declare_parameter("target_frame", "odom");
        this->declare_parameter("voxel_size", 0.05);  // 5cm voxel grid
        this->declare_parameter("min_height", 0.2);
        this->declare_parameter("max_height", 1.0);
        this->declare_parameter("publish_rate", 10.0);  // Reduced from 50Hz
        
        max_clouds_ = this->get_parameter("max_clouds").as_int();
        max_age_ = rclcpp::Duration::from_seconds(this->get_parameter("max_age_seconds").as_double());
        target_frame_ = this->get_parameter("target_frame").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        
        // TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publishers and subscribers
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "accumulated_cloud", qos);
            
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud2", qos,
            std::bind(&ImprovedCloudAccumulation::cloudCallback, this, std::placeholders::_1));
        
        // Timer for publishing
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&ImprovedCloudAccumulation::publishAccumulated, this));
            
        RCLCPP_INFO(this->get_logger(), "Improved cloud accumulation node started");
    }

private:
    struct CloudData {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud;
        rclcpp::Time timestamp;
    };
    
    void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        try {
            // Transform to target frame
            auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
            if (msg->header.frame_id != target_frame_) {
                auto transform = tf_buffer_->lookupTransform(
                    target_frame_, msg->header.frame_id, msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.1));
                tf2_sensor_msgs::doTransform(*msg, *transformed_cloud, transform);
            } else {
                *transformed_cloud = *msg;
            }
            transformed_cloud->header.frame_id = target_frame_;
            
            // Filter by height and add to collection
            auto filtered_cloud = heightFilter(transformed_cloud);
            
            // Add to deque
            CloudData cloud_data;
            cloud_data.cloud = filtered_cloud;
            cloud_data.timestamp = this->get_clock()->now();
            
            cloud_buffer_.push_back(cloud_data);
            
            // Remove old clouds
            removeOldClouds();
            
            RCLCPP_DEBUG(this->get_logger(), "Added cloud, buffer size: %zu", cloud_buffer_.size());
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }
    
    sensor_msgs::msg::PointCloud2::SharedPtr heightFilter(
        const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
    {
        auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>();
        filtered->header = cloud->header;
        filtered->height = 1;  // Unorganized
        filtered->is_dense = false;
        filtered->is_bigendian = cloud->is_bigendian;
        filtered->fields = cloud->fields;
        filtered->point_step = cloud->point_step;
        
        // Reserve space (estimate)
        filtered->data.reserve(cloud->data.size() / 2);
        
        // Iterate through points
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                continue;
            }
            
            if (*iter_z >= min_height_ && *iter_z <= max_height_) {
                // Copy the entire point (including other fields like intensity)
                const uint8_t* point_ptr = reinterpret_cast<const uint8_t*>(&(*iter_x)) - 
                                         cloud->fields[0].offset;
                filtered->data.insert(filtered->data.end(), 
                                    point_ptr, point_ptr + cloud->point_step);
                filtered->width++;
            }
        }
        
        filtered->row_step = filtered->width * filtered->point_step;
        return filtered;
    }
    
    void removeOldClouds()
    {
        auto now = this->get_clock()->now();
        
        // Remove by age
        while (!cloud_buffer_.empty() && 
               (now - cloud_buffer_.front().timestamp) > max_age_) {
            cloud_buffer_.pop_front();
        }
        
        // Remove by count
        while (cloud_buffer_.size() > static_cast<size_t>(max_clouds_)) {
            cloud_buffer_.pop_front();
        }
    }
    
    void publishAccumulated()
    {
        if (cloud_buffer_.empty()) {
            return;
        }
        
        // Create accumulated cloud
        auto accumulated = std::make_shared<sensor_msgs::msg::PointCloud2>();
        accumulated->header.stamp = this->get_clock()->now();
        accumulated->header.frame_id = target_frame_;
        accumulated->height = 1;
        accumulated->is_dense = false;
        
        if (!cloud_buffer_.empty()) {
            // Use first cloud as template
            const auto& first_cloud = cloud_buffer_.front().cloud;
            accumulated->is_bigendian = first_cloud->is_bigendian;
            accumulated->fields = first_cloud->fields;
            accumulated->point_step = first_cloud->point_step;
            
            // Reserve space
            size_t total_points = 0;
            for (const auto& cloud_data : cloud_buffer_) {
                total_points += cloud_data.cloud->width;
            }
            accumulated->data.reserve(total_points * accumulated->point_step);
            
            // Combine clouds with simple voxel filtering
            std::unordered_set<uint64_t> voxel_set;  // Simple voxel grid
            
            for (const auto& cloud_data : cloud_buffer_) {
                sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_data.cloud, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_data.cloud, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_data.cloud, "z");
                
                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    // Simple voxel grid hash
                    int vx = static_cast<int>(*iter_x / voxel_size_);
                    int vy = static_cast<int>(*iter_y / voxel_size_);
                    int vz = static_cast<int>(*iter_z / voxel_size_);
                    uint64_t voxel_key = (static_cast<uint64_t>(vx) << 20) | 
                                       (static_cast<uint64_t>(vy) << 10) | 
                                       static_cast<uint64_t>(vz);
                    
                    if (voxel_set.find(voxel_key) == voxel_set.end()) {
                        voxel_set.insert(voxel_key);
                        
                        // Copy point
                        const uint8_t* point_ptr = reinterpret_cast<const uint8_t*>(&(*iter_x)) - 
                                                 cloud_data.cloud->fields[0].offset;
                        accumulated->data.insert(accumulated->data.end(), 
                                               point_ptr, point_ptr + accumulated->point_step);
                        accumulated->width++;
                    }
                }
            }
            
            accumulated->row_step = accumulated->width * accumulated->point_step;
        }
        
        pub_->publish(*accumulated);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published accumulated cloud with %u points from %zu source clouds", 
                    accumulated->width, cloud_buffer_.size());
    }
    
    // Parameters
    int max_clouds_;
    rclcpp::Duration max_age_;
    std::string target_frame_;
    double voxel_size_;
    double min_height_, max_height_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data storage
    std::deque<CloudData> cloud_buffer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImprovedCloudAccumulation>());
    rclcpp::shutdown();
    return 0;
} 