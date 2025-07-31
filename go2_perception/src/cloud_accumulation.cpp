#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


sensor_msgs::msg::PointCloud2 accumulated_cloud;
std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;

void cloudCallback(
sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
    // Directly add the new point cloud to clouds
    clouds.push_back(cloud_msg);

    // If the number of point clouds in clouds exceeds a certain limit, delete the oldest point cloud
    if (clouds.size() > 30)
    {
        clouds.erase(clouds.begin());
    }

    // Create a new point cloud to store the merged point cloud
    auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    *merged_cloud = *clouds[0];

    // Traverse clouds and merge all point clouds into merged_cloud
    for (size_t it = 1; it < clouds.size(); it++)
    {
        merged_cloud->width += clouds[it]->width;
        merged_cloud->row_step += clouds[it]->row_step;
        merged_cloud->data.insert(merged_cloud->data.end(), clouds[it]->data.begin(), clouds[it]->data.end());
    }

    // Filter out points with height above 1.0 or below 0.2
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = merged_cloud->header;
    filtered_cloud.height = 1;
    filtered_cloud.is_dense = false;
    filtered_cloud.is_bigendian = merged_cloud->is_bigendian;
    filtered_cloud.fields = merged_cloud->fields;
    filtered_cloud.point_step = merged_cloud->point_step;
    filtered_cloud.row_step = 0;

    // Traverse point cloud data
    for (size_t i = 0; i < merged_cloud->width * merged_cloud->height; i++)
    {
        float x, y, z;
        memcpy(&x, &merged_cloud->data[i * merged_cloud->point_step + merged_cloud->fields[0].offset], sizeof(float));
        memcpy(&y, &merged_cloud->data[i * merged_cloud->point_step + merged_cloud->fields[1].offset], sizeof(float));
        memcpy(&z, &merged_cloud->data[i * merged_cloud->point_step + merged_cloud->fields[2].offset], sizeof(float));

        // Only keep points with height between 0.2 and 1.0
        if (z >= 0.2 && z <= 1.0) {
            filtered_cloud.data.insert(filtered_cloud.data.end(),
                                       &merged_cloud->data[i * merged_cloud->point_step],
                                       &merged_cloud->data[(i + 1) * merged_cloud->point_step]);
            filtered_cloud.row_step += merged_cloud->point_step;
            filtered_cloud.width++;
        }
    }

    accumulated_cloud = filtered_cloud;
    accumulated_cloud.header.frame_id = "odom";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.reliable();

  auto node = std::make_shared<rclcpp::Node>("cloud_accumulations");
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos);

  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/point_cloud2", rclcpp::SensorDataQoS(),
    [](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate rate(50.0);
  while (rclcpp::ok()) {
    accumulated_cloud.header.stamp = node->get_clock()->now();
    pub->publish(accumulated_cloud);

    executor.spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
