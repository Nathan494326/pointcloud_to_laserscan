

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher() : Node("point_cloud_publisher"), count_(0)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&PointCloudPublisher::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
  }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Create a new PointCloud2 message to hold the filtered points
  sensor_msgs::msg::PointCloud2 filtered_msg;
  filtered_msg.header = msg->header;
  filtered_msg.height = 1;
  filtered_msg.width = 0;
  filtered_msg.fields = msg->fields;
  filtered_msg.is_bigendian = msg->is_bigendian;
  filtered_msg.point_step = msg->point_step;
  filtered_msg.row_step = msg->row_step;
  filtered_msg.is_dense = msg->is_dense;

  // Print the total number of points in the received message
  RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", msg->data.size() / msg->point_step);

  if (count_ % 10 == 0) {
    // Iterate over the points in the received message
    for (size_t i = 0; i < msg->data.size(); i += msg->point_step) {
      // If the current point is one that we want to publish, add it to the filtered message
      filtered_msg.data.insert(filtered_msg.data.end(), msg->data.begin() + i, msg->data.begin() + i + msg->point_step);
      filtered_msg.width++;
    }
    // Publish the filtered message
    publisher_->publish(filtered_msg);
  }
  count_++;
}

    
    

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
