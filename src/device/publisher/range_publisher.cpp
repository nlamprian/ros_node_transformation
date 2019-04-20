#include <string>

#include <pluginlib/class_list_macros.h>

#include <ros_node_transformation/device/publisher/range_publisher.h>

namespace device {
namespace publisher {

bool RangePublisher::init(ros::NodeHandle &root_nh,
                          ros::NodeHandle &publisher_nh) {
  // Get topic name
  if (not publisher_nh.hasParam("topic_name")) {
    ROS_ERROR("RangePublisher: Failed to get topic_name");
    return false;
  }
  std::string topic_name;
  publisher_nh.getParam("topic_name", topic_name);

  // Get frame id
  if (not publisher_nh.hasParam("frame_id")) {
    ROS_ERROR("RangePublisher: Failed to get frame_id");
    return false;
  }
  std::string frame_id;
  publisher_nh.getParam("frame_id", frame_id);

  // Get radiation type
  if (not publisher_nh.hasParam("radiation_type")) {
    ROS_ERROR("RangePublisher: Failed to get radiation_type");
    return false;
  }
  int radiation_type;
  publisher_nh.getParam("radiation_type", radiation_type);

  // Get field of view
  if (not publisher_nh.hasParam("field_of_view")) {
    ROS_ERROR("RangePublisher: Failed to get field_of_view");
    return false;
  }
  float field_of_view;
  publisher_nh.getParam("field_of_view", field_of_view);

  // Get limits
  if (not publisher_nh.hasParam("limits")) {
    ROS_ERROR("RangePublisher: Failed to get limits");
    return false;
  }
  std::vector<float> limits;
  publisher_nh.getParam("limits", limits);
  if (limits.size() != 2) {
    ROS_ERROR("RangePublisher: Limits size should be 2");
    return false;
  }

  // Initialize topic
  initMsg(frame_id, radiation_type, field_of_view, limits);
  publisher_ = root_nh.advertise<sensor_msgs::Range>(topic_name, 1);

  return true;
}

void RangePublisher::publish(const std::string &data) {
  try {
    double range = std::stod(data);
    updateMsg(range);
  } catch (const std::exception &error) {
    ROS_ERROR_THROTTLE(1, "ImuPublisher: Failed to parse data - %s",
                       error.what());
    return;
  }

  publisher_.publish(msg_);
}

void RangePublisher::initMsg(const std::string &frame_id, int radiation_type,
                             float field_of_view,
                             const std::vector<float> &limits) {
  msg_.header.frame_id = frame_id;
  msg_.radiation_type = radiation_type;
  msg_.field_of_view = field_of_view;
  msg_.min_range = limits[0];
  msg_.max_range = limits[1];
}

void RangePublisher::updateMsg(double range) {
  msg_.header.stamp = ros::Time::now();
  msg_.range = range;
}

}  // namespace publisher
}  // namespace device

PLUGINLIB_EXPORT_CLASS(device::publisher::RangePublisher,
                       device::publisher::BasePublisher)
