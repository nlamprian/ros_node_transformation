#include <algorithm>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <ros_node_transformation/device/publisher/imu_publisher.h>

namespace device {
namespace publisher {

bool ImuPublisher::init(ros::NodeHandle &root_nh,
                        ros::NodeHandle &publisher_nh) {
  // Get topic name
  if (not publisher_nh.hasParam("topic_name")) {
    ROS_ERROR("ImuPublisher: Failed to get topic_name");
    return false;
  }
  std::string topic_name;
  publisher_nh.getParam("topic_name", topic_name);

  // Get frame id
  if (not publisher_nh.hasParam("frame_id")) {
    ROS_ERROR("ImuPublisher: Failed to get frame_id");
    return false;
  }
  std::string frame_id;
  publisher_nh.getParam("frame_id", frame_id);

  // Initialize topic
  initMsg(frame_id);
  publisher_ = root_nh.advertise<sensor_msgs::Imu>(topic_name, 1);

  return true;
}

void ImuPublisher::publish(const std::string &data) {
  try {
    ImuData imu_data = parse(data);
    updateMsg(imu_data);
  } catch (const std::exception &error) {
    ROS_ERROR_THROTTLE(1, "ImuPublisher: Failed to parse data - %s",
                       error.what());
    return;
  }

  publisher_.publish(msg_);
}

void ImuPublisher::initMsg(const std::string &frame_id) {
  msg_.header.frame_id = frame_id;
  msg_.orientation_covariance[0] = 0.01;
  msg_.orientation_covariance[4] = 0.01;
  msg_.orientation_covariance[8] = 0.01;
  msg_.angular_velocity_covariance[0] = 0.01;
  msg_.angular_velocity_covariance[4] = 0.01;
  msg_.angular_velocity_covariance[8] = 0.01;
  msg_.linear_acceleration_covariance[0] = 0.01;
  msg_.linear_acceleration_covariance[4] = 0.01;
  msg_.linear_acceleration_covariance[8] = 0.01;
}

void ImuPublisher::updateMsg(const ImuPublisher::ImuData &data) {
  msg_.header.stamp = ros::Time::now();
  msg_.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(data.roll, data.pitch, data.yaw);
  msg_.angular_velocity.x = data.vel[0];
  msg_.angular_velocity.y = data.vel[1];
  msg_.angular_velocity.z = data.vel[2];
  msg_.linear_acceleration.x = data.acc[0];
  msg_.linear_acceleration.y = data.acc[1];
  msg_.linear_acceleration.z = data.acc[2];
}

ImuPublisher::ImuData ImuPublisher::parse(const std::string &data) const {
  // Parse data
  std::vector<std::string> string_values;
  boost::split(string_values, data, [](char c) { return c == ','; });
  if (string_values.size() != 9)
    throw std::runtime_error("Wrong number of values: " +
                             std::to_string(string_values.size()));
  std::vector<double> values(string_values.size());
  std::transform(
      string_values.begin(), string_values.end(), values.begin(),
      [](const std::string &str) -> double { return std::stod(str); });

  // Get data
  ImuData imu_data;
  imu_data.roll = values[0];
  imu_data.pitch = values[1];
  imu_data.yaw = values[2];
  imu_data.vel[0] = values[3];
  imu_data.vel[1] = values[4];
  imu_data.vel[2] = values[5];
  imu_data.acc[0] = values[6];
  imu_data.acc[1] = values[7];
  imu_data.acc[2] = values[8];

  return imu_data;
}

}  // namespace publisher
}  // namespace device

PLUGINLIB_EXPORT_CLASS(device::publisher::ImuPublisher,
                       device::publisher::BasePublisher)
