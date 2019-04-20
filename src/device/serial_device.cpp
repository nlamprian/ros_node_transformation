#include <ros_node_transformation/device/serial_device.h>

namespace device {

SerialDevice::SerialDevice(const std::string &name, ros::NodeHandle &nh)
    : Device(name, nh) {}

SerialDevice::~SerialDevice() {
  ROS_INFO("SerialDevice[%s]: Destroyed device", name_.c_str());
}

bool SerialDevice::init(ros::NodeHandle &device_nh) {
  Device::init(device_nh);

  // Get port
  if (not device_nh.hasParam("port")) {
    ROS_ERROR("SerialDevice[%s]: Failed to get port", name_.c_str());
    return false;
  }
  std::string port;
  device_nh.getParam("port", port);

  // Initialize serial handler
  serial_handler_ = std::make_shared<serial::SerialHandler>(port);

  // Get device publisher
  ros::NodeHandle publisher_nh(device_nh, "publisher");
  data_publisher_ = loadDevicePublisher(publisher_nh);
  if (not data_publisher_) {
    ROS_ERROR("SerialDevice[%s]: Failed to get device publisher",
              name_.c_str());
    return false;
  }

  // Get publish rate
  if (not device_nh.hasParam("publish_rate")) {
    ROS_ERROR("SerialDevice[%s]: Failed to get publish_rate", name_.c_str());
    return false;
  }
  double publish_rate;
  device_nh.getParam("publish_rate", publish_rate);
  data_publish_period_ = ros::Duration(1 / publish_rate);

  // Register diagnostic statuses
  registerDiagnosticStatus("status", this, &SerialDevice::getStatusDiagnostics);
  registerDiagnosticStatus("time", this, &SerialDevice::getTimeDiagnostics);

  // Initialize publish timer
  publish_timer_ = nh_.createTimer(ros::Rate(200),
                                   &SerialDevice::publishTimerCallback, this);

  ROS_INFO("SerialDevice[%s]: Initialized successfully", name_.c_str());
  return true;
}

void SerialDevice::getStatusDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (serial_handler_->isActive())
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "Serial port is active");
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "Serial port is not active");
}

void SerialDevice::getTimeDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  ros::Duration time = ros::Time::now() - last_data_publish_time_;
  if (time < ros::Duration(1))
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No recent data");

  stat.add("Time since last data", time.toSec());
}

void SerialDevice::publishTimerCallback(const ros::TimerEvent & /*event*/) {
  // Publish data
  if (serial_handler_->hasData() and
      ros::Time::now() - last_data_publish_time_ >= data_publish_period_) {
    std::string data = serial_handler_->getData();
    data_publisher_->publish(data);
    last_data_publish_time_ = ros::Time::now();
  }

  publishDiagnostics();
}

}  // namespace device
