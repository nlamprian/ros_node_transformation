#ifndef ROS_NODE_TRANSFORMATION_DEVICE_DEVICE_H
#define ROS_NODE_TRANSFORMATION_DEVICE_DEVICE_H

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <ros_node_transformation/device/publisher/base_publisher.h>

namespace device {

/**
 * @brief Device interface. All derived devices must override the `init` method.
 */
class Device {
 public:
  Device(const std::string &name, ros::NodeHandle &nh)
      : name_(name),
        nh_(nh),
        publisher_loader_("ros_node_transformation",
                          "device::publisher::BasePublisher") {}

  virtual ~Device() {}

  /**
   * @brief Gets the device name.
   */
  const std::string &getName() const { return name_; }

  /**
   * @brief Initializes the device.
   */
  virtual bool init(ros::NodeHandle &device_nh) {
    // Get hardware id
    if (not device_nh.hasParam("hardware_id")) {
      ROS_ERROR("SerialDevice[%s]: Failed to get hardware_id", name_.c_str());
      return false;
    }
    std::string hardware_id;
    device_nh.getParam("hardware_id", hardware_id);

    // Initialize diagnostic updater
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(
        nh_, device_nh, ros::this_node::getName() + ":" + name_);
    diagnostic_updater_->setHardwareID(hardware_id);

    return true;
  }

  /**
   * @brief Creates and initializes a device publisher with the parameters in
   * the given namespace.
   */
  publisher::BasePublisherPtr loadDevicePublisher(
      ros::NodeHandle &publisher_nh) {
    // Get type
    if (not publisher_nh.hasParam("type")) {
      ROS_ERROR("SerialDevice[%s]: Failed to get publisher type",
                name_.c_str());
      return nullptr;
    }
    std::string type;
    publisher_nh.getParam("type", type);

    // Create publisher
    publisher::BasePublisherPtr publisher;
    try {
      publisher = publisher_loader_.createInstance(type);
    } catch (const pluginlib::PluginlibException &error) {
      ROS_ERROR("SerialDevice[%s]: Failed to load publisher plugin\n%s",
                name_.c_str(), error.what());
      return nullptr;
    }

    // Initialize  publisher
    bool inited = publisher->init(nh_, publisher_nh);
    if (not inited) {
      ROS_ERROR("SerialDevice[%s]: Failed to initialize publisher",
                name_.c_str());
      return nullptr;
    }

    return publisher;
  }

  /**
   * @brief Registers a diagnostic status with the diagnostic updater.
   */
  template <typename T>
  void registerDiagnosticStatus(
      const std::string &name, T *object,
      void (T::*callback)(diagnostic_updater::DiagnosticStatusWrapper &)) {
    diagnostic_updater_->add(name, object, callback);
  }

  /**
   * @brief Publishes diagnostics at diagnostic_period period.
   */
  void publishDiagnostics() { diagnostic_updater_->update(); }

 protected:
  std::string name_;

  ros::NodeHandle nh_;

 private:
  pluginlib::ClassLoader<publisher::BasePublisher> publisher_loader_;

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

using DevicePtr = std::shared_ptr<Device>;
using DeviceConstPtr = std::shared_ptr<const Device>;

}  // namespace device

#endif  // ROS_NODE_TRANSFORMATION_DEVICE_DEVICE_H
