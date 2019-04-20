#ifndef ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_IMU_PUBLISHER_H
#define ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_IMU_PUBLISHER_H

#include <sensor_msgs/Imu.h>

#include <ros_node_transformation/device/publisher/base_publisher.h>

namespace device {
namespace publisher {

/**
 * @brief Device publisher for IMU data.
 * @details Parses a `roll,pitch,yaw,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z` string
 * command and publishes a sensor_msgs::Imu message.
 */
class ImuPublisher : public BasePublisher {
  struct ImuData {
    double roll;
    double pitch;
    double yaw;
    double vel[3];
    double acc[3];
  };

 public:
  ImuPublisher() {}

  virtual ~ImuPublisher() override {}

  /**
   * @brief Initializes the publisher.
   */
  virtual bool init(ros::NodeHandle &root_nh,
                    ros::NodeHandle &publisher_nh) override;

  /**
   * @brief Parses the given data and publishes an Imu message.
   */
  virtual void publish(const std::string &data) override;

 private:
  /**
   * @brief Initializes the Imu message.
   */
  void initMsg(const std::string &frame_id);

  /**
   * @brief Updates the Imu message.
   */
  void updateMsg(const ImuData &data);

  /**
   * @brief Parses the given string data.
   */
  ImuData parse(const std::string &data) const;

  sensor_msgs::Imu msg_;
};

using ImuPublisherPtr = boost::shared_ptr<ImuPublisher>;
using ImuPublisherConstPtr = boost::shared_ptr<const ImuPublisher>;

}  // namespace publisher
}  // namespace device

#endif  // ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_IMU_PUBLISHER_H
