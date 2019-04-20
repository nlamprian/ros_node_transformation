#ifndef ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_RANGE_PUBLISHER_H
#define ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_RANGE_PUBLISHER_H

#include <sensor_msgs/Range.h>

#include <ros_node_transformation/device/publisher/base_publisher.h>

namespace device {
namespace publisher {

/**
 * @brief Device publisher for Range data.
 * @details Parses a `range` string command and publishes a sensor_msgs::Range
 * message.
 */
class RangePublisher : public BasePublisher {
 public:
  RangePublisher() {}

  virtual ~RangePublisher() override {}

  /**
   * @brief Initializes the publisher.
   */
  virtual bool init(ros::NodeHandle &root_nh,
                    ros::NodeHandle &publisher_nh) override;

  /**
   * @brief Parses the given data and publishes a Range message.
   */
  virtual void publish(const std::string &data) override;

 private:
  /**
   * @brief Initializes the Range message.
   */
  void initMsg(const std::string &frame_id, int radiation_type,
               float field_of_view, const std::vector<float> &limits);

  /**
   * @brief Updates the Range message.
   */
  void updateMsg(double range);

  sensor_msgs::Range msg_;
};

using RangePublisherPtr = boost::shared_ptr<RangePublisher>;
using RangePublisherConstPtr = boost::shared_ptr<const RangePublisher>;

}  // namespace publisher
}  // namespace device

#endif  // ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_RANGE_PUBLISHER_H
