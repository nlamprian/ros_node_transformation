#ifndef ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_BASE_PUBLISHER_H
#define ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_BASE_PUBLISHER_H

#include <string>

#include <ros/ros.h>

namespace device {
namespace publisher {

/**
 * @brief Device publisher interface. All derived publishers must override the
 * `init` and `publish` methods.
 */
class BasePublisher {
 public:
  BasePublisher() {}
  virtual ~BasePublisher() {}
  virtual bool init(ros::NodeHandle &root_nh,
                    ros::NodeHandle &publisher_nh) = 0;
  virtual void publish(const std::string &data) = 0;

 protected:
  ros::Publisher publisher_;
};

using BasePublisherPtr = boost::shared_ptr<BasePublisher>;
using BasePublisherConstPtr = boost::shared_ptr<const BasePublisher>;

}  // namespace publisher
}  // namespace device

#endif  // ROS_NODE_TRANSFORMATION_DEVICE_PUBLISHER_BASE_PUBLISHER_H
