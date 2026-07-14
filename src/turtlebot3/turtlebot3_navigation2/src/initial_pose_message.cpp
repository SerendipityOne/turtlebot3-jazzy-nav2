#include "turtlebot3_navigation2/initial_pose_message.hpp"

#include <cmath>

namespace turtlebot3_navigation2
{

geometry_msgs::msg::PoseWithCovarianceStamped makeInitialPose(
  const double x, const double y, const double yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = "map";
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = y;
  message.pose.pose.orientation.z = std::sin(yaw / 2.0);
  message.pose.pose.orientation.w = std::cos(yaw / 2.0);
  message.pose.covariance[0] = 0.01;
  message.pose.covariance[7] = 0.01;
  message.pose.covariance[35] = 0.01;
  return message;
}

}  // namespace turtlebot3_navigation2
