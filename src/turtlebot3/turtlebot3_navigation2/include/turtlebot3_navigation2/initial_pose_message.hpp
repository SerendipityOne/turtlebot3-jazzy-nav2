#ifndef TURTLEBOT3_NAVIGATION2__INITIAL_POSE_MESSAGE_HPP_
#define TURTLEBOT3_NAVIGATION2__INITIAL_POSE_MESSAGE_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace turtlebot3_navigation2
{

geometry_msgs::msg::PoseWithCovarianceStamped makeInitialPose(
  double x, double y, double yaw);

}  // namespace turtlebot3_navigation2

#endif  // TURTLEBOT3_NAVIGATION2__INITIAL_POSE_MESSAGE_HPP_
