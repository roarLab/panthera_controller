#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <panthera_controller/speed_limiter.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.hpp>

namespace panthera_controller {

class PantheraController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::VelocityJointInterface,
          hardware_interface::PositionJointInterface> {
 public:
  PantheraController();

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  std::string name_;

  struct Commands {
    double lin_x;
    double lin_y;
    double ang_z;
    ros::Time stamp;

    Commands() : lin_x(0.0), lin_y(0.0), ang_z(0.0), stamp(0.0) {}
  };

  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  ros::Subscriber sub_command_;

  /// Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;

  /// Speed limiters:
  Commands last1_cmd_;
  Commands last0_cmd_;
  SpeedLimiter limiter_lin_x_;
  SpeedLimiter limiter_lin_y_;
  SpeedLimiter limiter_ang_z_;

  void cmdVelCallback(const geometry_msgs::Twist& msg);
};

PLUGINLIB_EXPORT_CLASS(panthera_controller::PantheraController,
                       controller_interface::ControllerBase);
}  // namespace panthera_controller
