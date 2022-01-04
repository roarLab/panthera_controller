#pragma once

#include <angles/angles.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <panthera_controller/speed_limiter.h>
#include <panthera_msgs/State.h>
#include <panthera_msgs/TwistWithReconfiguration.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>

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

  void starting(const ros::Time& time) override;

  void stopping(const ros::Time& /*time*/) override;

 private:
  std::string name_;

  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  ros::Time last_odom_update_timestamp_;

  /// Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  struct Commands {
    double lin_x;
    double lin_y;
    double ang_z;
    double left_gait;
    double right_gait;
    ros::Time stamp;

    Commands()
        : lin_x(0.0),
          lin_y(0.0),
          ang_z(0.0),
          left_gait(0.0),
          right_gait(0.0),
          stamp(0.0) {}
  };

  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  ros::Subscriber sub_command_;

  /// Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;

  /// Frame to use for the robot base:
  std::string base_frame_id_;

  /// Wheel separation b (or track width), distance between left and right
  /// wheels (from the midpoint of the wheel width):
  double track_width_;

  /// Wheel radius (assuming it's the same for the left and right wheels):
  double wheel_radius_;

  /// Wheel base a (distance between front and rear wheel):
  double wheel_base_;

  // odometry related
  std::string odom_frame_id_;
  std::string state_frame_id_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> >
      odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<panthera_msgs::State> >
      state_pub_;

  /// Hardware handles:
  std::vector<hardware_interface::JointHandle> front_wheel_joints_;
  std::vector<hardware_interface::JointHandle> rear_wheel_joints_;
  std::vector<hardware_interface::JointHandle> front_steering_joints_;
  std::vector<hardware_interface::JointHandle> rear_steering_joints_;
  std::vector<hardware_interface::JointHandle> reconfiguration_joints_;

  /// Speed limiters:
  Commands last1_cmd_;
  Commands last0_cmd_;
  SpeedLimiter limiter_lin_x_;
  SpeedLimiter limiter_lin_y_;
  SpeedLimiter limiter_ang_z_;

  void cmdVelCallback(const panthera_msgs::TwistWithReconfiguration& msg);
  bool getWheelNames(ros::NodeHandle& controller_nh,
                     const std::string& wheel_param,
                     std::vector<std::string>& wheel_names);

  void brake();
  double normalize_angle(double angle);
};

PLUGINLIB_EXPORT_CLASS(panthera_controller::PantheraController,
                       controller_interface::ControllerBase);
}  // namespace panthera_controller
