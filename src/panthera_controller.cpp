#include <panthera_controller/panthera_controller.h>

namespace panthera_controller {
PantheraController::PantheraController()
    : command_struct_(), cmd_vel_timeout_(0.5) {}

bool PantheraController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Twist command related:
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  ROS_INFO_STREAM_NAMED(
      name_, "Velocity commands will be considered old if they are older than "
                 << cmd_vel_timeout_ << "s.");

  sub_command_ = controller_nh.subscribe(
      "cmd_vel", 1, &PantheraController::cmdVelCallback, this);

  return true;
}

void PantheraController::update(const ros::Time &time,
                                const ros::Duration &period) {
  // MOVE ROBOT
  // Retreive current velocity command and time step
  Commands curr_cmd = *(command_.readFromRT());
  const double dt = (time - curr_cmd.stamp).toSec();

  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_) {
    curr_cmd.lin_x = 0.0;
    curr_cmd.lin_y = 0.0;
    curr_cmd.ang_z = 0.0;
  }

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());

  limiter_lin_x_.limit(curr_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x,
                       cmd_dt);
  limiter_lin_y_.limit(curr_cmd.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y,
                       cmd_dt);
  limiter_ang_z_.limit(curr_cmd.ang_z, last0_cmd_.ang_z, last1_cmd_.ang_z,
                       cmd_dt);
  last1_cmd_ = last0_cmd_;
  last0_cmd_ = curr_cmd;

  //////////////////////////////////////////////////////
  // TODO: compute wheel velocities and steering angles
  //////////////////////////////////////////////////////

  //////////////////////////////////////////////////////
}

void PantheraController::cmdVelCallback(const geometry_msgs::Twist &msg) {
  if (isRunning()) {
    if (std::isnan(msg.linear.x) || std::isnan(msg.linear.y) ||
        std::isnan(msg.angular.z)) {
      ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
      return;
    }
    command_struct_.ang_z = msg.angular.z;
    command_struct_.lin_x = msg.linear.x;
    command_struct_.lin_y = msg.linear.y;
    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT(command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Added values to command. "
                               << "Ang: " << command_struct_.ang_z << ", "
                               << "Lin x: " << command_struct_.lin_x << ", "
                               << "Lin y: " << command_struct_.lin_y << ", "
                               << "Stamp: " << command_struct_.stamp);
  } else {
    ROS_ERROR_NAMED(name_,
                    "Can't accept new commands. Controller is not running.");
  }
}
}  // namespace panthera_controller